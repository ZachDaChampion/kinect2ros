#include "kinect2ros/kinect2ros.hpp"

#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;
using namespace camera_info_manager;
using namespace libfreenect2;
using namespace kinect2ros;
using namespace std;

static const bool IS_BIGENDIAN = false;  // TODO: Determine this programmatically

//                                                                                                //
// ======================================== Constructor ========================================= //
//                                                                                                //

Kinect2RosNode::Kinect2RosNode()
{
  node = rclcpp::Node::make_shared("kinect2ros");
  param_listener_ = make_unique<ParamListener>(node);
  params_ = make_unique<Params>(param_listener_->get_params());
  auto logger = node->get_logger();

  // Publish static transform if enabled.
  if (params_->transform.enabled) {
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node->get_clock()->now();
    t.header.frame_id = params_->depth_frame;
    t.child_frame_id = params_->color_frame;
    t.transform.translation.x = params_->transform.translation[0];
    t.transform.translation.y = params_->transform.translation[1];
    t.transform.translation.z = params_->transform.translation[2];
    t.transform.rotation.x = 0;
    t.transform.rotation.y = 0;
    t.transform.rotation.z = 0;
    t.transform.rotation.w = 1;
    static_tf_broadcaster_->sendTransform(t);
  }

  // Create the image transport publishers.
  color_cinfo_pub_ = image_transport::create_camera_publisher(node.get(), params_->base_topic +
                                                                        "/color/"
                                                                        "image_raw");
  depth_cinfo_pub_ = image_transport::create_camera_publisher(node.get(), params_->base_topic +
                                                                        "/depth/"
                                                                        "image_raw");
  ir_cinfo_pub_ =
      image_transport::create_camera_publisher(node.get(), params_->base_topic + "/ir/image_raw");

  // Create the camera info managers.
  color_cinfo_manager_ = std::make_shared<CameraInfoManager>(node.get());
  depth_cinfo_manager_ = std::make_shared<CameraInfoManager>(node.get());
  color_cinfo_manager_->loadCameraInfo(params_->calibration_files.color);
  depth_cinfo_manager_->loadCameraInfo(params_->calibration_files.depth);

  if (params_->point_cloud.enabled) {
    // Initialize the pointcloud message with 4 fields (x, y, z, rgb).
    pointcloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pointcloud_msg_->header.frame_id = params_->depth_frame;
    pointcloud_msg_->height = DEPTH_HEIGHT;
    pointcloud_msg_->width = DEPTH_WIDTH;
    pointcloud_msg_->fields.reserve(4);
    pointcloud_msg_->fields.emplace_back(
        create_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
    pointcloud_msg_->fields.emplace_back(
        create_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
    pointcloud_msg_->fields.emplace_back(
        create_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
    pointcloud_msg_->fields.emplace_back(
        create_point_field("rgb", 12, sensor_msgs::msg::PointField::FLOAT32, 1));
    pointcloud_msg_->is_bigendian = IS_BIGENDIAN;
    pointcloud_msg_->point_step = 16;
    pointcloud_msg_->row_step = pointcloud_msg_->point_step * DEPTH_WIDTH;
    pointcloud_msg_->data.reserve(pointcloud_msg_->row_step * DEPTH_HEIGHT);
    pointcloud_msg_->is_dense = false;

    // Create the pointcloud publisher.
    pointcloud_pub_ =
        node->create_publisher<sensor_msgs::msg::PointCloud2>(params_->base_topic + "/points", 10);
  }

  // Initialize the Kinect.
  string device_id = params_->device_id;
  if (device_id.empty()) {
    device_id = freenect2_.getDefaultDeviceSerialNumber();
    if (device_id.empty()) {
      RCLCPP_ERROR(logger, "No Kinect found");
      return;
    }
  }
  device_ = freenect2_.openDevice(device_id);
  if (device_ == nullptr) {
    RCLCPP_ERROR(logger, "Failed to open Kinect");
    return;
  }

  // Setup the frame listener.
  listener_ = new SyncMultiFrameListener(Frame::Color | Frame::Depth | Frame::Ir);
  device_->setColorFrameListener(listener_);
  device_->setIrAndDepthFrameListener(listener_);
  undistorted_ = new Frame(DEPTH_WIDTH, DEPTH_HEIGHT, 4);
  registered_ = new Frame(DEPTH_WIDTH, DEPTH_HEIGHT, 4);

  // Start the device.
  device_->start();
  RCLCPP_INFO(logger, "Kinect started");

  // Setup registration if pointcloud is enabled.
  if (params_->point_cloud.enabled) {
    registration_ = new Registration(device_->getIrCameraParams(), device_->getColorCameraParams());
    RCLCPP_INFO(logger, "Registration initialized");
  }
}

//                                                                                                //
// ========================================= Destructor ========================================= //
//                                                                                                //

Kinect2RosNode::~Kinect2RosNode()
{
  listener_->release(frames_);
  device_->stop();
  device_->close();

  delete device_;
  delete listener_;
  delete registration_;
  delete undistorted_;
  delete registered_;
}

//                                                                                                //
// ========================================== is_okay =========================================== //
//                                                                                                //

bool Kinect2RosNode::is_okay()
{
  if (device_ == nullptr) return false;
  if (listener_ == nullptr) return false;
  if (params_->point_cloud.enabled && registration_ == nullptr) return false;
  return true;
}

//                                                                                                //
// =========================================== update =========================================== //
//                                                                                                //

bool Kinect2RosNode::update(int timeout)
{
  if (!is_okay()) {
    RCLCPP_ERROR(node->get_logger(), "Cannot update; node is not healthy");
    return false;
  }

  // Receive frames from Kinect.
  if (!listener_->waitForNewFrame(frames_, timeout)) {
    RCLCPP_ERROR(node->get_logger(), "Timeout when receiving frame from Kinect");
    return false;
  }
  rclcpp::Time now = node->get_clock()->now();
  Frame* color_frame = frames_[Frame::Color];
  Frame* depth_frame = frames_[Frame::Depth];
  Frame* ir_frame = frames_[Frame::Ir];

  // Convert color frame to OpenCV mat.
  cv::Mat color_mat_rgba(color_frame->height, color_frame->width, CV_8UC4, color_frame->data);
  static cv::Mat color_mat_rgb(color_frame->height, color_frame->width, CV_8UC3);
  cv::cvtColor(color_mat_rgba, color_mat_rgb, cv::COLOR_RGBA2RGB);
  static cv::Mat color_mat_flipped(color_frame->height, color_frame->width, CV_8UC3);
  cv::flip(color_mat_rgb, color_mat_flipped, 1);

  // Convert depth frame to OpenCV mat.
  cv::Mat depth_mat(depth_frame->height, depth_frame->width, CV_32FC1, depth_frame->data);
  static cv::Mat depth_mat_flipped(depth_frame->height, depth_frame->width, CV_32FC1);
  cv::flip(depth_mat, depth_mat_flipped, 1);

  // Conver IR frame to OpenCV mat.
  cv::Mat ir_mat(ir_frame->height, ir_frame->width, CV_32FC1, ir_frame->data);
  static cv::Mat ir_mat_flipped(ir_frame->height, ir_frame->width, CV_32FC1);
  cv::flip(ir_mat, ir_mat_flipped, 1);

  // Create Image messages.
  color_msg_ = mat_to_msg(color_mat_flipped, "bgr8");
  depth_msg_ = mat_to_msg(depth_mat_flipped, "32FC1");
  ir_msg_ = mat_to_msg(ir_mat_flipped, "32FC1");

  // Create CameraInfo messages.
  sensor_msgs::msg::CameraInfo::SharedPtr color_cinfo_msg(
      new sensor_msgs::msg::CameraInfo(color_cinfo_manager_->getCameraInfo()));
  sensor_msgs::msg::CameraInfo::SharedPtr depth_cinfo_msg(
      new sensor_msgs::msg::CameraInfo(depth_cinfo_manager_->getCameraInfo()));

  // Set the frame ID and timestamp for the messages.
  color_msg_->header.frame_id = params_->color_frame;
  color_msg_->header.stamp = now;
  color_cinfo_msg->header.frame_id = params_->color_frame;
  color_cinfo_msg->header.stamp = now;
  depth_msg_->header.frame_id = params_->depth_frame;
  depth_msg_->header.stamp = now;
  depth_cinfo_msg->header.frame_id = params_->depth_frame;
  depth_cinfo_msg->header.stamp = now;
  ir_msg_->header.frame_id = params_->depth_frame;
  ir_msg_->header.stamp = now;

  // Publish images and camera info.
  color_cinfo_pub_.publish(color_msg_, color_cinfo_msg);
  depth_cinfo_pub_.publish(depth_msg_, depth_cinfo_msg);
  ir_cinfo_pub_.publish(ir_msg_, depth_cinfo_msg);

  if (params_->point_cloud.enabled) {
    // Apply registration to map the color frame onto the depth frame.
    registration_->apply(color_frame, depth_frame, undistorted_, registered_, params_->point_cloud.filter);

    // Construct pointcloud.
    pointcloud_msg_->header.stamp = now;
    auto data = &pointcloud_msg_->data;
    data->clear();
    data->reserve(undistorted_->height * undistorted_->width);
    for (size_t row = 0; row < undistorted_->height; ++row) {
      for (size_t col = 0; col < undistorted_->width; ++col) {
        // Calculate point.
        float new_point[4];
        registration_->getPointXYZRGB(undistorted_, registered_, row, col, new_point[0],
                                      new_point[1], new_point[2], new_point[3]);

        // Add to cloud.
        new_point[0] *= -1;
        const uint8_t* new_point_u8 = reinterpret_cast<uint8_t*>(new_point);
        data->insert(data->end(), new_point_u8, new_point_u8 + 16);
      }
    }

    // Publish pointcloud.
    pointcloud_pub_->publish(*pointcloud_msg_);
  }

  // Release frames from the frame buffer.
  listener_->release(frames_);
  return true;
}

//                                                                                                //
// ===================================== create_point_field ===================================== //
//                                                                                                //

sensor_msgs::msg::PointField Kinect2RosNode::create_point_field(const std::string& name,
                                                                const uint32_t offset,
                                                                const uint8_t datatype,
                                                                const uint32_t count)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = count;
  return field;
}

//                                                                                                //
// ========================================= mat_to_msg ========================================= //
//                                                                                                //

std::shared_ptr<sensor_msgs::msg::Image> Kinect2RosNode::mat_to_msg(cv::Mat& frame,
                                                                    std::string encoding)
{
  std_msgs::msg::Header header;
  sensor_msgs::msg::Image image;

  image.header = header;
  image.height = frame.rows;
  image.width = frame.cols;
  image.encoding = encoding;
  image.is_bigendian = IS_BIGENDIAN;
  image.step = frame.cols * frame.elemSize();
  size_t size = image.step * frame.rows;
  image.data.resize(size);

  if (frame.isContinuous())
    memcpy(reinterpret_cast<uint8_t*>(&image.data[0]), frame.data, size);
  else {
    uint8_t* image_data = reinterpret_cast<uint8_t*>(&image.data[0]);
    uint8_t* frame_data = frame.data;
    for (int i = 0; i < frame.rows; ++i) {
      memcpy(image_data, frame_data, image.step);
      image_data += image.step;
      frame_data += frame.step;
    }
  }

  return std::make_shared<sensor_msgs::msg::Image>(image);
}