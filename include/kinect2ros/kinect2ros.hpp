#ifndef KINECT2ROS__KINECT2ROS_HPP_
#define KINECT2ROS__KINECT2ROS_HPP_

#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>

#include <libfreenect2/libfreenect2.hpp>

#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

class Kinect2RosNode : public rclcpp::Node
{
public:
  static const size_t DEPTH_WIDTH = 512;    //< Width of the depth image.
  static const size_t DEPTH_HEIGHT = 424;   //< Height of the depth image.
  static const size_t COLOR_WIDTH = 1920;   //< Width of the color image.
  static const size_t COLOR_HEIGHT = 1080;  //< Height of the color image.

  /**
   * Construct a new Kinect 2 Ros Node object.
   *
   * @param[in] options The options for the node.
   */
  explicit Kinect2RosNode(const rclcpp::NodeOptions& options);

  /**
   * Destroy the Kinect 2 Ros Node object.
   */
  ~Kinect2RosNode();

  /**
   * Returns whether the node is okay or not. If this returns false, something has gone wrong and
   * the node should be stopped or restarted.
   *
   * @return True if everything is okay, false otherwise.
   */
  bool is_okay();

  /**
   * Update the node. This function will poll the Kinect and publish new images if available. If
   * this function returns false, the node should be exited.
   *
   * @param timeout The maximum amount of time to wait for a frame.
   * @return True if new data was published, false if an error occurred.
   */
  bool update(int timeout = 1000);

private:
  /**
   * Convert an OpenCV image to a ROS2 message. This code is adapted from klintan's ros2_usb_camera
   * and removes cv_bridge as a dependancy.
   *
   * @param frame The OpenCV image to convert.
   * @param encoding The encoding of the image. Default is "32FC1".
   * @return std::shared_ptr<sensor_msgs::msg::Image> The ROS2 message.
   */
  std::shared_ptr<sensor_msgs::msg::Image> mat_to_msg(cv::Mat& frame,
                                                      std::string encoding = "32FC1");

  /**
   * Create a point field object. This is used by PointCloud2 to define the fields of the point
   * cloud.
   *
   * @param[in] name The name of the field.
   * @param[in] offset The offset from the start of the point.
   * @param[in] datatype The datatype enum from sensor_msgs::msg::PointField.
   * @param[in] count Number of elements in the field (e.g. 3 for rgb).
   * @return A new point field object.
   */
  sensor_msgs::msg::PointField create_point_field(const std::string& name, const uint32_t offset,
                                                  const uint8_t datatype, const uint32_t count);

  std::string color_frame_;
  std::string depth_frame_;
  std::string camera_topic_;
  std::string device_id_;
  bool enable_pointcloud_;
  bool filter_pointcloud_;

  libfreenect2::Freenect2 freenect2_;
  libfreenect2::Freenect2Device* device_ = nullptr;
  libfreenect2::SyncMultiFrameListener* listener_ = nullptr;
  libfreenect2::Registration* registration_ = nullptr;

  libfreenect2::FrameMap frames_;
  libfreenect2::Frame* undistorted_;
  libfreenect2::Frame* registered_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> color_cinfo_manager_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> depth_cinfo_manager_;

  image_transport::CameraPublisher color_cinfo_pub_;
  image_transport::CameraPublisher depth_cinfo_pub_;
  image_transport::CameraPublisher ir_cinfo_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  std::shared_ptr<sensor_msgs::msg::Image> color_msg_;
  std::shared_ptr<sensor_msgs::msg::Image> depth_msg_;
  std::shared_ptr<sensor_msgs::msg::Image> ir_msg_;
  std::shared_ptr<sensor_msgs::msg::PointCloud2> pointcloud_msg_;
};

#endif