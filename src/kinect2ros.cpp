#include <cv_bridge/cv_bridge.h>

#include <cstdio>
#include <libfreenect2/libfreenect2.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace cv;
using rcl_interfaces::msg::ParameterDescriptor;

bool should_shutdown = false;  ///< Set to true to shut down the node.

/**
 * Handle SIGINT by setting a global flag to shut down.
 *
 * @param[in] s The signal number.
 */
void sigint_handler(int s) {
    (void)s;
    should_shutdown = true;
}

/**
 * A ROS2 node that publishes data from a Kinect v2 using libfreenect2.
 */
class Kinect2ROS : public rclcpp::Node {
   public:
    /**
     * Construct a new Kinect2ROS object.
     */
    Kinect2ROS() : Node("kinect2ros") {
        RCLCPP_INFO(this->get_logger(), "kinect2ros node has been created");

        /*
         * Declare parameters.
         */

        auto device_id_desc = ParameterDescriptor();
        device_id_desc.description = "The serial number of the Kinect v2 device to use.";
        this->declare_parameter("device_id", "", device_id_desc);

        auto registration_desc = ParameterDescriptor();
        registration_desc.description = "Whether to register the depth image to the color image.";
        this->declare_parameter("depth", true, registration_desc);

        auto compress_color_desc = ParameterDescriptor();
        compress_color_desc.description = "Whether to compress the color image.";
        this->declare_parameter("compress_color", true, compress_color_desc);

        auto upscale_depth_desc = ParameterDescriptor();
        upscale_depth_desc.description =
            "Whether to upscale the depth image to the resolution of the color image. Otherwise, "
            "both images will be at the resolution of the depth camera.";
        this->declare_parameter("upscale_depth", true, upscale_depth_desc);

        auto color_topic_desc = ParameterDescriptor();
        color_topic_desc.description = "The topic on which to publish color images.";
        this->declare_parameter("color_topic", "color", color_topic_desc);

        auto depth_topic_desc = ParameterDescriptor();
        depth_topic_desc.description = "The topic on which to publish depth images.";
        this->declare_parameter("depth_topic", "depth", depth_topic_desc);

        //  Read parameter values.
        _compress_color = this->get_parameter("compress_color").as_bool();
        _device_id = this->get_parameter("device_id").as_string();
        _registration = this->get_parameter("depth").as_bool();
        _upscale_depth = this->get_parameter("upscale_depth").as_bool();
        string color_topic = this->get_parameter("color_topic").as_string();
        string depth_topic = this->get_parameter("depth_topic").as_string();

        /*
         * Create publishers.
         */

        if (_compress_color)
            _color_pub_compressed =
                this->create_publisher<sensor_msgs::msg::CompressedImage>(color_topic, 1);
        else
            _color_pub_uncompressed =
                this->create_publisher<sensor_msgs::msg::Image>(color_topic, 1);
        _depth_pub = this->create_publisher<sensor_msgs::msg::Image>(depth_topic, 1);
    }

    /**
     * Get the value of the device ID parameter.
     *
     * @return The serial number of the Kinect v2 device to use.
     */
    string get_device_id() { return _device_id; }

    /**
     * Get the value of the registration parameter.
     *
     * @return Whether to register the depth image to the color image.
     */
    bool get_registration() { return _registration; }

    /**
     * Get the value of the upscale depth parameter.
     *
     * @return Whether to upscale the depth image to the resolution of the color image.
     */
    bool get_upscale_depth() { return _upscale_depth; }

    /**
     * Publish a color image from an OpenCV Mat. This will also compress the image if the
     * compress_color parameter is set to true.
     *
     * @param[in] color The color image.
     */
    void publish_color(Mat color) {
        std_msgs::msg::Header header;
        if (_compress_color) {
            auto msg =
                cv_bridge::CvImage(header, "bgr8", color).toCompressedImageMsg(cv_bridge::PNG);
            _color_pub_compressed->publish(*msg);
        } else {
            auto msg = cv_bridge::CvImage(header, "bgr8", color).toImageMsg();
            _color_pub_uncompressed->publish(*msg);
        }
    }

    /**
     * Publish a depth image from an OpenCV Mat.
     *
     * @param[in] depth The depth image.
     */
    void publish_depth(Mat depth) {
        std_msgs::msg::Header header;
        auto msg = cv_bridge::CvImage(header, "32FC1", depth).toImageMsg();
        _depth_pub->publish(*msg);
    }

   private:
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr _color_pub_compressed;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _color_pub_uncompressed;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _depth_pub;

    string _device_id;
    bool _registration;
    bool _compress_color;
    bool _upscale_depth;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create the node but don't start it yet.
    auto node = make_shared<Kinect2ROS>();

    // Grab the device ID and registration parameters.
    string device_id = node->get_device_id();
    bool enable_depth = node->get_registration();
    bool upscale_depth = node->get_upscale_depth();

    libfreenect2::Freenect2 freenect2;

    // If no device ID was specified, get the default device.
    if (device_id.empty()) {
        RCLCPP_INFO(node->get_logger(), "No device ID specified, using default device.");
        device_id = freenect2.getDefaultDeviceSerialNumber();
        if (device_id.empty()) {
            RCLCPP_ERROR(node->get_logger(), "No devices are connected.");
            return 1;
        }
    }

    // Try to open the device.
    RCLCPP_INFO(node->get_logger(), "Using device with serial number %s.", device_id.c_str());
    auto device = freenect2.openDevice(device_id);
    if (device == nullptr) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open device.");
        return 1;
    }

    // Set up signal handlers.
    signal(SIGINT, sigint_handler);
    should_shutdown = false;

    // Set up listeners. If we are registering the depth image, we need to listen to both color
    // and depth frames. Otherwise, we only need to listen to color frames.
    libfreenect2::SyncMultiFrameListener listener(enable_depth ? libfreenect2::Frame::Color |
                                                                     libfreenect2::Frame::Depth
                                                               : libfreenect2::Frame::Color);

    // Register the listeners with the device.
    device->setColorFrameListener(&listener);
    if (enable_depth) device->setIrAndDepthFrameListener(&listener);

    // Start the device.
    if (!device->start()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to start device.");
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Device %s started with firmware version %s.",
                device_id.c_str(), device->getFirmwareVersion());

    // Setup frame buffers.
    libfreenect2::FrameMap frames;
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4),
        depth_upscaled(1920, 1080 + 2, 4);
    Mat color_mat, depth_mat;

    // Setup registration if necessary.
    libfreenect2::Registration* registration = nullptr;
    if (enable_depth) {
        // IR camera params are the same as depth camera params.
        registration = new libfreenect2::Registration(device->getIrCameraParams(),
                                                      device->getColorCameraParams());
    }

    // Main loop. Keep publishing images until we receive SIGINT.
    while (!should_shutdown) {
        // If there is no new frame, spin and check again.
        if (!listener.hasNewFrame()) {
            rclcpp::spin_some(node);
            continue;
        }

        // Receive the frame. A frame is available, so this will not block.
        listener.waitForNewFrame(frames);
        auto color_frame = frames[libfreenect::Frame::Color];

        // Handle the case where we do not register the depth data. Here, we remove the depth
        // channel from the color image and publish it directly.
        if (!enable_depth) {
            cv::Mat(color_frame->height, color_frame->width, CV_8UC4, color_frame->data)
                .copyTo(color_mat);
            cv::cvtColor(color_mat, color_mat, cv::COLOR_BGRA2BGR);  // Remove depth channel.
            node->publish_color(color_mat);
        }

        // Handle the case where we are registering but not upscaling the depth data. Here, we
        // register the depth data to the color data and publish the result.
        else if (enable_depth && !upscale_depth) {
            // Register depth to color.
            auto depth_frame = frames[libfreenect::Frame::Depth];
            registration->apply(color_frame, depth_frame, &undistorted, &registered, true, nullptr);

            // Convert registered color pixels to OpenCV Mat and publish.
            cv::Mat(registered.height, registered.width, CV_8UC4, registered.data)
                .copyTo(color_mat);
            cv::cvtColor(color_mat, color_mat, cv::COLOR_BGRA2BGR);  // Remove depth channel.
            node->publish_color(color_mat);

            // Convert depth pixels to OpenCV Mat and publish.
            cv::Mat(registered.height, registered.width, CV_32FC1, registered.data)
                .copyTo(depth_mat);
            node->publish_depth(depth_mat);
        }

        // Handle the case where we are registering and upscaling the depth data. Here, we
        // register the depth data to the color data, upscale the depth data, and publish both.
        else if (enable_depth && upscale_depth) {
            // Register depth to color.
            auto depth_frame = frames[libfreenect::Frame::Depth];
            registration->apply(color_frame, depth_frame, &undistorted, &registered, true,
                                &depth_upscaled);

            // Convert original color pixels to OpenCV Mat and publish.
            cv::Mat(color_frame->height, color_frame->width, CV_8UC4, color_frame->data)
                .copyTo(color_mat);
            cv::cvtColor(color_mat, color_mat, cv::COLOR_BGRA2BGR);  // Remove depth channel.
            node->publish_color(color_mat);

            // Convert upscaled depth pixels to OpenCV Mat and publish.
            cv::Mat(depth_upscaled.height, depth_upscaled.width, CV_32FC1, depth_upscaled.data)
                .copyTo(depth_mat);
            node->publish_depth(depth_mat);
        }
    }

    // Release resources.
    listener.release(frames);
    device->stop();
    device->close();
    delete registration;

    RCLCPP_INFO(node->get_logger(), "Kinect has been disconnected cleanly.");

    rclcpp::shutdown();
}
