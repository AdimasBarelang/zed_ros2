#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <darknet_ros_msgs/msg/bounding_boxes.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sl/Camera.hpp"

using namespace std;
using namespace sl;

class ZedCameraNode : public rclcpp::Node
{
public:
  ZedCameraNode() : Node("zed_camera_node")
  {
    // Create a publisher for the captured image
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("zed_camera/image_raw", 10);
    point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("zed_camera/point_cloud", 10);

    // Create a subscriber for the bounding boxes
    bbox_sub_ = create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
      "darknet_ros/bounding_boxes", 10,
      [this](const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg) {
        // Process the bounding boxes
        processBoundingBoxes(msg);
      });

    // Set configuration parameters
    init_parameters_.camera_resolution = RESOLUTION::VGA;
    init_parameters_.camera_fps = 60;
    init_parameters_.coordinate_units = UNIT::METER;

    // Open the camera
    auto returned_state = zed_.open(init_parameters_);
    if (returned_state != ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Error %d, exit program.", static_cast<int>(returned_state));
      return;
    }

    // Create a timer to capture images at 60 fps
    capture_timer_ = create_wall_timer(1s / 60, [this]() {
      // Grab an image
      auto returned_state = zed_.grab();
      if (returned_state == ERROR_CODE::SUCCESS) {
        // Get the left image
        zed_.retrieveImage(image_, VIEW::LEFT);
        zed_.retrieveMeasure(point_cloud_, MEASURE::XYZRGBA);

        // Convert the ZED image to OpenCV format
        cv::Mat frame(image_.getHeight(), image_.getWidth(), CV_8UC4, image_.getPtr<sl::uchar1>(sl::MEM::CPU));

        // Convert the image to a ROS message using cv_bridge
        cv_bridge::CvImage cv_image(std_msgs::msg::Header(), "bgra8", frame);
        sensor_msgs::msg::Image::SharedPtr msg = cv_image.toImageMsg();

        // Publish the image asynchronously
        image_pub_->publish(*msg);
      }
    });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr bbox_sub_;
  sl::Camera zed_;
  sl::InitParameters init_parameters_;
  sl::Mat image_, point_cloud_;
  rclcpp::TimerBase::SharedPtr capture_timer_;

  // void processBoundingBoxes(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg)
  // {
  //   if (msg->bounding_boxes.empty()) {
  //     RCLCPP_INFO(get_logger(), "No bounding boxes received.");
  //     return;
  //   }

  //   for (const auto& bbox : msg->bounding_boxes) {
  //     int x = bbox.xmin + (bbox.xmax - bbox.xmin) / 2;
  //     int y = bbox.ymin + (bbox.ymax - bbox.ymin) / 2;

  //     sl::float4 point_cloud_value;
  //     point_cloud_.getValue(x, y, &point_cloud_value);

  //     if (std::isfinite(point_cloud_value.z)) {
  //       float distance = sqrt(point_cloud_value.x * point_cloud_value.x +
  //                           point_cloud_value.y * point_cloud_value.y +
  //                           point_cloud_value.z * point_cloud_value.z);
  //       cout << "Distance to Object at {" << x << ";" << y << "}: " << distance << "M" << endl;
  //     } else {
  //       cout << "The Distance to Object at {" << x << ";" << y << "} cannot be computed" << endl;
  //     }
  //   }
  // }

  void processBoundingBoxes(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg)
  {
    if (msg->bounding_boxes.empty()) {
      RCLCPP_INFO(get_logger(), "No bounding boxes received.");
      return;
    }

    std::map<std::string, float> distances; // Map to store distances for each object class

    for (const auto& bbox : msg->bounding_boxes) {
      int x = bbox.xmin + (bbox.xmax - bbox.xmin) / 2;
      int y = bbox.ymin + (bbox.ymax - bbox.ymin) / 2;

      sl::float4 point_cloud_value;
      point_cloud_.getValue(x, y, &point_cloud_value);

      if (std::isfinite(point_cloud_value.z)) {
        float distance = sqrt(point_cloud_value.x * point_cloud_value.x +
                              point_cloud_value.y * point_cloud_value.y +
                              point_cloud_value.z * point_cloud_value.z);
        distances[bbox.class_id] = distance; // Store distance in the map with object class as key
      }
    }

    // Print distances for each object class
    for (const auto& distance : distances) {
      cout << "Distance to " << distance.first << ": " << distance.second << "M" << endl;
    }
  }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedCameraNode>());
  rclcpp::shutdown();

  return 0;
}
