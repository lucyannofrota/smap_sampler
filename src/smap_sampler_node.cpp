
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"

#include "smap_interfaces/msg/smap_data.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// ZED2 Camera
// Image
//    /zed2/zed_node/rgb/image_rect_color
//    sensor_msgs/msg/Image
// PointCloud2
//    /zed2/zed_node/point_cloud/cloud_registered
//    sensor_msgs/msg/PointCloud2

// Node

// TODO: Configure callback groups

#define FROM_FRAME std::string("map")
#define TO_FRAME std::string("base_link")

#define RAD2DEG(x) 180 * x / M_PI

using std::placeholders::_1;

namespace smap
{

class smap_sampler : public rclcpp::Node
{
private:
  //** Variables **//
  sensor_msgs::msg::Image last_image_msg;
  sensor_msgs::msg::PointCloud2 last_pcl2_msg;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};

  // Publisher
  rclcpp::Publisher<smap_interfaces::msg::SmapData>::SharedPtr SmapData_pub = this->create_publisher<smap_interfaces::msg::SmapData>("/smap_sampler/data", 10);
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/smap_sampler/pose", 10);

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub = this->create_subscription<sensor_msgs::msg::Image>(
    "/zed2/zed_node/rgb/image_rect_color",
    10,
    std::bind(&smap::smap_sampler::image_callback, this, _1)
  );
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/zed2/zed_node/point_cloud/cloud_registered",
    10,
    std::bind(&smap::smap_sampler::pcl2_callback, this, _1)
  );

  // Timer
  rclcpp::TimerBase::SharedPtr pose_timer{nullptr};
  rclcpp::TimerBase::SharedPtr smap_data_timer{nullptr};

public:
  // Constructor/Destructor
  smap_sampler()
  : Node("smap_sampler")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing smap_sampler");

    // tf buffer
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  
    // Callbacks
    this->smap_data_timer = this->create_wall_timer(
      std::chrono::milliseconds(250), // Change Frequency
      std::bind(
        &smap_sampler::data_sampler,
        this
      )
    );

    this->pose_timer = this->create_wall_timer(
      std::chrono::milliseconds(50), // Change Frequency
      std::bind(
        &smap_sampler::pose_sampler,
        this
      )
    );
  }
  ~smap_sampler()
  {
  }

  void on_process(void) // Pooling
  {
    // RCLCPP_DEBUG(this->get_logger(),"Process smap_sampler");
  }

private:

  bool sample_pose(geometry_msgs::msg::PoseStamped &current_pose)
  {
    // Sample Position
    geometry_msgs::msg::TransformStamped transform;

    // https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html
    try {
      transform = tf_buffer->lookupTransform(
        FROM_FRAME, TO_FRAME,
        tf2::TimePointZero
      );
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Could not transform %s to %s: %s",
        TO_FRAME.c_str(), FROM_FRAME.c_str(), ex.what()
      );
      return true;
    }

    double roll, pitch, yaw;
    tf2::Matrix3x3(
      tf2::Quaternion(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
      )).getRPY(roll, pitch, yaw);

    current_pose.header.frame_id = "/map";
    current_pose.header.stamp = this->get_clock().get()->now();

    current_pose.pose.position.x = transform.transform.translation.x;
    current_pose.pose.position.y = transform.transform.translation.y;
    current_pose.pose.position.z = transform.transform.translation.z;
    current_pose.pose.orientation = transform.transform.rotation;

    return false;
  }

  void pose_sampler(void)
  {
    // Sample Position
    static geometry_msgs::msg::PoseStamped current_pose;
    this->sample_pose(current_pose);
    this->pose_pub->publish(current_pose);
  }

  void data_sampler(void)
  { 

    static bool invalid = true;
    // Sample Position
    static smap_interfaces::msg::SmapData msg;
    invalid = this->sample_pose(msg.stamped_pose);
    // TODO: Implement semaphores

    // Sample Image
    if(!invalid) msg.rgb_image = this->last_image_msg;
    // TODO: invalid verification + warn
    invalid |= 0;
    // Sample Point Cloud 2
    if(!invalid) msg.pointcloud = this->last_pcl2_msg;
    // TODO: invalid verification + warn
    invalid |= 0;

    // Publish msg
    if(!invalid) this->SmapData_pub->publish(msg);
    else{
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid sampled data."
      );
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    this->last_image_msg = *msg;
  }

  void pcl2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    this->last_pcl2_msg = *msg;
  }

//public:
  
};

}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<smap::smap_sampler>());

  rclcpp::shutdown();

  return 0;
}