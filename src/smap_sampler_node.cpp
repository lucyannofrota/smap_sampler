#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"

#include "smap_interfaces/msg/smap_data.hpp"
#include "std_msgs/msg/string.hpp"

// ZED2 Camera
// Image
//    /zed2/zed_node/rgb/image_rect_color
//    sensor_msgs/msg/Image
// PointCloud2
//    /zed2/zed_node/point_cloud/cloud_registered
//    sensor_msgs/msg/PointCloud2

// Node

#define FROM_FRAME std::string("map")
#define TO_FRAME std::string("base_link")

#define RAD2DEG(x) 180 * x / M_PI

namespace smap
{

class smap_sampler : public rclcpp::Node
{
private:
  //** Variables **//
  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};

  // Publisher
  rclcpp::Publisher<smap_interfaces::msg::SmapData>::SharedPtr SmapData_pub = this->create_publisher<smap_interfaces::msg::SmapData>("/smap_sampler/data", 10);
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/smap_sampler/pose", 10);

  // Subscriptions
  //rclcpp::Subscriptions<

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
      std::chrono::milliseconds(50), // Change Frequency
      std::bind(
        &smap_sampler::data_sampler,
        this
      )
    );

    this->pose_timer = this->create_wall_timer(
      std::chrono::milliseconds(50), // Change Frequency
      std::bind(
        &smap_sampler::data_sampler,
        this
      )
    );
    RCLCPP_INFO(this->get_logger(), "smap_sampler initialized.");
  }
  ~smap_sampler()
  {
  }

  void on_process(void) // Pooling
  {
    // RCLCPP_DEBUG(this->get_logger(),"Process smap_sampler");
  }

private:

  void sample_pose(geometry_msgs::msg::PoseStamped &current_pose)
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

      return;
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
  }

  void data_sampler(void)
  { 
    // Sample Position
    static geometry_msgs::msg::PoseStamped current_pose;
    this->sample_pose(current_pose);

    // Sample Image
    


  }

  void pose_sampler(void)
  {
    // Sample Position
    static geometry_msgs::msg::PoseStamped current_pose;
    this->sample_pose(current_pose);
    this->pose_pub->publish(current_pose);
  }
//public:
  
};

}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc,argv);

  //std::shared_ptr<smap::smap_sampler> _smap_sampler_node = std::make_shared<smap::smap_sampler>();

  rclcpp::spin(std::make_shared<smap::smap_sampler>());

  rclcpp::shutdown();

  printf("hello world smap_sampler package\n");
  return 0;
}