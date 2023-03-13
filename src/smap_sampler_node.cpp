#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

#include "smap_interfaces/msg/smap_data.hpp"
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
#define MAP_FRAME std::string("map")
#define BASE_LINK_FRAME std::string("base_link")
#define CAMERA_FRAME std::string("zed2_left_camera_frame")

using std::placeholders::_1;

namespace smap
{

class smap_sampler : public rclcpp::Node
{
private:
  //** Variables **//
  sensor_msgs::msg::Image::SharedPtr last_image_msg;
  sensor_msgs::msg::PointCloud2::SharedPtr last_pcl2_msg;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};

  // Publisher
  rclcpp::Publisher<smap_interfaces::msg::SmapData>::SharedPtr SmapData_pub = this->create_publisher<smap_interfaces::msg::SmapData>("/smap/sampler/data", 10);
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/smap/sampler/pose", 10);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/smap/sampler/pcl", 10);

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
  }

private:

  bool get_transform(const std::string &target_frame, const std::string &source_frame, geometry_msgs::msg::TransformStamped &transform)
  {
    // https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html
    //tf_buffer->lookupTransform()
    try {
      transform = tf_buffer->lookupTransform(
        target_frame, source_frame,
        rclcpp::Time(0, 0, RCL_SYSTEM_TIME),
        rclcpp::Duration(1,0)
      );
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Get Could not transform %s to %s: %s",
        source_frame.c_str(), target_frame.c_str(), ex.what()
      );
      return true;
      //tf_buffer->waitForTransform()
    }
    return false;
  }

  void pose_sampler(void)
  {
    // Sample Position
    static geometry_msgs::msg::PoseStamped current_pose;
    static geometry_msgs::msg::TransformStamped transform;

    if(this->get_transform(MAP_FRAME,BASE_LINK_FRAME,transform)) RCLCPP_WARN(
      this->get_logger(),
      "Invalid pose sampled."
    );
    
    current_pose.header = transform.header;
    current_pose.pose.position.x = transform.transform.translation.x;
    current_pose.pose.position.y = transform.transform.translation.y;
    current_pose.pose.position.z = transform.transform.translation.z;
    current_pose.pose.orientation = transform.transform.rotation;

    this->pose_pub->publish(current_pose);
  }

  void data_sampler(void)
  { 

    static bool invalid = false;
    // Sample Position
    static smap_interfaces::msg::SmapData msg;

    static geometry_msgs::msg::PoseStamped current_pose;
    static geometry_msgs::msg::TransformStamped transform;

    if(this->get_transform(MAP_FRAME,BASE_LINK_FRAME,transform)){
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid pose sampled."
      );
      invalid = true;
    }
    else{
      invalid = false;
      msg.stamped_pose.header = transform.header;
      msg.stamped_pose.pose.position.x = transform.transform.translation.x;
      msg.stamped_pose.pose.position.y = transform.transform.translation.y;
      msg.stamped_pose.pose.position.z = transform.transform.translation.z;
      msg.stamped_pose.pose.orientation = transform.transform.rotation;
    }
    // TODO: Implement semaphores

    // Sample Image
    if(!invalid){
      // Verify the integrity of the msg
      if(
        this->last_image_msg == nullptr || this->last_image_msg->height == 0 || 
        this->last_image_msg->width == 0 || this->last_image_msg->step == 0 || 
        this->last_image_msg->data.empty()
      ){
        invalid = true;
        RCLCPP_WARN(
          this->get_logger(),
          "Invalid image sampled."
        );
      }else{
        msg.rgb_image.header = this->last_image_msg->header;
        msg.rgb_image.height = this->last_image_msg->height;
        msg.rgb_image.width = this->last_image_msg->width;
        msg.rgb_image.is_bigendian = this->last_image_msg->is_bigendian;
        msg.rgb_image.step = this->last_image_msg->step;
        msg.rgb_image.encoding = this->last_image_msg->encoding;
        msg.rgb_image.data = this->last_image_msg->data;
      }
    }

    // Sample Point Cloud 2
    if(!invalid){
      // Verify the integrity of the msg
      if(
        this->last_pcl2_msg == nullptr || this->last_pcl2_msg->height == 0 || 
        this->last_pcl2_msg->width == 0 || this->last_pcl2_msg->data.empty() || 
        this->last_pcl2_msg->fields.empty() || this->last_pcl2_msg->row_step == 0 || 
        this->last_pcl2_msg->point_step == 0
      ){
        invalid = true;
        RCLCPP_WARN(
          this->get_logger(),
          "Invalid point cloud sampled."
        );
      }else{
        // Transform points
        this->get_transform(MAP_FRAME,CAMERA_FRAME,transform);
        tf2::doTransform(*(this->last_pcl2_msg),msg.pointcloud,transform);
        this->pcl_pub->publish(msg.pointcloud);
      }
    }

    // Publish msg
    if(!invalid) this->SmapData_pub->publish(msg);
    else{
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid data sampled."
      );
    }
  }

  void image_callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    this->last_image_msg = msg;
  }

  void pcl2_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    this->last_pcl2_msg = msg;
  }
//public:
  
};

}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc,argv);

  //while(rclcpp::ok()){
  //  try{
  //      _smap_node->on_process(); // Pooling
  //      _topological_map_node->on_process(); // Pooling
  //      executor.spin_once();
  //    }catch (std::exception& e){
  //      std::cout << "Exception!" << std::endl;
  //      std::cout << e.what() << std::endl;
  //    }
  //  }
  //  rclcpp::shutdown();
  //}

  rclcpp::spin(std::make_shared<smap::smap_sampler>());

  rclcpp::shutdown();

  return 0;
}