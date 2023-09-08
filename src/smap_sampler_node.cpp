#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "smap_interfaces/msg/smap_data.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

#include <memory>
#include <semaphore.h>

// ZED2 Camera info
// Image
//    /zed2/zed_node/rgb/image_rect_color
//    sensor_msgs/msg/Image
// PointCloud2
//    /zed2/zed_node/point_cloud/cloud_registered
//    sensor_msgs/msg/PointCloud2

#define MAP_FRAME std::string( "map" )
#define BASE_LINK_FRAME std::string( "base_link" )
#define CAMERA_FRAME std::string( "zed2_left_camera_frame" )

using std::placeholders::_1;

namespace smap
{

class smap_sampler_node : public rclcpp::Node
{
  private:

    // Variables
    smap_interfaces::msg::SmapData SmapData_msg;
    bool new_data_available = false;

    // TF
    std::unique_ptr< tf2_ros::Buffer > tf_buffer;
    std::shared_ptr< tf2_ros::TransformListener > tf_listener { nullptr };

    // Publisher
    rclcpp::Publisher< smap_interfaces::msg::SmapData >::SharedPtr SmapData_pub;
    rclcpp::Publisher< geometry_msgs::msg::PoseStamped >::SharedPtr pose_pub;
    rclcpp::Publisher< sensor_msgs::msg::PointCloud2 >::SharedPtr pcl_pub;
    rclcpp::Publisher< sensor_msgs::msg::Image >::SharedPtr image_pub;

    // Subscriptions
    rclcpp::Subscription< sensor_msgs::msg::Image >::SharedPtr image_sub;
    rclcpp::Subscription< sensor_msgs::msg::PointCloud2 >::SharedPtr pcl2_sub;

    // Timer
    rclcpp::TimerBase::SharedPtr pose_timer { nullptr };
    rclcpp::TimerBase::SharedPtr smap_data_timer { nullptr };

    // Mutexes
    std::mutex image_mutex;
    std::mutex pcl_mutex;

  public:

    // Constructor/Destructor
    smap_sampler_node() : Node( "smap_sampler_node" )
    {
        RCLCPP_INFO( this->get_logger(), "Initializing smap_sampler_node" );

        // tf buffer
        tf_buffer   = std::make_unique< tf2_ros::Buffer >( this->get_clock() );
        tf_listener = std::make_shared< tf2_ros::TransformListener >( *tf_buffer );

        // Timers
        this->smap_data_timer = this->create_wall_timer(
            std::chrono::milliseconds( 4 ),  // Change Frequency
            std::bind( &smap_sampler_node::data_sampler, this ) );

        this->pose_timer = this->create_wall_timer(
            std::chrono::milliseconds( 50 ),  // Change Frequency
            std::bind( &smap_sampler_node::pose_sampler, this ) );

        // Publisher
        SmapData_pub = this->create_publisher< smap_interfaces::msg::SmapData >(
            std::string( this->get_namespace() ) + std::string( "/sampler/data" ), 10 );
        pose_pub = this->create_publisher< geometry_msgs::msg::PoseStamped >(
            std::string( this->get_namespace() ) + std::string( "/sampler/pose" ), 10 );
        pcl_pub = this->create_publisher< sensor_msgs::msg::PointCloud2 >(
            std::string( this->get_namespace() ) + std::string( "/sampler/pcl" ), 10 );
        image_pub = this->create_publisher< sensor_msgs::msg::Image >(
            std::string( this->get_namespace() ) + std::string( "/sampler/image" ), 10 );

        // Subscriptions
        image_sub = this->create_subscription< sensor_msgs::msg::Image >(
            "/zed2/zed_node/rgb/image_rect_color", 10,
            std::bind( &smap::smap_sampler_node::image_callback, this, _1 ) );
        pcl2_sub = this->create_subscription< sensor_msgs::msg::PointCloud2 >(
            "/zed2/zed_node/point_cloud/cloud_registered", 10,
            std::bind( &smap::smap_sampler_node::pcl2_callback, this, _1 ) );

        RCLCPP_INFO( this->get_logger(), "smap_sampler_node initialized!" );
    }

    ~smap_sampler_node() {}

    void on_process( void )  // Pooling
    {
    }

  private:

    bool get_transform(
        const std::string& target_frame, const std::string& source_frame,
        geometry_msgs::msg::TransformStamped& transform )
    {
        try
        {
            transform = tf_buffer->lookupTransform(
                target_frame, source_frame, rclcpp::Time( 0, 0, RCL_SYSTEM_TIME ), rclcpp::Duration( 1, 0 ) );
        }
        catch( const tf2::TransformException& ex )
        {
            RCLCPP_WARN(
                this->get_logger(), "Get Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(),
                ex.what() );
            return true;
        }
        return false;
    }

    void pose_sampler( void )
    {
        if( this->pose_pub->get_subscription_count() > 0 )
        {
            // Sample Position
            static geometry_msgs::msg::PoseStamped current_pose;
            static geometry_msgs::msg::TransformStamped transform;

            if( this->get_transform( MAP_FRAME, BASE_LINK_FRAME, transform ) )
                RCLCPP_WARN( this->get_logger(), "Invalid pose sampled." );

            current_pose.header           = transform.header;
            current_pose.pose.position.x  = transform.transform.translation.x;
            current_pose.pose.position.y  = transform.transform.translation.y;
            current_pose.pose.position.z  = transform.transform.translation.z;
            current_pose.pose.orientation = transform.transform.rotation;

            this->pose_pub->publish( current_pose );
        }
    }

    void data_sampler( void )
    {
        if( ( this->pcl_pub->get_subscription_count() + this->SmapData_pub->get_subscription_count()
              + this->image_pub->get_subscription_count() )
                > 0
            && this->new_data_available )
        {

            bool invalid = false;

            // Sample Position
            static geometry_msgs::msg::PoseStamped current_pose;
            static geometry_msgs::msg::TransformStamped transform;

            if( this->get_transform( MAP_FRAME, BASE_LINK_FRAME, transform ) )
            {
                RCLCPP_WARN( this->get_logger(), "Invalid pose sampled." );
                invalid = true;
            }
            else
            {
                invalid                                          = false;
                this->SmapData_msg.stamped_pose.header           = transform.header;
                this->SmapData_msg.stamped_pose.pose.position.x  = transform.transform.translation.x;
                this->SmapData_msg.stamped_pose.pose.position.y  = transform.transform.translation.y;
                this->SmapData_msg.stamped_pose.pose.position.z  = transform.transform.translation.z;
                this->SmapData_msg.stamped_pose.pose.orientation = transform.transform.rotation;
            }

            // Sample Image
            if( !invalid )
            {
                // Verify the integrity of the msg
                if( this->SmapData_msg.rgb_image.height == 0 || this->SmapData_msg.rgb_image.width == 0
                    || this->SmapData_msg.rgb_image.step == 0 || this->SmapData_msg.rgb_image.data.empty() )
                {
                    invalid = true;
                    RCLCPP_WARN( this->get_logger(), "Invalid image sampled." );
                }
            }

            // Sample Point Cloud 2
            if( !invalid )
            {
                // Verify the integrity of the msg
                if( this->SmapData_msg.pointcloud.height == 0 || this->SmapData_msg.pointcloud.width == 0
                    || this->SmapData_msg.pointcloud.data.empty() || this->SmapData_msg.pointcloud.fields.empty()
                    || this->SmapData_msg.pointcloud.row_step == 0 || this->SmapData_msg.pointcloud.point_step == 0 )
                {
                    invalid = true;
                    RCLCPP_WARN( this->get_logger(), "Invalid point cloud sampled." );
                }
                else
                {
                    this->get_transform( MAP_FRAME, CAMERA_FRAME, this->SmapData_msg.camera_to_map );
                    this->SmapData_msg.pointcloud.header.frame_id = MAP_FRAME;
                }
            }

            // Publish msg
            if( !invalid )
            {
                this->SmapData_pub->publish( this->SmapData_msg );
                if( this->pcl_pub->get_subscription_count() > 0 )
                {
                    // tf2::doTransform(*(this->last_pcl2_msg),msg.pointcloud,transform);
                    this->pcl_pub->publish( this->SmapData_msg.pointcloud );
                }
                if( this->image_pub->get_subscription_count() > 0 )
                    this->image_pub->publish( this->SmapData_msg.rgb_image );
                this->new_data_available = false;
            }
            else RCLCPP_WARN( this->get_logger(), "Invalid data sampled." );
        }
    }

    void image_callback( sensor_msgs::msg::Image::SharedPtr msg )
    {
        this->new_data_available     = true;
        this->SmapData_msg.rgb_image = *msg;
    }

    void pcl2_callback( sensor_msgs::msg::PointCloud2::SharedPtr msg ) { this->SmapData_msg.pointcloud = *msg; }
};

}  // namespace smap

int main( int argc, char** argv )
{
    (void) argc;
    (void) argv;

    rclcpp::init( argc, argv );

    // Launch Node
    std::shared_ptr< smap::smap_sampler_node > _smap_sampler_node = std::make_shared< smap::smap_sampler_node >();

    // Add nodes to executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node( _smap_sampler_node );

    while( rclcpp::ok() )
    {
        try
        {
            _smap_sampler_node->on_process();  // Pooling
            executor.spin_once();
        }
        catch( std::exception& e )
        {
            std::cout << "Exception!" << std::endl;
            std::cout << e.what() << std::endl;
        }
    }

    rclcpp::shutdown();

    return 0;
}
