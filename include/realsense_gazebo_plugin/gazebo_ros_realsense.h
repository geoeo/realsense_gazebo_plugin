#ifndef _GAZEBO_ROS_REALSENSE_PLUGIN_
#define _GAZEBO_ROS_REALSENSE_PLUGIN_

#include "realsense_gazebo_plugin/RealSensePlugin.h"

#include <gazebo_plugins/gazebo_ros_utils.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <geometry_msgs/Twist.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <memory>

namespace gazebo
{
  /// \brief A plugin that simulates Real Sense camera streams.
  class GazeboRosRealsense : public RealSensePlugin
  {
    /// \brief Constructor.
    public: GazeboRosRealsense();

    /// \brief Destructor.
    public: ~GazeboRosRealsense();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback that publishes a received Depth Camera Frame as an
    /// ImageStamped message.
    public: virtual void OnNewDepthFrame();

    /// \brief Callback that publishes a received Camera Frame as an
    /// ImageStamped message.
    public: virtual void OnNewFrame(const rendering::CameraPtr cam,
                                    const transport::PublisherPtr pub);

    public: virtual void OnUpdate();

    protected: boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

    /// \brief A pointer to the ROS node.
    ///  A node will be instantiated if it does not exist.
    protected: ros::NodeHandle* rosnode_realsense_;
    protected: ros::NodeHandle* rosnode_;
    
    private: image_transport::ImageTransport* itnode_;
    protected: image_transport::Publisher color_pub_, ir1_pub_, ir2_pub_, depth_pub_;

    /// \brief ROS image messages
    protected: sensor_msgs::Image image_msg_, depth_msg_;

    private: ros::Subscriber sub;

    private: void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    private:  GazeboRosPtr gazebo_ros_;
    private: std::string fixed_frame;

    // Values taken from realsense.xacro for publishing camera / ackermann position
  private:
    double base_cam_off_x;
    double base_cam_off_y;
    double base_cam_off_z;

    double model_x_ackermann_offset;
    double model_z_ackermann_offset;

  };
}
#endif /* _GAZEBO_ROS_REALSENSE_PLUGIN_ */
