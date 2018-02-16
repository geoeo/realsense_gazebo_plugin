#include "realsense_gazebo_plugin/gazebo_ros_realsense.h"


namespace gazebo
{
// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(GazeboRosRealsense)

/////////////////////////////////////////////////
GazeboRosRealsense::GazeboRosRealsense()
{
}

/////////////////////////////////////////////////
GazeboRosRealsense::~GazeboRosRealsense()
{
  ROS_DEBUG_STREAM_NAMED("realsense_camera", "Unloaded");
}

/////////////////////////////////////////////////
void GazeboRosRealsense::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  ROS_INFO("Realsense Gazebo ROS plugin loading.");

  RealSensePlugin::Load(_model, _sdf);

  this->rosnode_realsense_ = new ros::NodeHandle("/realsense");
  this->rosnode_ = new ros::NodeHandle("/r1");

  // Subscribe to the https://github.com/tuw-robotics/tuw_teleop/tree/master/tuw_keyboard2twist node
  sub = rosnode_->subscribe("cmd_vel",1000,&GazeboRosRealsense::CmdVelCallback,this);

  // initialize camera_info_manager
  this->camera_info_manager_.reset(
    new camera_info_manager::CameraInfoManager(*this->rosnode_realsense_, "realsense"));

  this->itnode_ = new image_transport::ImageTransport(*this->rosnode_realsense_);

  this->color_pub_ = this->itnode_->advertise("camera/color/image_raw", 2);
  this->ir1_pub_ = this->itnode_->advertise("camera/ir/image_raw", 2);
  this->ir2_pub_ = this->itnode_->advertise("camera/ir2/image_raw", 2);
  this->depth_pub_ = this->itnode_->advertise("camera/depth/image_raw", 2);
}

/////////////////////////////////////////////////
void GazeboRosRealsense::OnNewFrame(const rendering::CameraPtr cam,
                                    const transport::PublisherPtr pub)
{
  common::Time current_time = this->world->GetSimTime();

  ignition::math::Vector3d colorCamPos = cam->WorldPosition();
  ignition::math::Quaternion<double> colorCamQuaternion = cam->WorldPose().Rot();
  //ROS_INFO("x:%f, y:%f, z:%f",colorCamPos.X(),colorCamPos.Y(),colorCamPos.Z());

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(colorCamPos.X(), colorCamPos.Y(), colorCamPos.Z()) );
  tf::Quaternion q;
  q.setRPY(colorCamQuaternion.Roll(),colorCamQuaternion.Pitch(),colorCamQuaternion.Yaw());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_base"));

  // identify camera
  std::string camera_id = cam->Name();
  image_transport::Publisher* image_pub;
  if (camera_id.find(COLOR_CAMERA_NAME) != std::string::npos)
  {
    camera_id = COLOR_CAMERA_NAME;
    image_pub = &(this->color_pub_);
  }
  else if (camera_id.find(IRED1_CAMERA_NAME) != std::string::npos)
  {
    camera_id = IRED1_CAMERA_NAME;
    image_pub = &(this->ir1_pub_);
  }
  else if (camera_id.find(IRED2_CAMERA_NAME) != std::string::npos)
  {
    camera_id = IRED2_CAMERA_NAME;
    image_pub = &(this->ir2_pub_);
  }
  else
  {
    ROS_ERROR("Unknown camera name\n");
    camera_id = COLOR_CAMERA_NAME;
    image_pub = &(this->color_pub_);
  }

  // copy data into image
  this->image_msg_.header.frame_id = camera_id;
  this->image_msg_.header.stamp.sec = current_time.sec;
  this->image_msg_.header.stamp.nsec = current_time.nsec;

  // set image encoding
  std::string pixel_format = cam->ImageFormat();
  if (pixel_format == "L_INT8")
  {
    pixel_format = sensor_msgs::image_encodings::MONO8;
  }
  else if (pixel_format == "RGB_INT8")
  {
    pixel_format = sensor_msgs::image_encodings::RGB8;
  }
  else
  {
    ROS_ERROR("Unsupported Gazebo ImageFormat\n");
    pixel_format = sensor_msgs::image_encodings::BGR8;
  }

  // copy from simulation image to ROS msg
  fillImage(this->image_msg_,
    pixel_format,
    cam->ImageHeight(), cam->ImageWidth(),
    cam->ImageDepth() * cam->ImageWidth(),
    reinterpret_cast<const void*>(cam->ImageData()));

  // publish to ROS
  image_pub->publish(this->image_msg_);
}

/////////////////////////////////////////////////
void GazeboRosRealsense::OnNewDepthFrame()
{
  // get current time
  common::Time current_time = this->world->GetSimTime();

  RealSensePlugin::OnNewDepthFrame();

  // copy data into image
  this->depth_msg_.header.frame_id = DEPTH_CAMERA_NAME;
  this->depth_msg_.header.stamp.sec = current_time.sec;
  this->depth_msg_.header.stamp.nsec = current_time.nsec;

  // set image encoding
  std::string pixel_format = sensor_msgs::image_encodings::MONO16;

  // copy from simulation image to ROS msg
  fillImage(this->depth_msg_,
    pixel_format,
    this->depthCam->ImageHeight(), this->depthCam->ImageWidth(),
    2 * this->depthCam->ImageWidth(),
    reinterpret_cast<const void*>(this->depthMap.data()));

  // publish to ROS
  this->depth_pub_.publish(this->depth_msg_);
}

/////////////////////////////////////////////////
void GazeboRosRealsense::OnUpdate()
{
  ros::spinOnce();
}

/////////////////////////////////////////////////
void GazeboRosRealsense::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  geometry_msgs::Vector3 linear =  msg->linear;
  geometry_msgs::Vector3 angular = msg->angular;
  ignition::math::Vector3d gazebo_linear = ignition::math::Vector3d(linear.x,linear.y,linear.z);
  ignition::math::Vector3d gazebo_angular = ignition::math::Vector3d(angular.x,angular.y,angular.z);
  //ROS_INFO("I heard: I got a message on r1/cmd_vel");
  this->rsModel->SetWorldTwist(gazebo_linear,gazebo_angular);
}

}
