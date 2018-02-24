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
  gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _model, _sdf, "Ackeramnndrive" ) );
	gazebo_ros_->isInitialized();

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
// https://github.com/Myzhar/ros_depthsense_camera/issues/9
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
//TODO: Decouple Camera from Transform update
/////////////////////////////////////////////////
void GazeboRosRealsense::OnUpdate()
{
  ros::spinOnce();

  physics::ModelPtr robot = this->rsModel;
  gazebo::math::Pose pose = robot->GetWorldPose();
  gazebo::math::Vector3 pos = pose.pos;
  //gazebo::math::Pose base_link_pose = robot->GetChildLink("base_link")->GetWorldPose();
  // //unsigned int childCount = base_link->GetChildCount();

  double base_cam_off_x;
  double base_cam_off_y;
  double base_cam_off_z;

  gazebo_ros_->getParameter<double>( base_cam_off_x, 		"camera_base_x",		0.0 );
  gazebo_ros_->getParameter<double>( base_cam_off_y, 		"camera_base_y",		0.0 );
  gazebo_ros_->getParameter<double>( base_cam_off_z, 		"camera_base_z",		0.0 );

  double world_cam_x = pos.x + base_cam_off_x;
  double world_cam_y = pos.y + base_cam_off_y;
  double world_cam_z = pos.z + base_cam_off_z;



  //ROS_INFO("test: %f, %f, %f",world_cam_x,world_cam_y,world_cam_z);

  //physics::Joint_V child_link = robot->GetChild()
  //unsigned int l = child_link.size();

  //sensors::SensorManager* man =  sensors::SensorManager::Instance();
  // sensors::SensorPtr camera = sensors::SensorManager::Instance()->GetSensor("simple_ackermann::base_link::color");
  // ignition::math::Pose3d cam_pose = camera->Pose();

  // float diff_x = cam_pose.Pos().X();
  // float diff_y = cam_pose.Pos().Y();
  // float diff_z = cam_pose.Pos().Z();

  // ROS_INFO("pos: %f,%f,%f",diff_x,diff_y,diff_z);

  // for(unsigned int i = 0; i < childCount; i++){
  //   const math::Pose pose = base_link->GetChildJoints().at(i)->GetWorldPose();
  //   //const char *s = robot->GetChild(i)->GetName().c_str();
  //   //physics::BasePtr base = base_link->GetChild(1);
  //   //unsigned int t = base->GetType(); // 9
  //   //ROS_INFO("Type: %u",t);
  //   float diff_x = pose.pos.x;
  //   float diff_y = pose.pos.y;
  //   float diff_z = pose.pos.z;
  //   //ROS_INFO("pos: %f,%f,%f",diff_x,diff_y,diff_z);
  //   //gazebo::physics::BallJoint<SimbodyJoint> joint = boost::dynamic_pointer_cast<gazebo::physics::BallJoint<SimbodyJoint>>(base);
  //   //physics::LinkPtr cam_link = joint.GetChild();


    

  // }

  //physics::LinkPtr joint_link = base_link.GetJointLink(1);

  //ROS_INFO("Child Count: %u",childCount);

  static tf::TransformBroadcaster br;

  gazebo::math::Quaternion quaternion = pose.rot;

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(pos.x, pos.y, pos.z) );
  tf::Quaternion q;
  q.setRPY(quaternion.GetRoll(),quaternion.GetPitch(),quaternion.GetYaw());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot_base"));


  tf::Transform cam_transform;
  cam_transform.setOrigin( tf::Vector3(world_cam_x, world_cam_y, world_cam_z) );
  tf::Quaternion cam_q;
  cam_q.setRPY(quaternion.GetRoll(),quaternion.GetPitch(),quaternion.GetYaw());
  cam_transform.setRotation(cam_q);
  br.sendTransform(tf::StampedTransform(cam_transform, ros::Time::now(), "world", "camera_base"));


}

/////////////////////////////////////////////////
void GazeboRosRealsense::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  geometry_msgs::Vector3 linear =  msg->linear;
  geometry_msgs::Vector3 angular = msg->angular;
  ROS_INFO("Linear Vel on r1/cmd_vel: %f, %f, %f",linear.x,linear.y,linear.z);
  ROS_INFO("Angular Vel on r1/cmd_vel: %f, %f, %f",angular.x,angular.y,angular.z);
  //ROS_INFO("I heard: I got a message on r1/cmd_vel");


  gazebo::math::Pose pose = this->rsModel->GetWorldPose();
  gazebo::math::Quaternion quaternion = pose.rot;
  // Communative because they rotate around the same axis
  gazebo::math::Quaternion new_quaternion =  gazebo::math::Quaternion(angular.x,angular.y,angular.z)*pose.rot;
  //gazebo::math::Quaternion new_quaternion =  pose.rot*gazebo::math::Quaternion(angular.x,angular.y,angular.z);
  gazebo::math::Vector3 Pos = pose.pos;

  float x_new = Pos.x + linear.x*cos(new_quaternion.GetYaw());
  float y_new = Pos.y + linear.x*sin(new_quaternion.GetYaw());

  
  gazebo::math::Pose new_pose = gazebo::math::Pose(gazebo::math::Vector3(x_new,y_new,0),new_quaternion);


  //ignition::math::Vector3d gazebo_linear = ignition::math::Vector3d(linear.x,linear.y,linear.z);
  //ignition::math::Vector3d gazebo_angular = ignition::math::Vector3d(angular.x,angular.y,angular.z);

  // Whats the point of this ? Only rotates model around its own axis
  //this->rsModel->SetWorldTwist(gazebo_linear,gazebo_angular); 

  // Similar to TWIST
  /*gazebo::math::Pose new_pose = gazebo::math::Pose(gazebo_linear.X() + Pos.x,
                                                           gazebo_linear.Y() + Pos.y,
                                                           gazebo_linear.Z() + Pos.z,
                                                           quaternion.GetRoll() + angular.x,
                                                           quaternion.GetPitch() + angular.y,
                                                           quaternion.GetYaw() + angular.z); */

  this->rsModel->SetWorldPose(new_pose);
  
  
}

}
