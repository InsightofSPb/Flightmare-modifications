// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"
#include "flightros/pilot/flight_pilot.hpp"
#include <cmath>
namespace flightros {

FlightPilot::FlightPilot(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::INDUSTRIAL),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(50.0) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }


  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

  // add mono camera
  rgb_camera_ = std::make_shared<RGBCamera>();
  Vector<3> B_r_BC(0.0, 0.35, -0.2);
  // Matrix<3, 3> R_BC = Quaternion(std::cos(-M_PI_4), std::sin(-M_PI_4), 0.0, 0.0).toRotationMatrix();
  Matrix<3, 3> R_BC = Quaternion(0.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(720);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  rgb_camera_->setPostProcesscing(
    std::vector<bool>{true, true, true});  // depth, segmentation, optical flow
  quad_ptr_->addRGBCamera(rgb_camera_);

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);

  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("flight_pilot/state_estimate", 1,
                                 &FlightPilot::poseCallback, this);

  timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_),
                                     &FlightPilot::mainLoopCallback, this);

  // initialize publishers
  image_transport::ImageTransport it(pnh_);
  rgb_pub_ = it.advertise("/rgb", 1);
  segmentation_pub_ = it.advertise("/segmentation", 1);
  depth_pub_ = it.advertise("/depth", 1);
  opticalflow_pub_ = it.advertise("/optical_flow", 1);

  point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1);

  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();

  // connect unity
  setUnity(unity_render_);
  connectUnity();
}

FlightPilot::~FlightPilot() {}

void FlightPilot::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;
  
  quad_ptr_->setState(quad_state_);

  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();

    if (quad_ptr_->getCollision()) {
      // collision happened
      ROS_INFO("COLLISION");
    }

    cv::Mat img;
    ros::Time timestamp = ros::Time::now();

    rgb_camera_->getRGBImage(img);
    sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    rgb_msg->header.stamp = timestamp;
    rgb_pub_.publish(rgb_msg);

    rgb_camera_->getSegmentation(img);
    sensor_msgs::ImagePtr segmentation_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    segmentation_msg->header.stamp = timestamp;
    segmentation_pub_.publish(segmentation_msg);

    rgb_camera_->getDepthMap(img);
    sensor_msgs::ImagePtr depth_msg =
      cv_bridge::CvImage(std_msgs::Header(), "32FC1", img).toImageMsg();
    depth_msg->header.stamp = timestamp;
    depth_pub_.publish(depth_msg);

    rgb_camera_->getOpticalFlow(img);
    sensor_msgs::ImagePtr opticflow_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    opticflow_msg->header.stamp = timestamp;
    opticalflow_pub_.publish(opticflow_msg);
  }
}

void FlightPilot::mainLoopCallback(const ros::TimerEvent &event) {
  // empty
}

void FlightPilot::depthImageToPointCloud(const sensor_msgs::ImageConstPtr& depth_msg) {
    // Assuming depth_msg is your depth image message
    float fx = 525.0;  // Example focal length x
    float fy = 525.0;  // Example focal length y
    float cx = 319.5;  // Example principal point x (image width / 2)
    float cy = 239.5;  // Example principal point y (image height / 2)
    float factor = 5000.0;  // Example for scaling depth values to meters (specific to depth sensor)

    sensor_msgs::PointCloud2 cloud;
    cloud.header = depth_msg->header;
    cloud.height = depth_msg->height;
    cloud.width = depth_msg->width;
    cloud.is_dense = false;
    sensor_msgs::PointCloud2Modifier pcd_modifier(cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(uint16_t);

    for (int v = 0; v < depth_msg->height; ++v, depth_row += row_step) {
        for (int u = 0; u < depth_msg->width; ++u) {
            uint16_t depth = depth_row[u];

            // Convert depth to meters
            float z = depth / factor;
            if (z == 0) continue;  // Skip no depth

            // Convert (u, v, z) to (x, y, z)
            float x = (u - cx) * z / fx;
            float y = (v - cy) * z / fy;

            *iter_x = x;
            *iter_y = y;
            *iter_z = z;

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
    }

    // Now you can publish this point cloud or use it further
    point_cloud_pub_.publish(cloud);
}


bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilot::loadParams(void) {
  // load parameters
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}

}  // namespace flightros
