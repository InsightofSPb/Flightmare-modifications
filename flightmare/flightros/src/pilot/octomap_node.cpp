#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

class OctomapNode {
public:
    OctomapNode() : it_(nh_) {
        depth_sub_ = it_.subscribe("/depth", 1, &OctomapNode::depthCallback, this);
        octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap", 1);
        octree_ = new octomap::OcTree(0.1); // Resolution of 0.1 meters
    }

    ~OctomapNode() {
        delete octree_;
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (int i = 0; i < cv_ptr->image.rows; ++i) {
            for (int j = 0; j < cv_ptr->image.cols; ++j) {
                float depth = cv_ptr->image.at<float>(i, j);
                if (std::isfinite(depth)) {
                    pcl::PointXYZ point;
                    point.x = j * depth;
                    point.y = i * depth;
                    point.z = depth;
                    cloud.points.push_back(point);
                }
            }
        }

        for (auto& point : cloud.points) {
            octree_->updateNode(octomap::point3d(point.x, point.y, point.z), true);
        }

        octree_->updateInnerOccupancy();

        octomap_msgs::Octomap octomap_msg;
        octomap_msgs::fullMapToMsg(*octree_, octomap_msg);
        octomap_msg.header.frame_id = "map";
        octomap_msg.header.stamp = ros::Time::now();
        octomap_pub_.publish(octomap_msg);
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber depth_sub_;
    ros::Publisher octomap_pub_;
    octomap::OcTree* octree_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_node");
    OctomapNode node;
    ros::spin();
    return 0;
}