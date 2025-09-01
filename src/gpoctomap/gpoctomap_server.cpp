#include <string>
#include <iostream>
#include <numeric>
#include <cmath>
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include "markerarray_pub.h"
#include "gpoctomap.h"

class GPOctoMapServer {
public:
    GPOctoMapServer(ros::NodeHandle& nh) : nh_(nh) {
        // === Parameters ===
        nh_.param<std::string>("frame_id", frame_id_, std::string("/map"));
        nh_.param<std::string>("cloud_topic", cloud_topic_, std::string("/sonar_camera_merge/cloud"));
        nh_.param<std::string>("topic_occ", map_topic_occ_, std::string("/occupied_cells_vis_array"));
        nh_.param<std::string>("topic_free", map_topic_free_, std::string("/free_cells_vis_array"));
        nh_.param<double>("max_range", max_range_, -1.0);
        nh_.param<double>("resolution", resolution_, 0.1);
        nh_.param<int>("block_depth", block_depth_, 4);
        nh_.param<double>("sf2", sf2_, 1.0);
        nh_.param<double>("ell", ell_, 1.0);
        nh_.param<double>("free_resolution", free_resolution_, 0.1);
        nh_.param<double>("ds_resolution", ds_resolution_, 0.1);
        nh_.param<double>("free_thresh", free_thresh_, 0.3);
        nh_.param<double>("occupied_thresh", occupied_thresh_, 0.7);
        nh_.param<double>("min_z", min_z_, 0.0);
        nh_.param<double>("max_z", max_z_, 0.0);
        nh_.param<bool>("original_size", original_size_, true);

        nh_.param<double>("noise", noise_, 0.01);
        nh_.param<double>("l", l_, 100.0);
        nh_.param<double>("min_var", min_var_, 0.001);
        nh_.param<double>("max_var", max_var_, 1000.0);
        nh_.param<double>("max_known_var", max_known_var_, 0.02);

        // === Create objects ===
        map_ = std::make_unique<la3dm::GPOctoMap>(resolution_, block_depth_, sf2_, ell_, noise_, l_, min_var_, max_var_, max_known_var_, free_thresh_, occupied_thresh_);
        listener_ = std::make_unique<tf::TransformListener>();
        m_pub_occ_ = std::make_unique<la3dm::MarkerArrayPub>(nh_, map_topic_occ_, resolution_);
        m_pub_free_ = std::make_unique<la3dm::MarkerArrayPub>(nh_, map_topic_free_, resolution_);

        // === Subscriber ===
        cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &GPOctoMapServer::cloudCallback, this);

        ROS_INFO("GPOctoMapServer initialized.");
    }

    ~GPOctoMapServer() {
        // Save Occupied Cloud
        if (occupied_cloud_ && !occupied_cloud_->empty()) {
            if (pcl::io::savePCDFileBinary("/home/ivana-rfal/gpcoctomap.pcd", *occupied_cloud_) == -1) {
                ROS_ERROR("Failed to save PCD file!");
            } else {
                ROS_INFO_STREAM("Saved occupancy map with " << occupied_cloud_->size()
                                << " points to /home/ivana-rfal/gpcoctomap.pcd");
            }
        } else {
            ROS_WARN("Occupied cloud is empty, not saving PCD file.");
        }

        // Compute and log timing stats
        if (!times_.empty()) {
            double sum = std::accumulate(times_.begin(), times_.end(), 0.0);
            double avg = sum / times_.size();
            double sq_sum = std::inner_product(times_.begin(), times_.end(), times_.begin(), 0.0);
            double stdev = std::sqrt(sq_sum / times_.size() - avg * avg);

            ROS_INFO_STREAM("Final Timing Stats: Average = " << avg 
                            << "s, StdDev = " << stdev 
                            << "s over " << times_.size() << " runs.");
        }
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
        tf::StampedTransform transform;
        try {
            listener_->waitForTransform(frame_id_, cloud->header.frame_id, cloud->header.stamp, ros::Duration(5.0));
            listener_->lookupTransform(frame_id_, cloud->header.frame_id, cloud->header.stamp, transform);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }

        ros::Time start = ros::Time::now();

        // === Transform origin ===
        la3dm::point3f origin;
        tf::Vector3 translation = transform.getOrigin();
        tf::Quaternion orientation = transform.getRotation();

        if (first_ || orientation.angleShortestPath(last_orientation_) > orientation_change_thresh_ ||
            translation.distance(last_position_) > position_change_thresh_) 
        {
            ROS_INFO_STREAM("Cloud received");

            first_ = false;
            last_position_ = translation;
            last_orientation_ = orientation;
            origin.x() = (float)translation.x();
            origin.y() = (float)translation.y();
            origin.z() = (float)translation.z();

            sensor_msgs::PointCloud2 cloud_map;
            pcl_ros::transformPointCloud(frame_id_, *cloud, cloud_map, *listener_);

            la3dm::PCLPointCloud::Ptr pcl_cloud(new la3dm::PCLPointCloud());
            pcl::fromROSMsg(cloud_map, *pcl_cloud);

            // === Downsample ===
            la3dm::PCLPointCloud filtered_cloud;
            pcl::VoxelGrid<pcl::PointXYZ> filterer;
            filterer.setInputCloud(pcl_cloud);
            filterer.setLeafSize(ds_resolution_, ds_resolution_, ds_resolution_);
            filterer.filter(filtered_cloud);

            if (filtered_cloud.size() > 5) {
                map_->insert_pointcloud(filtered_cloud, origin, (float)resolution_, (float)free_resolution_, (float)max_range_);
            }
            updated_ = true;
        }

        if (updated_) {
            publishMap(start);
            updated_ = false;
        }
    }

    void publishMap(const ros::Time& start) {
        ros::Time start_pub = ros::Time::now();

        m_pub_occ_->clear();
        m_pub_free_->clear();
        occupied_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

        for (auto it = map_->begin_leaf(); it != map_->end_leaf(); ++it) {
            la3dm::point3f p = it.get_loc();

            if (it.get_node().get_state() == la3dm::State::OCCUPIED) {
                if (original_size_) {
                    m_pub_occ_->insert_point3d(p.x(), p.y(), p.z(), min_z_, max_z_, it.get_size());
                    occupied_cloud_->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
                } else {
                    for (auto& n : it.get_pruned_locs()) {
                        m_pub_occ_->insert_point3d(n.x(), n.y(), n.z(), min_z_, max_z_, map_->get_resolution());
                    }
                }
            } else if (it.get_node().get_state() == la3dm::State::FREE) {
                if (original_size_) {
                    m_pub_free_->insert_point3d(p.x(), p.y(), p.z(), min_z_, max_z_, it.get_size(), it.get_node().get_prob());
                } else {
                    for (auto& n : it.get_pruned_locs()) {
                        m_pub_free_->insert_point3d(n.x(), n.y(), n.z(), min_z_, max_z_, map_->get_resolution(), it.get_node().get_prob());
                    }
                }
            }
        }

        ros::Time end_pub = ros::Time::now();
        double duration = (end_pub - start).toSec();
        times_.push_back(duration);

        double avg = std::accumulate(times_.begin(), times_.end(), 0.0) / times_.size();
        double sq_sum = std::inner_product(times_.begin(), times_.end(), times_.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / times_.size() - avg * avg);

        ROS_INFO_STREAM("Map published in " << duration << "s. Avg=" << avg << " SD=" << stdev);

        m_pub_occ_->publish();
        m_pub_free_->publish();

    }

    // === ROS members ===
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    std::unique_ptr<tf::TransformListener> listener_;
    std::unique_ptr<la3dm::MarkerArrayPub> m_pub_occ_, m_pub_free_;
    std::unique_ptr<la3dm::GPOctoMap> map_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr occupied_cloud_;

    // === Params ===
    std::string frame_id_, cloud_topic_, map_topic_occ_, map_topic_free_;
    double max_range_, resolution_, free_resolution_, ds_resolution_;
    double free_thresh_, occupied_thresh_, min_z_, max_z_;
    double sf2_, ell_, noise_, l_, min_var_, max_var_, max_known_var_;
    int block_depth_;
    bool original_size_;

    // === State ===
    tf::Vector3 last_position_;
    tf::Quaternion last_orientation_;
    bool first_ = true;
    bool updated_ = false;
    double position_change_thresh_ = 0.1;
    double orientation_change_thresh_ = 0.2;
    std::vector<double> times_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gpoctomap_server");
    ros::NodeHandle nh("~");

    GPOctoMapServer server(nh);

    ros::spin();
    return 0;
}
