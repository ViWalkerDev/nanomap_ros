#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include "openvdb/openvdb.h"
#include <openvdb/io/Stream.h>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "nanomap_msgs/msg/openvdb_grid.hpp"
#include "nanomap/manager/SimManager.h"
#include "nanomap/map/OccupancyMap.h"
#include "nanomap/config/Config.h"
#include "nanomap/nanomap.h"
#include "nanomap/sensor/SensorData.h"
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std::chrono_literals;
static rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_point_cloud;

std::string string_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

  using GridType = openvdb::FloatGrid;
  using TreeType = GridType::TreeType;
  using IterType = TreeType::ValueOnIter;
  using SensorData = nanomap::sensor::SensorData;
class LeafPublisherNode : public rclcpp::Node
{
public:
  LeafPublisherNode(openvdb::FloatGrid::Ptr grid, std::string node_name)
  : Node(node_name), count_(0) , grid_(grid)
  {
    std::string topic = grid_->getName();
    publisher_ = this->create_publisher<nanomap_msgs::msg::OpenvdbGrid>(topic, 10);
    auto timer_callback =
      [this]() -> void {
        auto curr_thread = string_thread_id();
        msg_.header.frame_id = "world";
        msg_.header.stamp = now();
        std::ostringstream ostr(std::ios_base::binary);
        openvdb::GridPtrVecPtr grids(new openvdb::GridPtrVec);
        gridCopy_ = grid_->deepCopy();
        grids->push_back(gridCopy_);
        openvdb::io::Stream(ostr).write(*grids);
        int size = ostr.str().size();
        msg_.size = size;
        msg_.data.resize(size);
        std::memcpy(msg_.data.data(), reinterpret_cast<unsigned char*>(const_cast<char*>(ostr.str().c_str())), size);
        gridCopy_->clear();
        this->publisher_->publish(msg_);
      };
    timer_ = this->create_wall_timer(50ms, timer_callback);
  }


private:
  nanomap_msgs::msg::OpenvdbGrid msg_;
  openvdb::FloatGrid::Ptr grid_;
  openvdb::FloatGrid::Ptr gridCopy_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nanomap_msgs::msg::OpenvdbGrid>::SharedPtr publisher_;
  size_t count_;
};


class SensorPublisher : public rclcpp::Node
{
public:
  SensorPublisher(std::shared_ptr<nanomap::config::Config> config, int agentIndex, int sensorIndex)
  : Node("sensor_publisher"), count_(0) , config_(config), agentIndex_(agentIndex), sensorIndex_(sensorIndex)
  {
    std::string topic = "Agent_" + std::to_string(agentIndex_) + "SensorData_" + config_->sensorData(sensorIndex_)->sensorName();
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);

    auto timer_callback =
      [this]() -> void {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud =
          boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        for (int x = 0; x < config_->sensorData(sensorIndex_)->sharedParameters()._pclSize; x++) {
          cloud->push_back(pcl::PointXYZ(config_->sensorData(sensorIndex_)->pointCloud().col(x)(0),
                                          config_->sensorData(sensorIndex_)->pointCloud().col(x)(1),
                                          config_->sensorData(sensorIndex_)->pointCloud().col(x)(2)));

        }
        pcl::toROSMsg(*cloud, msg_);
        // Extract current thread
        auto curr_thread = string_thread_id();
        msg_.header.frame_id = "world";
        msg_.header.stamp = now();
        this->publisher_->publish(msg_);
      };
    timer_ = this->create_wall_timer(50ms, timer_callback);
  }

private:
  sensor_msgs::msg::PointCloud2 msg_;
  std::shared_ptr<nanomap::config::Config> config_;
  int agentIndex_;
  int sensorIndex_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  size_t count_;
};


class PosePublisher : public rclcpp::Node
{
public:
  PosePublisher(nanomap::manager::SimManager& simManager, std::shared_ptr<nanomap::config::Config> config, int agentIndex, int sensorIndex)
  : Node("agent_pose_publisher"), manager_(&simManager), config_(config), agentIndex_(agentIndex), sensorIndex_(sensorIndex)
  {
    std::string topic = "Agent_" + std::to_string(agentIndex_) + "PoseStamped";
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);

    auto timer_callback =
    [this]() -> void {
      rclcpp::Time now = this->get_clock()->now();
      std::vector<std::shared_ptr<SensorData>> sensorData = config_->sensorData();
      msg_.header.stamp = now;
      msg_.header.frame_id = "world";
      msg_.pose.position.x = sensorData[sensorIndex_]->sharedParameters()._pose.position(0);
      msg_.pose.position.y = sensorData[sensorIndex_]->sharedParameters()._pose.position(1);
      msg_.pose.position.z = sensorData[sensorIndex_]->sharedParameters()._pose.position(2);
      msg_.pose.orientation.x = (double)sensorData[sensorIndex_]->sharedParameters()._pose.orientation.x();
      msg_.pose.orientation.y = (double)sensorData[sensorIndex_]->sharedParameters()._pose.orientation.y();
      msg_.pose.orientation.z = (double)sensorData[sensorIndex_]->sharedParameters()._pose.orientation.z();
      msg_.pose.orientation.w = (double)sensorData[sensorIndex_]->sharedParameters()._pose.orientation.w();
      this->publisher_->publish(msg_);
    };

      timer_ = this->create_wall_timer(20ms, timer_callback);
  }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    nanomap::manager::SimManager* manager_;
    std::shared_ptr<nanomap::config::Config> config_;
    std::string agentname_;
    int agentIndex_;
    int sensorIndex_;
    geometry_msgs::msg::PoseStamped msg_;
  };
//
class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher(nanomap::manager::SimManager& simManager, std::shared_ptr<nanomap::config::Config> config)
  : Node("agent_frame_publisher"), manager_(&simManager), config_(config)
  {
    this->declare_parameter<std::string>("agentname", "agent");
    this->get_parameter("agentname", agentname_);
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto timer_callback =
    [this]() -> void {
      rclcpp::Time now = this->get_clock()->now();
      geometry_msgs::msg::TransformStamped t;
      std::vector<nanomap::Pose> poses = manager_->agentPoses();
      std::vector<std::shared_ptr<SensorData>> sensorData = config_->sensorData();
      t.header.stamp = now;
      t.header.frame_id = "world";
      t.child_frame_id = agentname_.c_str();
      t.transform.translation.x = sensorData[0]->sharedParameters()._pose.position(0);
      t.transform.translation.y = sensorData[0]->sharedParameters()._pose.position(1);
      t.transform.translation.z = sensorData[0]->sharedParameters()._pose.position(2);
      t.transform.rotation.x = (double)sensorData[0]->sharedParameters()._pose.orientation.x();
      t.transform.rotation.y = (double)sensorData[0]->sharedParameters()._pose.orientation.y();
      t.transform.rotation.z = (double)sensorData[0]->sharedParameters()._pose.orientation.z();
      t.transform.rotation.w = (double)sensorData[0]->sharedParameters()._pose.orientation.w();
      tf_broadcaster_->sendTransform(t);

    };

      timer_ = this->create_wall_timer(20ms, timer_callback);
  }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nanomap::manager::SimManager* manager_;
    std::shared_ptr<nanomap::config::Config> config_;
    std::string agentname_;

  };
  class ViewLoopNode : public rclcpp::Node
  {
  public:
    ViewLoopNode(nanomap::manager::SimManager& manager, std::shared_ptr<nanomap::map::Map> map_ptr, std::string node_name)
    : Node(node_name), count_(0) , manager_(&manager), map_(map_ptr)
    {
      auto timer_callback =
        [this]() -> void {
          std::chrono::duration<double, std::milli> delay;
          auto start = std::chrono::high_resolution_clock::now();
          manager_->updateAgentViews(map_);
          auto end = std::chrono::high_resolution_clock::now();
          delay = end-start;
          std::cout << "simUpdateTime = " << delay.count() << "ms"<<  std::endl;
          };
      timer_ = this->create_wall_timer(50ms, timer_callback);
    }

  private:
    nanomap::manager::SimManager* manager_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<nanomap::map::Map> map_;
    size_t count_;
  };


class PoseLoopNode : public rclcpp::Node
{
public:
  PoseLoopNode(nanomap::manager::SimManager& manager, nanomap::Pose &pose,std::string node_name)
  : Node(node_name), count_(0) , manager_(&manager), pose_(&pose)
  {
    auto timer_callback =
      [this]() -> void {
          nanomap::Pose pose = *pose_;
          std::vector<nanomap::Pose> poses;
          poses.push_back(pose);
          manager_->updateAgentPoses(poses);
        };
    timer_ = this->create_wall_timer(10ms, timer_callback);
  }

private:
  nanomap::manager::SimManager* manager_;
  rclcpp::TimerBase::SharedPtr timer_;
  nanomap::Pose* pose_;
  size_t count_;
};

class PoseListenerNode : public rclcpp::Node
{
public:
  PoseListenerNode(nanomap::Pose &pose, std::string pose_topic, std::string node_name)
  : Node(node_name), count_(0) , pose_(&pose)
  {
  subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
         pose_topic, 10, std::bind(&PoseListenerNode::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    pose_->orientation = Eigen::Quaternionf(msg->pose.orientation.w,
                                           msg->pose.orientation.x,
                                           msg->pose.orientation.y,
                                           msg->pose.orientation.z
                                          );
    pose_->position = Eigen::Vector3f(msg->pose.position.x,
                                           msg->pose.position.y,
                                           msg->pose.position.z
                                          );
  }
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  nanomap::Pose* pose_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  openvdb::initialize();

  if(!(argc ==3)){
    std::cout << "please provide config path and agent pose topic for sim from published pose" << std::endl;
    return 0;
  }



  std::string simConfig = argv[1];
  std::shared_ptr<nanomap::config::Config> config = std::make_shared<nanomap::config::Config>(nanomap::config::Config(simConfig));
  std::shared_ptr<nanomap::map::Map> map_ptr = std::make_shared<nanomap::map::Map>(nanomap::map::Map(config->mappingRes(),
                                                                                                     config->probHitThres(),
                                                                                                     config->probMissThres()));
  nanomap::manager::SimManager simManager(config);

  Eigen::Vector3f rpy(0,0,0);
  Eigen::Vector3f xyz(0,0,0);

  nanomap::Pose pose;
  std::vector<std::shared_ptr<nanomap::agent::Agent>> agentData = config->agentData();
  pose = agentData[0]->pose();
  std::vector<nanomap::Pose> poses;
  poses.push_back(pose);
  simManager.updateAgentPoses(poses);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto occupiedVoxelPubNode = std::make_shared<LeafPublisherNode>(map_ptr->occupiedGrid(),"occupiedGridVoxels");
  nanomap::Pose sub_pose;
  auto poseLoop = std::make_shared<PoseLoopNode>(simManager, sub_pose, "poseLoop");
  auto viewLoop = std::make_shared<ViewLoopNode>(simManager, map_ptr,"viewLoop");
  auto framePublisher = std::make_shared<FramePublisher>(simManager, config);
  auto sensorPublisher = std::make_shared<SensorPublisher>(config, 0, 0);
  std::string agentTopicName = argv[2];
  std::cout << agentTopicName << std::endl;
  auto poseListener = std::make_shared<PoseListenerNode>(sub_pose, agentTopicName,"agentPoseListener");
  executor.add_node(poseListener);
  executor.add_node(poseLoop);
  executor.add_node(viewLoop);
  executor.add_node(occupiedVoxelPubNode);
  executor.add_node(framePublisher);
  executor.add_node(sensorPublisher);
  executor.spin();
  rclcpp::shutdown();
  simManager.closeHandler();
  return 0;
}
