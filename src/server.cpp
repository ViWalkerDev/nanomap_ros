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
#include "nanomap_msgs/msg/openvdb_grid.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "nanomap/manager/Manager.h"
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
    timer_ = this->create_wall_timer(100ms, timer_callback);
  }


private:
  nanomap_msgs::msg::OpenvdbGrid msg_;
  openvdb::FloatGrid::Ptr grid_;
  openvdb::FloatGrid::Ptr gridCopy_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nanomap_msgs::msg::OpenvdbGrid>::SharedPtr publisher_;
  size_t count_;
};

  class MapListenerNode : public rclcpp::Node
  {
  public:
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> view_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;


    MapListenerNode(nanomap::manager::Manager& manager, std::shared_ptr<nanomap::map::Map> map_ptr, std::string sensorTopic, std::string sensorName, std::string poseTopic,std::string node_name)
    : Node(node_name), manager_(&manager), map_(map_ptr), sensorTopic_(sensorTopic), sensorName_(sensorName), poseTopic_(poseTopic)
    {
      view_sub_.subscribe(this, sensorTopic_);
      pose_sub_.subscribe(this, poseTopic_);
      syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_policy>>(approximate_policy(10), view_sub_, pose_sub_);
      syncApproximate->registerCallback(&MapListenerNode::approximate_sync_callback,this);
    }


  private:
    void approximate_sync_callback(const std::shared_ptr<sensor_msgs::msg::PointCloud2>& sensor_msg, const std::shared_ptr<geometry_msgs::msg::PoseStamped>& pose_msg)
      {
            nanomap::Pose pose;
            pose.orientation = Eigen::Quaternionf(pose_msg->pose.orientation.w,
                                                   pose_msg->pose.orientation.x,
                                                   pose_msg->pose.orientation.y,
                                                   pose_msg->pose.orientation.z
                                                  );
            pose.position = Eigen::Vector3f(pose_msg->pose.position.x,
                                                   pose_msg->pose.position.y,
                                                   pose_msg->pose.position.z
                                                  );
            manager_->insertPointCloud(sensorName_, sensor_msg->width, sensor_msg->height,
               sensor_msg->point_step , &(sensor_msg->data[0]), pose, map_);

      }
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, geometry_msgs::msg::PoseStamped> approximate_policy;
      std::shared_ptr<message_filters::Synchronizer<approximate_policy>>syncApproximate;

    nanomap::manager::Manager* manager_;
    std::string sensorTopic_;
    std::string sensorName_;
    std::string poseTopic_;
    std::shared_ptr<nanomap::map::Map> map_;
  };


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  openvdb::initialize();

  if(argc != 5 ){
    std::cout << "please provide the path to main config file, the sensor publish topic, sensor name, and pose topic" << std::endl;
    return 0;
  }



  std::string simConfig = argv[1];
  std::string sensorTopic = argv[2];
  std::string sensorName = argv[3];
  std::string poseTopic = argv[4];
  std::shared_ptr<nanomap::config::Config> config = std::make_shared<nanomap::config::Config>(nanomap::config::Config(simConfig));
  std::shared_ptr<nanomap::map::Map> map_ptr = std::make_shared<nanomap::map::Map>(nanomap::map::Map(config->mappingRes(),
                                                                                                     config->probHitThres(),
                                                                                                     config->probMissThres()));
  nanomap::manager::Manager Manager(config);
  std::vector<std::shared_ptr<nanomap::agent::Agent>> agentData = config->agentData();
  rclcpp::executors::MultiThreadedExecutor executor;
  auto occupiedVoxelPubNode = std::make_shared<LeafPublisherNode>(map_ptr->occupiedGrid(),"occupiedGridVoxels");

  auto mapLoop = std::make_shared<MapListenerNode>(Manager, map_ptr, sensorTopic, sensorName, poseTopic, "mapLoop");

  executor.add_node(mapLoop);
  executor.add_node(occupiedVoxelPubNode);
  executor.spin();
  rclcpp::shutdown();


  Manager.closeHandler();
  return 0;
}
