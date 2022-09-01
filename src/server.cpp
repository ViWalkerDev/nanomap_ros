#include "ros/ros.h"

#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "nanomap_msgs/OpenvdbGrid.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//
// #include <tf2/transform_datatypes.h>
// #include <tf2_ros/buffer.h>


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <mutex>

#include "openvdb/openvdb.h"
#include <openvdb/io/Stream.h>

#include "nanomap/manager/Manager.h"
#include "nanomap/map/OccupancyMap.h"
#include "nanomap/config/Config.h"
#include "nanomap/nanomap.h"
#include "nanomap/sensor/SensorData.h"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
using namespace std::chrono_literals;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
  // class MapUpdater
  // {
  // public:
  //
  //
  //
  //   MapUpdater(nanomap::manager::Manager& manager, std::shared_ptr<nanomap::map::Map> map_ptr, std::string sensorName, ros::Time time_now, std::shared_ptr<std::mutex> mu)
  //   : manager_(&manager), map_(map_ptr), sensorName_(sensorName), sensorStamp_(time_now), mu_(mu)
  //   {
  //     count = 0;
  //   }
  //
  //
  //
  //   void processInput(const boost::shared_ptr<sensor_msgs::PointCloud2 const> sensor_msg, const boost::shared_ptr<geometry_msgs::PoseStamped const> pose_msg)
  //     {
  //       std::unique_lock<std::mutex> lock(*mu_);
  //       nanomap::Pose pose;
  //       pose.orientation = Eigen::Quaternionf(pose_msg->pose.orientation.w,
  //                                                pose_msg->pose.orientation.x,
  //                                                pose_msg->pose.orientation.y,
  //                                                pose_msg->pose.orientation.z
  //                                               );
  //       pose.position = Eigen::Vector3f(pose_msg->pose.position.x,
  //                                                pose_msg->pose.position.y,
  //                                                pose_msg->pose.position.z
  //                                               );
  //       //only perform a map update on a new sensor input
  //       //std::cout << pose.position(0) << "/" << pose.position(1) << "/" << pose.position(2) <<std::endl;
  //        if(sensorStamp_ != sensor_msg->header.stamp){
  //           //std::cout << "updating map" << std::endl;
  //           //std::cout << sensorName_ << "/" << sensor_msg->width << "/" << sensor_msg->height << "/" << sensor_msg->point_step << std::endl;
  //           sensorStamp_ = sensor_msg->header.stamp;
  //           manager_->insertPointCloud(sensorName_, sensor_msg->width, sensor_msg->height, sensor_msg->point_step , const_cast<unsigned char*>(&(sensor_msg->data[0])), pose, map_);
  //      }else{
  //         //update pose only TODO
  //      }
  //   }
  //   private:
  //   nanomap::manager::Manager* manager_;
  //   std::string sensorName_;
  //   std::shared_ptr<nanomap::map::Map> map_;
  //   ros::Time sensorStamp_;
  //   std::shared_ptr<std::mutex> mu_;
  //   int count;
  // };
  //
  // class MapUpdater
  // {
  // public:
  //
  //
  //
  //   MapUpdater(nanomap::manager::Manager& manager, std::shared_ptr<nanomap::map::Map> map_ptr, std::string sensorName, ros::Time time_now, std::shared_ptr<std::mutex> mu)
  //   : manager_(&manager), map_(map_ptr), sensorName_(sensorName), sensorStamp_(time_now), mu_(mu)
  //   {
  //     count = 0;
  //   }
  //
  //
  //
  //   private:
  //
  // };

boost::shared_ptr<sensor_msgs::PointCloud2 const> sensorMsg;
boost::shared_ptr<geometry_msgs::PoseStamped const> poseMsg;



void processInput(const boost::shared_ptr<sensor_msgs::PointCloud2 const> sensor_msg, const boost::shared_ptr<geometry_msgs::PoseStamped const> pose_msg)
  {
    poseMsg = pose_msg;
    sensorMsg = sensor_msg;
}

int main(int argc, char * argv[])
{

  if(argc != 8 ){
    std::cout << "please provide the path to main config file, the desired grid publishing topic, the desired grid publishing rate in hz (max 10), the sensor publish topic, the sensor name, the pose topic and the desired update rate" << std::endl;
    return 0;
  }

  ros::init(argc, argv, "NanoMapServer");
    ros::NodeHandle n;
  openvdb::initialize();
  //std::mutex(mu);
  std::shared_ptr<std::mutex> mu = std::make_shared<std::mutex>();

  std::string simConfig = argv[1];
  std::string gridTopic = argv[2];
  int         gridRate = atoi(argv[3]);
  std::string sensorTopic = argv[4];
  std::string sensorName = argv[5];
  std::string poseTopic = argv[6];
  int         updateRate = atoi(argv[7]);
  std::shared_ptr<nanomap::config::Config> config = std::make_shared<nanomap::config::Config>(nanomap::config::Config(simConfig));
  std::shared_ptr<nanomap::map::Map> map_ptr = std::make_shared<nanomap::map::Map>(nanomap::map::Map(config->mappingRes(),
                                                                                                     config->probHitThres(),
                                                                                                     config->probMissThres()));
  nanomap::manager::Manager Manager(config);
  ros::Time sensorStamp = ros::Time::now();
  //ros::Duration(3).sleep();
  //MapUpdater mapUpdater(Manager, map_ptr, sensorName, time_now, mu);

  //boost::shared_ptr<sensor_msgs::PointCloud2> sensorMsg;
  //boost::shared_ptr<geometry_msgs::PoseStamped> poseMsg;

  ros::Publisher grid_pub = n.advertise<nanomap_msgs::OpenvdbGrid>(gridTopic, 100);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sensor_sub(n, sensorTopic, 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(n, poseTopic, 1);
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sensor_sub, pose_sub);
//  sync.registerCallback(boost::bind(&MapUpdater::processInput, &mapUpdater, _1, _2));
  sync.registerCallback(boost::bind(&processInput, _1, _2));

  // Running at 10Hz
  ros::Rate loop_rate(updateRate);
  int count = 0;
  int countTarget = std::round(updateRate/gridRate)-1;
  if(countTarget < 0){
    countTarget = 0;
  }
  openvdb::FloatGrid::Ptr gridCopy;
  nanomap_msgs::OpenvdbGrid msg_;
  while(!sensorMsg && !poseMsg){
    std::cout << "Waiting for Messages..." << std::endl;
    ros::spinOnce();
    ros::Duration(1).sleep();
  }
  std::cout << "Messages Received!" << std::endl;
  while (ros::ok()) {

    //auto curr_thread = string_thread_id();
    if(sensorStamp != sensorMsg->header.stamp){
      std::cout << "Updating Map..." << std::endl;
      //        if(sensorStamp_ != sensor_msg->header.stamp){
      //           //std::cout << "updating map" << std::endl;
      sensorStamp = sensorMsg->header.stamp;
      nanomap::Pose pose;
      pose.orientation = Eigen::Quaternionf(poseMsg->pose.orientation.w,
                                               poseMsg->pose.orientation.x,
                                               poseMsg->pose.orientation.y,
                                               poseMsg->pose.orientation.z
                                              );
      pose.position = Eigen::Vector3f(poseMsg->pose.position.x,
                                               poseMsg->pose.position.y,
                                               poseMsg->pose.position.z
                                              );
      std::cout << pose.position(0) << "/" << pose.position(1) << "/" << pose.position(2) <<std::endl;
      std::cout << sensorName << "/" << sensorMsg->width << "/" << sensorMsg->height << "/" << sensorMsg->point_step << std::endl;

      Manager.insertPointCloud(sensorName, sensorMsg->width, sensorMsg->height, sensorMsg->point_step , const_cast<unsigned char*>(&(sensorMsg->data[0])), pose, map_ptr);
    }


    if(count >= countTarget){
      if(map_ptr->occupiedGrid()->tree().leafCount() > 0){
        std::cout << "Publishing Map..." << std::endl;
      //  std::unique_lock<std::mutex> lock(*mu);
        msg_.header.frame_id = "world";
        msg_.header.stamp = ros::Time::now();
        std::ostringstream ostr(std::ios_base::binary);
        openvdb::GridPtrVecPtr grids(new openvdb::GridPtrVec);
        gridCopy = map_ptr->occupiedGrid()->deepCopy();
        grids->push_back(gridCopy);
        openvdb::io::Stream(ostr).write(*grids);
        int size = ostr.str().size();
        msg_.size = size;
        msg_.data.resize(size);
        std::memcpy(msg_.data.data(), reinterpret_cast<unsigned char*>(const_cast<char*>(ostr.str().c_str())), size);
        //gridCopy->clear();
        grid_pub.publish(msg_);
        count = 0;
      }

      //openvdb::io::File("test.vdb").write({map_ptr->occupiedGrid()});
    }
		ros::spinOnce();
		loop_rate.sleep();
		++count;
  }
  Manager.closeHandler();
  return 0;
}
