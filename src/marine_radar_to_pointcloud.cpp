#include <ros/ros.h>
#include <marine_sensor_msgs/RadarSector.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"


ros::Publisher pointcloud_publisher;
float detection_threshold = 0.0;

float last_increment = 0.0;

void radarSectorCallback(const marine_sensor_msgs::RadarSectorConstPtr &msg)
{
  //ROS_INFO_STREAM("angle min: " << msg->angle_min << " angle max: " << msg->angle_max << " increment: " << msg->angle_increment);

  // hack for bug in earlier halo driver
  auto angle_increment = msg->angle_increment;
  if(angle_increment < 0.0)
    last_increment = angle_increment;
  else
    angle_increment = last_increment;


  if(!msg->intensities.empty())
  {
    pcl::PointCloud<pcl::PointXYZI> pc;
    pc.header.frame_id = msg->header.frame_id;
    pc.header.stamp = msg->header.stamp.toNSec()/1000;

    for(int i = 0; i < msg->intensities.size(); i++)
    {
      double angle = msg->angle_start + i*angle_increment;
      double c = cos(angle);
      double s = sin(angle);
      float range_increment = (msg->range_max - msg->range_min)/float(msg->intensities[i].echoes.size());

      for(int j = 0; j < msg->intensities[i].echoes.size(); j++)
      {
        if(msg->intensities[i].echoes[j] > detection_threshold)
        {
          auto range = msg->range_min+ j*range_increment;
          pcl::PointXYZI p;
          p.x = range*c;
          p.y = range*s;
          p.z = 0.0;
          p.intensity = msg->intensities[i].echoes[j];
          pc.push_back(p);
        }
      }
    }
    pointcloud_publisher.publish(pc);
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "marine_radar_to_pointcloud");

  ros::NodeHandle nh, pnh("~");

  detection_threshold = pnh.param("detection_threshold", 0.0);

  ros::Subscriber radar_subscriber = nh.subscribe("radar_data", 50, &radarSectorCallback);

  pointcloud_publisher = pnh.advertise<pcl::PointCloud<pcl::PointXYZI> >("pointcloud", 10);
    
  ros::spin();
  return 0;
}    
