#include <iostream>
#include <string>
#include <ros/ros.h> 
#include <pcl/point_cloud.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 

ros::Publisher pub;

int
main (int argc, char** argv)
{
  ROS_INFO("Start!!!!!!!");

  ros::init (argc, argv, "pcl_input");

  ros::NodeHandle nh;
  
  ros::Publisher pcl_pub;

  pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcd_data", 1);

  pcl::PointCloud<pcl::PointXYZ> testcloud1;
  pcl::PointCloud<pcl::PointXYZ> testcloud2;
  pcl::PointCloud<pcl::PointXYZ> cloud1;
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  sensor_msgs::PointCloud2 output1;
  sensor_msgs::PointCloud2 output2;
  ROS_INFO("Loading data.......");
  pcl::io::loadPCDFile ("/home/cj/catkin_ws/src/my_pcl_tutorial/src/000000.pcd", testcloud1);
  pcl::io::loadPCDFile ("/home/cj/catkin_ws/src/my_pcl_tutorial/src/000001.pcd", testcloud2);
  for(int i=0;i<testcloud1.size();i++){
    if(testcloud1.points[i].x*testcloud1.points[i].x+testcloud1.points[i].y*testcloud1.points[i].y<50*50){
      cloud1.push_back(testcloud1.points[i]);
    }
  }
  for(int j=0;j<testcloud2.size();j++){
    if(testcloud2.points[j].x*testcloud2.points[j].x+testcloud2.points[j].y*testcloud2.points[j].y<50*50){
      cloud2.push_back(testcloud2.points[j]);
    }
  }
  pcl::toROSMsg(cloud1, output1);
  pcl::toROSMsg(cloud2, output2);
  int count=0;
  output1.header.frame_id = "point_cloud";
  output2.header.frame_id = "point_cloud";
  ros::Rate loop_rate(1);
  while(ros::ok()){
      ROS_INFO("%d",count);
      if(count%2==1)pcl_pub.publish(output1);
      else pcl_pub.publish(output2);
      ros::spinOnce();
      loop_rate.sleep();
      count++;
  }
return 0;
}