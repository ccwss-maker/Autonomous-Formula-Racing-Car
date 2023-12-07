#include <iostream>
#include <string.h>  
#include <ros/ros.h>  
#include "sensor_msgs/PointCloud2.h"
#include "stdio.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace ros;
using namespace pcl;
using namespace sensor_msgs;

int *rand_rgb();

class SubscribeAndPublish  
{  
    public:  
    SubscribeAndPublish()  
    {  
        pcl_pub=n_.advertise<sensor_msgs::PointCloud2>("/pcl_output",1);
        //subscribe  
        pointcloud = n_.subscribe("/car/laser", 1, &SubscribeAndPublish::pointcloud_callback, this); 
    }  
    void pointcloud_callback(const PointCloud2ConstPtr & msg);  
    void input();
    private:  
    ros::NodeHandle n_;   
    Publisher pcl_pub;
    Subscriber pointcloud;
};  

int main(int argc, char **argv)  
{
  //Initiate ROS  
  ros::init(argc, argv, "subscribe_and_publish");  

  //Create an object of class SubscribeAndPublish that will take care of everything  
  SubscribeAndPublish test;  
  //ros::spin();
  ros::MultiThreadedSpinner s(1);  //多线程
  ros::spin(s); 

  return 0;  
}

void SubscribeAndPublish::pointcloud_callback(const PointCloud2ConstPtr & msg)
{
  PointCloud<PointXYZ>::Ptr rawCloud(new PointCloud<PointXYZ>);
  PointCloud2 src=*msg;
  fromROSMsg(src,*rawCloud);

  // // 降采样****************************************************
  // VoxelGrid<PointXYZ> vox;
	// PointCloud<PointXYZ>::Ptr vox_cloud(new PointCloud<PointXYZ>);
	// vox.setInputCloud(rawCloud);
	// vox.setLeafSize(0.1, 0.1, 0.1);
	// vox.filter(*vox_cloud);

//   //去噪声***************************************************
// 	StatisticalOutlierRemoval<PointXYZ>sor;
// 	PointCloud<PointXYZ>::Ptr sor_cloud(new PointCloud<PointXYZ>);
// 	sor.setInputCloud(rawCloud);
// 	sor.setMeanK(20);
// 	sor.setStddevMulThresh(0.02);
// 	sor.filter(*sor_cloud);
// //法向量求解**********************************************
// 	NormalEstimation<PointXYZ, Normal> ne;
// 	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
// 	PointCloud<Normal>::Ptr normal_cloud(new PointCloud<Normal>);
// 	ne.setInputCloud(sor_cloud);
// 	ne.setKSearch(20);
// 	ne.setSearchMethod(tree);
// 	ne.compute(*normal_cloud);
//   //基于法向量和曲率的区域生长算法**************************
// 	PointCloud<PointXYZ>::Ptr reg_cloud(new PointCloud<PointXYZ>);
// 	RegionGrowing<PointXYZ, Normal> reg;
// 	reg.setInputCloud(sor_cloud);
// 	reg.setSearchMethod(tree);
// 	reg.setNumberOfNeighbours(20);
// 	reg.setMinClusterSize(50);
// 	reg.setMaxClusterSize(100000);
// 	reg.setSmoothnessThreshold(3.0 / 180 * M_PI);
// 	reg.setCurvatureThreshold(1.0);
// 	reg.setInputNormals(normal_cloud);

// 	vector<PointIndices>clusters;
// 	reg.extract(clusters);

//   visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("Result of RegionGrowing"));
//   PointCloud<PointXYZ>::Ptr copy_cloud(new PointCloud<PointXYZ>);
//   for (int iter = 0; iter < clusters.size();iter++)
// 	{
		
// 		vector<int> inlier = clusters[iter].indices;
// 		copyPointCloud<PointXYZ>(*sor_cloud, inlier, *copy_cloud);
// 		stringstream ss;
// 		ss << "/home/wust/Personal_Data/back" << "RegionGrowing_clouds" << iter << ".pcd";
// 		// io::savePCDFileASCII(ss.str(), *copy_cloud);
// 		int *rgb = rand_rgb();//随机生成0-255的颜色值
// 		visualization::PointCloudColorHandlerCustom<PointXYZ>rgb1(copy_cloud, rgb[0], rgb[1], rgb[2]);//提取的平面不同彩色展示
// 		delete[]rgb;
// 		viewer->addPointCloud(copy_cloud, rgb1, ss.str());
// 		viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
// 	}
// 	viewer->spin();

  sensor_msgs::PointCloud2 output;
  toROSMsg(*rawCloud, output);
  pcl_pub.publish(*rawCloud);

}

int *rand_rgb()
{
  //随机产生颜色
  int *rgb = new int[3];
  rgb[0] = rand() % 255;
  rgb[1] = rand() % 255;
  rgb[2] = rand() % 255;
  return rgb;
}