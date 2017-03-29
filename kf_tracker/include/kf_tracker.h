#include <iostream>
#include <string.h>
#include <fstream>
#include <algorithm>
#include <iterator>
#include "featureDetection.h"
#include "CKalmanFilter.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/video/tracking.hpp"
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
 #include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <limits>
#include <utility>
#include <pcl/registration/correspondence_estimation.h>

using namespace std;
using namespace cv;

class KFTracker
{
public:
  KFTracker();

private:
  tf::TransformListener* tran;
  tf::TransformListener lr(ros::Duration(10));
  ros::Subscriber sub;
  ros::Publisher pub_cluster0, pub_cluster1, pub_cluster2,
                 pub_cluster3, pub_cluster4, pub_cluster5
  ros::Publisher objID_pub;
  ros::Publisher cc_pos;
  ros::Publisher markerPub;
  std::vector<geometry_msgs::Point> prevClusterCenters;
  std::vector<int> objID;// Output of the data association using KF
  bool firstFrame;

  int stateDim;// [x,y,v_x,v_y]//,w,h]
  int measDim;// [z_x,z_y,z_w,z_h]
  int ctrlDim;

  cv::KalmanFilter KF0, KF1, KF2, KF3, KF4, KF5; // TODO put these into vectors

  void init_kf(const sensor_msgs::PointCloud2ConstPtr& input);
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr &input);
  void publish_cloud(ros::Publisher &pub, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster,
                      const sensor_msgs::PointCloud2ConstPtr &input);
}
