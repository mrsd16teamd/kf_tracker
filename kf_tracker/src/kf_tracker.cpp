#include "kf_tracker.h"
#include "kf_tracker_utils.h"

KFTracker::KFTracker()
{
  ros::NodeHandle nh;
  rsub = nh.subscribe ("cloudnear", 1, &KFTracker::cloud_cb, this);

  pub_cluster0 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_0", 1);
  pub_cluster1 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_1", 1);
  pub_cluster2 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_2", 1);
  pub_cluster3 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_3", 1);
  pub_cluster4 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_4", 1);
  pub_cluster5 = nh.advertise<sensor_msgs::PointCloud2> ("cluster_5", 1);

  objID_pub = nh.advertise<std_msgs::Int32MultiArray>("obj_id", 1);

  cc_pos=nh.advertise<std_msgs::Float32MultiArray>("ccs",100);//clusterCenter1
  markerPub= nh.advertise<visualization_msgs::MarkerArray> ("viz",1);

  state_dim = 4; // TODO initialize these correctly
  measDim = 2;
  ctrlDim = 0;

  // TODO fix this
  cv::KalmanFilter KF0(stateDim,measDim,ctrlDim,CV_32F);
  cv::KalmanFilter KF1(stateDim,measDim,ctrlDim,CV_32F);
  cv::KalmanFilter KF2(stateDim,measDim,ctrlDim,CV_32F);
  cv::KalmanFilter KF3(stateDim,measDim,ctrlDim,CV_32F);
  cv::KalmanFilter KF4(stateDim,measDim,ctrlDim,CV_32F);
  cv::KalmanFilter KF5(stateDim,measDim,ctrlDim,CV_32F);

  firstFrame = true;

}

// TODO put these somewhere
cv::Mat state(stateDim,1,CV_32F);
cv::Mat_<float> measurement(2,1);


/*
//Count unique object IDs. just to make sure same ID has not been assigned to two KF_Trackers.
int countIDs(vector<int> v)
{
    transform(v.begin(), v.end(), v.begin(), abs); // O(n) where n = distance(v.end(), v.begin())
    sort(v.begin(), v.end()); // Average case O(n log n), worst case O(n^2) (usually implemented as quicksort.
    // To guarantee worst case O(n log n) replace with make_heap, then sort_heap.

    // Unique will take a sorted range, and move things around to get duplicated
    // items to the back and returns an iterator to the end of the unique section of the range
    auto unique_end = unique(v.begin(), v.end()); // Again n comparisons
    return distance(unique_end, v.begin()); // Constant time for random access iterators (like vector's)
}
*/

/*
objID: vector containing the IDs of the clusters that should be associated with each KF_Tracker
objID[0] corresponds to KFT0, objID[1] corresponds to KFT1 etc.
*/

void KFT(const std_msgs::Float32MultiArray ccs)
{
  // First predict, to update the internal statePre variable
  std::vector<cv::Mat> pred;
  pred.push_back(KF0.predict());
  pred.push_back(KF1.predict());
  pred.push_back(KF2.predict());
  pred.push_back(KF3.predict());
  pred.push_back(KF4.predict());
  pred.push_back(KF5.predict());

  // Get measurements
  // Extract the position of the clusters forom the multiArray. To check if the data
  // coming in, check the .z (every third) coordinate and that will be 0.0
  std::vector<geometry_msgs::Point> clusterCenters;//clusterCenters

  int i=0;
  for (std::vector<float>::const_iterator it=ccs.data.begin();it!=ccs.data.end();it+=3)
  {
      geometry_msgs::Point pt;
      pt.x=*it;
      pt.y=*(it+1);
      pt.z=*(it+2);

      clusterCenters.push_back(pt);
  }

  std::vector<geometry_msgs::Point> KFpredictions;
  i=0;
  for (std::vector<cv::Mat>::iterator it=pred.begin();it!=pred.end();it++)
  {
    geometry_msgs::Point pt;
    pt.x=(*it).at<float>(0);
    pt.y=(*it).at<float>(1);
    pt.z=(*it).at<float>(2);

    KFpredictions.push_back(pt);
  }

  // Find the cluster that is more probable to be belonging to a given KF.
  objID.clear();//Clear the objID vector
  objID.resize(6);//Allocate default elements so that [i] doesnt segfault. Should be done better
  // Copy clusterCentres for modifying it and preventing multiple assignments of the same ID
  std::vector<geometry_msgs::Point> copyOfClusterCenters(clusterCenters);
  std::vector<std::vector<float> > distMat;

  for(int filterN=0;filterN<6;filterN++)
  {
    std::vector<float> distVec;
    for(int n=0;n<6;n++)
    {
      distVec.push_back(euclidean_distance(KFpredictions[filterN],copyOfClusterCenters[n]));
    }

    distMat.push_back(distVec);

    /*// Based on distVec instead of distMat (global min). Has problems with the person's leg going out of scope
     int ID=std::distance(distVec.begin(),min_element(distVec.begin(),distVec.end()));
     //cout<<"finterlN="<<filterN<<"   minID="<<ID
     objID.push_back(ID);
    // Prevent assignment of the same object ID to multiple clusters
     copyOfClusterCenters[ID].x=100000;// A large value so that this center is not assigned to another cluster
     copyOfClusterCenters[ID].y=10000;
     copyOfClusterCenters[ID].z=10000;
    */
   cout<<"filterN="<<filterN<<"\n";
  }

  for(int clusterCount=0;clusterCount<6;clusterCount++)
  {
    // 1. Find min(distMax)==> (i,j);
    std::pair<int,int> minIndex(findIndexOfMin(distMat));

    // 2. objID[i]=clusterCenters[j]; counter++
    objID[minIndex.first]=minIndex.second;

    // 3. distMat[i,:]=10000; distMat[:,j]=10000
    distMat[minIndex.first]=std::vector<float>(6,10000.0);// Set the row to a high number.
    for(int row=0;row<distMat.size();row++)//set the column to a high number
    {
      distMat[row][minIndex.second]=10000.0;
    }
  }

  visualization_msgs::MarkerArray clusterMarkers;

  for (int i=0;i<6;i++)
  {
    visualization_msgs::Marker m;

    m.id=i;
    m.type=visualization_msgs::Marker::CUBE;
    m.header.frame_id="/laser";
    m.scale.x=0.3;         m.scale.y=0.3;         m.scale.z=0.3;
    m.action=visualization_msgs::Marker::ADD;
    m.color.a=1.0;
    m.color.r=0;
    m.color.g=1;
    m.color.b=0;

    //geometry_msgs::Point clusterC(clusterCenters.at(objID[i]));
    geometry_msgs::Point clusterC(KFpredictions[i]);
    m.pose.position.x=clusterC.x;
    m.pose.position.y=clusterC.y;
    m.pose.position.z=clusterC.z;

    clusterMarkers.markers.push_back(m);
  }

  prevClusterCenters=clusterCenters;

  markerPub.publish(clusterMarkers);

  std_msgs::Int32MultiArray obj_id;
  for(std::vector<int>::iterator it=objID.begin();it!=objID.end();it++)
      obj_id.data.push_back(*it);

  // Publish the object IDs
  objID_pub.publish(obj_id);

  // convert clusterCenters from geometry_msgs::Point to floats
  std::vector<std::vector<float> > cc;
  for (int i=0;i<6;i++)
  {
    vector<float> pt;
    pt.push_back(clusterCenters[objID[i]].x);
    pt.push_back(clusterCenters[objID[i]].y);
    pt.push_back(clusterCenters[objID[i]].z);

    cc.push_back(pt);
  }
  float meas0[2]={cc[0].at(0),cc[0].at(1)};
  float meas1[2]={cc[1].at(0),cc[1].at(1)};
  float meas2[2]={cc[2].at(0),cc[2].at(1)};
  float meas3[2]={cc[3].at(0),cc[3].at(1)};
  float meas4[2]={cc[4].at(0),cc[4].at(1)};
  float meas5[2]={cc[5].at(0),cc[5].at(1)};

  // The update phase
  cv::Mat meas0Mat=cv::Mat(2,1,CV_32F,meas0);
  cv::Mat meas1Mat=cv::Mat(2,1,CV_32F,meas1);
  cv::Mat meas2Mat=cv::Mat(2,1,CV_32F,meas2);
  cv::Mat meas3Mat=cv::Mat(2,1,CV_32F,meas3);
  cv::Mat meas4Mat=cv::Mat(2,1,CV_32F,meas4);
  cv::Mat meas5Mat=cv::Mat(2,1,CV_32F,meas5);

  if (!(meas0Mat.at<float>(0,0)==0.0f || meas0Mat.at<float>(1,0)==0.0f))
      Mat estimated0 = KF0.correct(meas0Mat);
  if (!(meas1[0]==0.0f || meas1[1]==0.0f))
      Mat estimated1 = KF1.correct(meas1Mat);
  if (!(meas2[0]==0.0f || meas2[1]==0.0f))
      Mat estimated2 = KF2.correct(meas2Mat);
  if (!(meas3[0]==0.0f || meas3[1]==0.0f))
      Mat estimated3 = KF3.correct(meas3Mat);
  if (!(meas4[0]==0.0f || meas4[1]==0.0f))
      Mat estimated4 = KF4.correct(meas4Mat);
  if (!(meas5[0]==0.0f || meas5[1]==0.0f))
      Mat estimated5 = KF5.correct(meas5Mat);
} // KFT

void publish_cloud(ros::Publisher& pub, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster, const sensor_msgs::PointCloud2ConstPtr& input){
  sensor_msgs::PointCloud2::Ptr clustermsg (new sensor_msgs::PointCloud2);
  pcl::toROSMsg (*cluster , *clustermsg);
  clustermsg->header.frame_id = input->header.frame_id;
  clustermsg->header.stamp = ros::Time::now();
  pub.publish (*clustermsg);
}

void KFTracker::init_kf(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Initialize 6 Kalman Filters; Assuming 6 max objects in the dataset.
  // Could be made generic by creating a Kalman Filter only when a new object is detected

  float dvx=0.01f; //1.0
  float dvy=0.01f;//1.0
  float dx=1.0f;
  float dy=1.0f;
  KF0.transitionMatrix = (Mat_<float>(4, 4) << dx,0,1,0,   0,dy,0,1,  0,0,dvx,0,  0,0,0,dvy);
  KF1.transitionMatrix = (Mat_<float>(4, 4) << dx,0,1,0,   0,dy,0,1,  0,0,dvx,0,  0,0,0,dvy);
  KF2.transitionMatrix = (Mat_<float>(4, 4) << dx,0,1,0,   0,dy,0,1,  0,0,dvx,0,  0,0,0,dvy);
  KF3.transitionMatrix = (Mat_<float>(4, 4) << dx,0,1,0,   0,dy,0,1,  0,0,dvx,0,  0,0,0,dvy);
  KF4.transitionMatrix = (Mat_<float>(4, 4) << dx,0,1,0,   0,dy,0,1,  0,0,dvx,0,  0,0,0,dvy);
  KF5.transitionMatrix = (Mat_<float>(4, 4) << dx,0,1,0,   0,dy,0,1,  0,0,dvx,0,  0,0,0,dvy);

  cv::setIdentity(KF0.measurementMatrix);
  cv::setIdentity(KF1.measurementMatrix);
  cv::setIdentity(KF2.measurementMatrix);
  cv::setIdentity(KF3.measurementMatrix);
  cv::setIdentity(KF4.measurementMatrix);
  cv::setIdentity(KF5.measurementMatrix);

  // Process Noise Covariance Matrix Q
  // [ Ex 0  0    0 0    0 ]
  // [ 0  Ey 0    0 0    0 ]
  // [ 0  0  Ev_x 0 0    0 ]
  // [ 0  0  0    1 Ev_y 0 ]
  //// [ 0  0  0    0 1    Ew ]
  //// [ 0  0  0    0 0    Eh ]
  float sigmaP=0.01;
  float sigmaQ=0.1;
  setIdentity(KF0.processNoiseCov, Scalar::all(sigmaP));
  setIdentity(KF1.processNoiseCov, Scalar::all(sigmaP));
  setIdentity(KF2.processNoiseCov, Scalar::all(sigmaP));
  setIdentity(KF3.processNoiseCov, Scalar::all(sigmaP));
  setIdentity(KF4.processNoiseCov, Scalar::all(sigmaP));
  setIdentity(KF5.processNoiseCov, Scalar::all(sigmaP));
  // Meas noise cov matrix R
  cv::setIdentity(KF0.measurementNoiseCov, cv::Scalar(sigmaQ));//1e-1
  cv::setIdentity(KF1.measurementNoiseCov, cv::Scalar(sigmaQ));
  cv::setIdentity(KF2.measurementNoiseCov, cv::Scalar(sigmaQ));
  cv::setIdentity(KF3.measurementNoiseCov, cv::Scalar(sigmaQ));
  cv::setIdentity(KF4.measurementNoiseCov, cv::Scalar(sigmaQ));
  cv::setIdentity(KF5.measurementNoiseCov, cv::Scalar(sigmaQ));

  // Process the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  /* Creating the KdTree from input point cloud*/
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::fromROSMsg (*input, *input_cloud);

  tree->setInputCloud (input_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.04);
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (200);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
  /* Extract the clusters out of pc and save indices in cluster_indices.*/
  ec.extract (cluster_indices);


  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;
  // Vector of cluster pointclouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cluster_vec;
  // Cluster centroids
  std::vector<pcl::PointXYZ> clusterCentroids;

  for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    float x=0.0; float y=0.0;
    int numPts=0;

    for(pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      cloud_cluster->points.push_back(input_cloud->points[*pit]);
      x+=input_cloud->points[*pit].x;
      y+=input_cloud->points[*pit].y;
      numPts++;
    }

    pcl::PointXYZ centroid;
    centroid.x=x/numPts;
    centroid.y=y/numPts;
    centroid.z=0.0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr source, target;

    target = cloud_cluster;
    source = input_cloud;
    // ... read or fill in source and target
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
    est.setInputSource (source);
    est.setInputTarget (target);
    //est.setMaxCorrespondenceDistance(4);
    pcl::Correspondences all_correspondences;
    // Determine all reciprocal correspondences
    est.determineReciprocalCorrespondences (all_correspondences);

    cluster_vec.push_back(cloud_cluster);

    //Get the centroid of the cluster
    clusterCentroids.push_back(centroid);
  }

  //Ensure at least 6 clusters exist to publish (later clusters may be empty)
  while (cluster_vec.size() < 6){
    pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    empty_cluster->points.push_back(pcl::PointXYZ(0,0,0));
    cluster_vec.push_back(empty_cluster);
  }

  while (clusterCentroids.size()<6)
  {
    pcl::PointXYZ centroid;
    centroid.x=0.0;
    centroid.y=0.0;
    centroid.z=0.0;

    clusterCentroids.push_back(centroid);
  }

  // Set initial state
  KF0.statePre.at<float>(0)=clusterCentroids.at(0).x;
  KF0.statePre.at<float>(1)=clusterCentroids.at(0).y;
  KF0.statePre.at<float>(2)=0;// initial v_x
  KF0.statePre.at<float>(3)=0;//initial v_y

  // Set initial state
  KF1.statePre.at<float>(0)=clusterCentroids.at(1).x;
  KF1.statePre.at<float>(1)=clusterCentroids.at(1).y;
  KF1.statePre.at<float>(2)=0;// initial v_x
  KF1.statePre.at<float>(3)=0;//initial v_y

  // Set initial state
  KF2.statePre.at<float>(0)=clusterCentroids.at(2).x;
  KF2.statePre.at<float>(1)=clusterCentroids.at(2).y;
  KF2.statePre.at<float>(2)=0;// initial v_x
  KF2.statePre.at<float>(3)=0;//initial v_y


  // Set initial state
  KF3.statePre.at<float>(0)=clusterCentroids.at(3).x;
  KF3.statePre.at<float>(1)=clusterCentroids.at(3).y;
  KF3.statePre.at<float>(2)=0;// initial v_x
  KF3.statePre.at<float>(3)=0;//initial v_y

  // Set initial state
  KF4.statePre.at<float>(0)=clusterCentroids.at(4).x;
  KF4.statePre.at<float>(1)=clusterCentroids.at(4).y;
  KF4.statePre.at<float>(2)=0;// initial v_x
  KF4.statePre.at<float>(3)=0;//initial v_y

  // Set initial state
  KF5.statePre.at<float>(0)=clusterCentroids.at(5).x;
  KF5.statePre.at<float>(1)=clusterCentroids.at(5).y;
  KF5.statePre.at<float>(2)=0;// initial v_x
  KF5.statePre.at<float>(3)=0;//initial v_y

  firstFrame=false;

  for (int i=0;i<6;i++)
  {
    geometry_msgs::Point pt;
    pt.x=clusterCentroids.at(i).x;
    pt.y=clusterCentroids.at(i).y;
    prevClusterCenters.push_back(pt);
  }

}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // If this is the first frame, initialize kalman filters for the clustered objects
  if (firstFrame)
  {
    init_kf(input);
  }

  else
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    /* Creating the KdTree from input point cloud*/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::fromROSMsg (*input, *input_cloud);

    tree->setInputCloud (input_cloud);

    /* Here we are creating a vector of PointIndices, which contains the actual index
    * information in a vector<int>. The indices of each detected cluster are saved here.
    * Cluster_indices is a vector containing one instance of PointIndices for each detected
    * cluster. Cluster_indices[0] contain all indices of the first cluster in input point cloud.
    */
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.04);
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (200);
    ec.setSearchMethod (tree);
    ec.setInputCloud (input_cloud);
    //cout<<"PCL init successfull\n";
    /* Extract the clusters out of pc and save indices in cluster_indices.*/
    ec.extract (cluster_indices);


    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    // Vector of cluster pointclouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cluster_vec;

    // Cluster centroids
    std::vector<pcl::PointXYZ> clusterCentroids;

    for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      float x=0.0; float y=0.0;
      int numPts=0;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for(pit = it->indices.begin(); pit != it->indices.end(); pit++)
      {
        cloud_cluster->points.push_back(input_cloud->points[*pit]);

        x+=input_cloud->points[*pit].x;
        y+=input_cloud->points[*pit].y;
        numPts++;
      }

      pcl::PointXYZ centroid;
      centroid.x=x/numPts;
      centroid.y=y/numPts;
      centroid.z=0.0;

      pcl::PointCloud<pcl::PointXYZ>::Ptr source, target;

      target = cloud_cluster;
      source = input_cloud;
      // ... read or fill in source and target
      pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
      est.setInputSource (source);
      est.setInputTarget (target);
      //est.setMaxCorrespondenceDistance(4);
      pcl::Correspondences all_correspondences;
      // Determine all reciprocal correspondences
      est.determineReciprocalCorrespondences (all_correspondences);

      if (centroid.x < 0.7  && centroid.x > -0.7 && centroid.y > -0.7 && centroid.y <0.7 && centroid.x != 0 && centroid.y != 0 )
      {
        cluster_vec.push_back(cloud_cluster);
        clusterCentroids.push_back(centroid);
      }
    }

    //Ensure at least 6 clusters exist to publish (later clusters may be empty)
    while (cluster_vec.size() < 6){
      pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      empty_cluster->points.push_back(pcl::PointXYZ(0,0,0));
      cluster_vec.push_back(empty_cluster);
    }

    while (clusterCentroids.size()<6)
    {
      pcl::PointXYZ centroid;
      centroid.x=0.0;
      centroid.y=0.0;
      centroid.z=0.0;

      clusterCentroids.push_back(centroid);
    }

    std_msgs::Float32MultiArray cc;
    std_msgs::Float32MultiArray cctemp;
    Eigen::Vector4f obstaclepoint;
    geometry_msgs::PointStamped bill;
    geometry_msgs::PointStamped m;
    bill.header.frame_id = "map";

    for(int i=0;i<6;i++)
    {
      cc.data.push_back(clusterCentroids.at(i).x);
      cc.data.push_back(clusterCentroids.at(i).y);
      cc.data.push_back(clusterCentroids.at(i).z);

      if ( i == 0)
      {
        cctemp.data.push_back(clusterCentroids.at(i).x);
        cctemp.data.push_back(clusterCentroids.at(i).y);
        cctemp.data.push_back(clusterCentroids.at(i).z);
        obstaclepoint[0] = clusterCentroids.at(i).x;
        obstaclepoint[1] = clusterCentroids.at(i).y;
        obstaclepoint[2] = clusterCentroids.at(i).z;
      }
    }

    cc_pos.publish(cctemp);// Publish cluster mid-points.
    KFT(cc);
    int i=0;
    bool publishedCluster[6];


    for(std::vector<int>::iterator it=objID.begin();it!=objID.end();it++)
    {
      // TODO get rid of this monstrosity
      switch(i)
      {
      case 0: {
        publish_cloud(pub_cluster0,cluster_vec[*it],input);
        publishedCluster[i]=true;//Use this flag to publish only once for a given obj ID
        i++;
        break;
        }
        case 1: {
        publish_cloud(pub_cluster1,cluster_vec[*it],input);
        publishedCluster[i]=true;//Use this flag to publish only once for a given obj ID
        i++;
        break;
        }
        case 2: {
        publish_cloud(pub_cluster2,cluster_vec[*it],input);
        publishedCluster[i]=true;//Use this flag to publish only once for a given obj ID
        i++;
        break;
        }
        case 3: {
        publish_cloud(pub_cluster3,cluster_vec[*it],input);
        publishedCluster[i]=true;//Use this flag to publish only once for a given obj ID
        i++;
        break;
        }
        case 4: {
        publish_cloud(pub_cluster4,cluster_vec[*it],input);
        publishedCluster[i]=true;//Use this flag to publish only once for a given obj ID
        i++;
        break;
        }
        case 5: {
        publish_cloud(pub_cluster5,cluster_vec[*it],input);
        publishedCluster[i]=true;//Use this flag to publish only once for a given obj ID
        i++;
        break;
        }
        default: break;
      }
    }
  } //else
} //cloud_cb

int main(int argc, char** argv)
{
  ros::init (argc,argv,"kf_tracker");

  KFTracker kf_tracker;

  ros::spin();
}
