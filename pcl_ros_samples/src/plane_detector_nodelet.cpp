#include "pcl_ros_samples/plane_detector_nodelet.h"

PLUGINLIB_EXPORT_CLASS(pcl_ros_samples::PlaneDetector, nodelet::Nodelet)

namespace pcl_ros_samples
{
  PlaneDetector::PlaneDetector()
  {
    // constructor process
  }
  PlaneDetector::~PlaneDetector()
  {
    // destructor process
  }

  void PlaneDetector::onInit()
  {
    NODELET_DEBUG("Initializing plane_detector_nodelet...");

    ros::NodeHandle& pnh = getPrivateNodeHandle();
    // pnh.getParam("value", value_);
    pub = pnh.advertise<sensor_msgs::PointCloud2>("plane", 1);
    sub = pnh.subscribe("input", 1, &PlaneDetector::applyFilter, this);

    /* from connection_base_nodelt in jsk_topic_tools
    bool use_multithread;
    ros::param::param<bool>("~use_multithread_callback", use_multithread, true);
    if (use_multithread) {
      NODELET_DEBUG("use multithread callback");
      nh_.reset (new ros::NodeHandle (getMTNodeHandle ()));
    } else {
      NODELET_DEBUG("use singlethread callback");
      nh_.reset (new ros::NodeHandle (getNodeHandle ()));
      } */
    // output_pub_ = advertise<sensor_msgs::PointCloud2>(*nh_, "plane", 1);
  }

  void PlaneDetector::applyFilter(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    // boost::mutex::scoped_lock lock(mutex_);

    std_msgs::Header header = input->header;
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2; // initialize object
    pcl_conversions::toPCL(*input, *cloud); // from input, generate content of cloud

    /* 1. Downsampling and Publishing voxel */
    // LeafSize: should small enough to caputure a leaf of plants
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud); // imutable pointer
    pcl::PCLPointCloud2 cloud_voxel; // cloud_filtered to cloud_voxel
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

    sor.setInputCloud(cloudPtr); // set input
    sor.setLeafSize(0.02f, 0.02f, 0.02f); // 2cm, model equation
    sor.filter(cloud_voxel); // set output

    sensor_msgs::PointCloud2 output_voxel;
    pcl_conversions::fromPCL(cloud_voxel, output_voxel);
    // pub_voxel.publish(output_voxel);

    /* 2. Filtering with RANSAC */
    // RANSAC initialization
    pcl::SACSegmentation<pcl::PointXYZ> seg; // Create the segmentation object
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    seg.setOptimizeCoefficients(true); // Optional
    // seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); // Use SACMODEL_PERPENDICULAR_PLANE instead
    seg.setMethodType(pcl::SAC_RANSAC);

    // minimum number of points calculated from N and distanceThres
    seg.setMaxIterations (1000); // N in RANSAC
    // seg.setDistanceThreshold(0.025); // 0.01 - 0.05 default: 0.02 // 閾値（しきい値）
    seg.setDistanceThreshold(0.050); // 0.01 - 0.05 default: 0.02 // 閾値（しきい値）

    // param for perpendicular
    seg.setAxis(Eigen::Vector3f (0.0, 0.0, 1.0));
    seg.setEpsAngle(pcl::deg2rad (10.0f)); // 5.0 -> 10.0

    // convert from PointCloud2 to PointXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud_voxel, *cloud_voxel_xyz);

    // RANSAC application
    seg.setInputCloud(cloud_voxel_xyz);
    seg.segment(*inliers, *coefficients); // values are empty at beginning

    // inliers.indices have array index of the points which are included as inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = inliers->indices.begin ();
         pit != inliers->indices.end (); pit++) {
      cloud_plane_xyz->points.push_back(cloud_voxel_xyz->points[*pit]);
    }
    // Organized as an image-structure
    cloud_plane_xyz->width = cloud_plane_xyz->points.size ();
    cloud_plane_xyz->height = 1;

    /* insert code to set arbitary frame_id setting
       such as frame_id ="/assemble_cloud_1"
       with respect to "/odom or /base_link" */

    // Conversions: PointCloud<T>, PCLPointCloud2, sensor_msgs::PointCloud2
    pcl::PCLPointCloud2 cloud_plane_pcl;
    pcl::toPCLPointCloud2(*cloud_plane_xyz, cloud_plane_pcl);
    sensor_msgs::PointCloud2 cloud_plane_ros;
    pcl_conversions::fromPCL(cloud_plane_pcl, cloud_plane_ros);
    // cloud_plane_ros.header.frame_id = "/base_link"; // odom -> /base_link
    cloud_plane_ros.header.frame_id = header.frame_id;
    cloud_plane_ros.header.stamp = header.stamp; // ros::Time::now() -> header.stamp
    pub.publish(cloud_plane_ros);
  }

  // void PlaneDetector::subscribe()
  // {
  //   // Subscription
  //   input_sub_ = nh_->subscribe("input", 1, &PlaneDetector::applyFilter, this);
  // }

  // void PlaneDetector::unsubscribe()
  // {
  //   // Stop Subscription
  //   input_sub_.shutdown();
  // }
  PLUGINLIB_DECLARE_CLASS(pcl_ros_samples, PlaneDetector, pcl_ros_samples::PlaneDetector, nodelet::Nodelet);
}
