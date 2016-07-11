// nodelet
#include <nodelet/nodelet.h>

// ROS
#include <ros/ros.h>
#include <ros/names.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl_ros/pcl_nodelet.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/angles.h>

namespace pcl_ros_samples
{

  class PlaneDetector : public nodelet::Nodelet
  {
  public:
    int sample_int;

  protected:
    boost::mutex mutex_;
    boost::shared_ptr<ros::NodeHandle> nh_;
    virtual void onInit(); // virtual enable a function to be overwritten
    virtual void applyFilter(const sensor_msgs::PointCloud2ConstPtr &input);
    ros::Subscriber input_sub_;
    ros::Publisher output_pub_;

    virtual void subscribe();
    virtual void unsubscribe();

    template<class T> ros::Publisher
    // advertise(ros::NodeHandle& nh,
    //             std::string topic, int queue_size)
    // {
    // boost::mutex::scoped_lock lock(connection_mutex_);
    // ros::SubscriberStatusCallback connect_cb
    // = boost::bind(&ConnectionBasedNodelet::connectionCallback, this, _1);
    // ros::SubscriberStatusCallback disconnect_cb
    // = boost::bind(&ConnectionBasedNodelet::connectionCallback, this, _1);
    // bool latch;
    // nh.param("latch", latch, false);
    // ros::Publisher ret = nh.advertise<T>(topic, queue_size,
    //                                        connect_cb,
    //                                        disconnect_cb,
    //                                        ros::VoidConstPtr(),
    //                                        latch);
    // publishers_.push_back(ret);
    // return ret;
    // }

  };
}
