#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <bondcpp/bond.h>
#include <cloud_3dpc_extractor_msgs/NewProcNode.h>
#include <fstream>
#include <queue>
#include <limits>

bool measure_time;
std::ofstream time_file_proc;
std::ofstream time_file_comm;

namespace cloud_3DPC_extractor {

  class StereoCamBuffer {

  private:

    image_geometry::StereoCameraModel model;
  
    std::queue<bond::Bond*> bonds;
    std::map< std::string, std::string > topics;
    std::map< std::string, image_transport::CameraPublisher > left_cams;
    std::map< std::string, image_transport::CameraPublisher > right_cams;
  
    image_transport::CameraSubscriber camera_sub_;
    bool replace_;

    ros::Time last_update_time;
  
    // Publisher part
  
    ros::Publisher feature_pub;
    
    // subscriber
  
    image_transport::SubscriberFilter left_sub_, right_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

    boost::shared_ptr<ExactSync> exact_sync_;
    boost::shared_ptr<ApproximateSync> approximate_sync_;
    int queue_size_;
  
    // for sync checking
  
    ros::WallTimer check_synced_timer_;
    int left_received_, right_received_, left_info_received_, right_info_received_, all_received_;

    double heartbeat_period;
    double heartbeat_timeout;

     
    // for sync checking
  
    static void increment(int* value)
    {
      ++(*value);
    }

    bool check_if_broken;
  
  public:
    
    ros::Subscriber nodesSub;
    ros::CallbackQueue my_callback_queue;

    void dataCb(const sensor_msgs::ImageConstPtr& l_image_msg,
		const sensor_msgs::ImageConstPtr& r_image_msg,
		const sensor_msgs::CameraInfoConstPtr& l_info_msg,
		const sensor_msgs::CameraInfoConstPtr& r_info_msg)
    {
 
      // For sync error checking
      ++all_received_; 
   
      // call implementation
      imageCallback(l_image_msg, r_image_msg, l_info_msg, r_info_msg);
    }


    void checkInputsSynchronized()
    {
      int threshold = 3 * all_received_;
      if (left_received_ >= threshold || right_received_ >= threshold || 
	  left_info_received_ >= threshold || right_info_received_ >= threshold) {
	ROS_WARN("[stereo_processor] Low number of synchronized left/right/left_info/right_info tuples received.\n"
		 "Left images received:       %d (topic '%s')\n"
		 "Right images received:      %d (topic '%s')\n"
		 "Left camera info received:  %d (topic '%s')\n"
		 "Right camera info received: %d (topic '%s')\n"
		 "Synchronized tuples: %d\n"
		 "Possible issues:\n"
		 "\t* stereo_image_proc is not running.\n"
		 "\t  Does `rosnode info %s` show any connections?\n"
		 "\t* The cameras are not synchronized.\n"
		 "\t  Try restarting the node with parameter _approximate_sync:=True\n"
		 "\t* The network is too slow. One or more images are dropped from each tuple.\n"
		 "\t  Try restarting the node, increasing parameter 'queue_size' (currently %d)",
		 left_received_, left_sub_.getTopic().c_str(),
		 right_received_, right_sub_.getTopic().c_str(),
		 left_info_received_, left_info_sub_.getTopic().c_str(),
		 right_info_received_, right_info_sub_.getTopic().c_str(),
		 all_received_, ros::this_node::getName().c_str(), queue_size_);
      }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& l_image_msg,
		       const sensor_msgs::ImageConstPtr& r_image_msg,
		       const sensor_msgs::CameraInfoConstPtr& l_info_msg,
		       const sensor_msgs::CameraInfoConstPtr& r_info_msg) {

      bool found = bonds.empty();	

      if (l_image_msg->header.stamp < last_update_time) {
	return;
      } else {
	last_update_time = l_image_msg->header.stamp;
      }
        
      if (measure_time) {
	ros::Duration comm_time = ros::Time::now() - r_info_msg->header.stamp;
	time_file_comm << std::fixed << std::setprecision(4) << r_info_msg->header.stamp.toSec() << " tcomm_{cam_buffer} = " << std::fixed << std::setprecision(6) << comm_time.toSec() << std::endl;
      }

      ROS_DEBUG("Received image seq %d", l_image_msg->header.seq);
  
      while (!found && !bonds.empty()) { 
	bond::Bond *next_bond = bonds.front();
	bonds.pop();

	if (check_if_broken && next_bond->isBroken()) { 
	  topics.erase(next_bond->getId());	
	  left_cams.erase(next_bond->getId());
	  right_cams.erase(next_bond->getId());
	  ROS_ERROR_STREAM("Bond " << next_bond->getId() << " broken");
	  delete next_bond;				

	} else {

	  bonds.push(next_bond);
	  found = true;
	  if (measure_time) {
	    ros::Duration buffer_proc_time = ros::Time::now() - r_info_msg->header.stamp;
	    time_file_proc << std::fixed << std::setprecision(4) << r_info_msg->header.stamp.toSec() << " tproc_{buffer} = " << std::fixed << std::setprecision(6) << buffer_proc_time.toSec() << std::endl;
	  }

	  left_cams[next_bond->getId()].publish(l_image_msg, l_info_msg);
	  right_cams[next_bond->getId()].publish(r_image_msg, r_info_msg);
	}  
      }	
    }
  
    void newProcNodeCallback (const cloud_3dpc_extractor_msgs::NewProcNodeConstPtr& rec_msg) {
	
      ros::NodeHandle nh;
      image_transport::ImageTransport it(nh);
	
      if (topics.count(rec_msg->id) <= 0) { 
	ROS_INFO_STREAM("New Node found " << rec_msg->id);
	bond::Bond *bond = new bond::Bond(rec_msg->topic, rec_msg->id);

	bond->setHeartbeatPeriod(heartbeat_period);
	bond->setHeartbeatTimeout(heartbeat_timeout);
	bond->start();

	std::string left_cam_topic = rec_msg->id + "/" + "left" + "/" + "image_raw";
	std::string right_cam_topic = rec_msg->id + "/" + "right" + "/" + "image_raw";

	left_cams[bond->getId()] = it.advertiseCamera(left_cam_topic, 1);
	right_cams[bond->getId()] = it.advertiseCamera(right_cam_topic, 1);
	topics[bond->getId()] = rec_msg->topic;
	bonds.push(bond);

	ROS_INFO_STREAM("Cameras advertised " << left_cams[bond->getId()].getTopic() << " " << right_cams[bond->getId()].getTopic());
      }
    }

    StereoCamBuffer(const std::string& transport) :
      replace_(false) {
    
      // Read local parameters

      ros::NodeHandle local_nh("~");
      std::string save_time_file_name_proc;
      std::string save_time_file_name_comm;

      // Resolve topic names

      ros::NodeHandle nh;
      ros::NodeHandle nh2;

      last_update_time = ros::Time::now();
      int receive_queue_size_;

      bool use_udp;

      std::string stereo_ns = nh.resolveName("stereo");
      std::string left_topic = ros::names::clean(stereo_ns + "/left/" + nh.resolveName("image_raw"));
      std::string right_topic = ros::names::clean(stereo_ns + "/right/" + nh.resolveName("image_raw"));

      std::string left_info_topic = stereo_ns + "/left/camera_info";
      std::string right_info_topic = stereo_ns + "/right/camera_info";
      
      local_nh.param<std::string>("output_filename_proc", save_time_file_name_proc, "time_buffer_proc.csv");
      local_nh.param<std::string>("output_filename_comm", save_time_file_name_comm, "time_buffer_comm.csv");
      local_nh.param<bool>("measure_time", measure_time, true);
      local_nh.param<bool>("use_udp", use_udp, false);
      local_nh.param<bool>("check_if_broken", check_if_broken, true);
      local_nh.param<double>("heartbeat_period", heartbeat_period, 5.0);
      local_nh.param<double>("heartbeat_timeout", heartbeat_timeout, 10.0);

      if (measure_time) {
        time_file_proc.open(save_time_file_name_proc.c_str(), std::fstream::out | std::fstream::app);
        time_file_comm.open(save_time_file_name_comm.c_str(), std::fstream::out | std::fstream::app);
        time_file_proc.precision(std::numeric_limits< double >::digits10);
        time_file_comm.precision(std::numeric_limits< double >::digits10);
      }

      // Synchronize input topics. Optionally do approximate synchronization.
      local_nh.param("queue_size", queue_size_, 1);
      local_nh.param("receive_queue_size", receive_queue_size_, 1);

      bool approx;
      local_nh.param("approximate_sync", approx, false);

      image_transport::ImageTransport it(nh);


      if (use_udp) {
	image_transport::TransportHints hints(transport, ros::TransportHints().udp());
	left_sub_.subscribe(it, left_topic, receive_queue_size_, hints);
	right_sub_.subscribe(it, right_topic, receive_queue_size_, hints);
	left_info_sub_.subscribe(nh, left_info_topic, receive_queue_size_, ros::TransportHints().udp());
	right_info_sub_.subscribe(nh, right_info_topic, receive_queue_size_, ros::TransportHints().udp());
      } else {
	left_sub_.subscribe(it, left_topic, 1, transport);
	right_sub_.subscribe(it, right_topic, 1, transport);
	left_info_sub_.subscribe(nh, left_info_topic, 1);
	right_info_sub_.subscribe(nh, right_info_topic, 1);
      }

      // Complain every 15s if the topics appear unsynchronized
      left_sub_.registerCallback(boost::bind(StereoCamBuffer::increment, &left_received_));
      right_sub_.registerCallback(boost::bind(StereoCamBuffer::increment, &right_received_));
      left_info_sub_.registerCallback(boost::bind(StereoCamBuffer::increment, &left_info_received_));
      right_info_sub_.registerCallback(boost::bind(StereoCamBuffer::increment, &right_info_received_));
      check_synced_timer_ = nh.createWallTimer(ros::WallDuration(15.0),
					       boost::bind(&StereoCamBuffer::checkInputsSynchronized, this));

      left_info_received_ = 0;
      right_info_received_ = 0;
      left_received_ = 0;
      right_received_ = 0;
      all_received_ = 0;

      if (approx)
	{
	  approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size_),
						      left_sub_, right_sub_, left_info_sub_, right_info_sub_) );
	  approximate_sync_->registerCallback(boost::bind(&StereoCamBuffer::dataCb, this, _1, _2, _3, _4));
	}
      else
	{
	  exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_),
					  left_sub_, right_sub_, left_info_sub_, right_info_sub_) );
	  exact_sync_->registerCallback(boost::bind(&StereoCamBuffer::dataCb, this, _1, _2, _3, _4));
	}



      ROS_INFO_STREAM(std::endl << std::boolalpha <<
		      "============================================================" << std::endl <<
		      "Started Stereo Cam Buffer node with the following parameters" << std::endl <<
		      "============================================================" << std::endl <<
		      "Check if bond broken    : " << check_if_broken                << std::endl <<
		      "Left Camera             : " << left_topic                     << std::endl <<
		      "Left Camera info        : " << left_info_topic                << std::endl <<
		      "Right Camera            : " << right_topic                    << std::endl <<
		      "Right Camera info       : " << right_info_topic               << std::endl <<
		      "Queue size              : " << queue_size_                    << std::endl <<
		      "Receive Queue size      : " << receive_queue_size_            << std::endl <<
		      "Measure Time            : " << measure_time                   << std::endl <<
		      "Output Filename (Proc)  : " << save_time_file_name_proc       << std::endl <<
		      "Output Filename (Comm)  : " << save_time_file_name_comm       << std::endl <<
		      "Use UDP                 : " << use_udp                        << std::endl <<
		      "Transport               : " << transport                      << std::endl <<
		      "Approximate Sync        : " << approx                         << std::endl <<
		      "Heartbeat Period (s)    : " << heartbeat_period               << std::endl <<
		      "Heartbeat Timeout (s)   : " << heartbeat_timeout              << std::endl <<
		      "==========================================================="  << std::endl);
    }

  };
}

int main(int argc, char** argv) {
    
  ros::init(argc, argv, "stereo_cam_buffer");
  ros::NodeHandle nh;
  ros::NodeHandle nh2("~");
  ros::NodeHandle nh3;

  std::string transport;

  nh2.param<std::string>("transport", transport, "raw");
  cloud_3DPC_extractor::StereoCamBuffer buffer(transport);
  nh3.setCallbackQueue(&buffer.my_callback_queue);
  ros::AsyncSpinner spinner(1, &buffer.my_callback_queue);
  buffer.nodesSub = nh3.subscribe("new_proc_node", 64,
                                  &cloud_3DPC_extractor::StereoCamBuffer::newProcNodeCallback,
                                  &buffer);
  spinner.start();
  ros::spin();  
}
