#ifndef STEREO_CAM_BUFFER_H
#define STEREO_CAM_BUFFER_H

/*********************************************************************************
 *
 *  Cloud-based 3DPC Extraction platform Front-end
 *
 *  @author Javier J. Salmeron-Garcia (jsalmeron2@us.es)
 *
 *  University of Seville
 *
 *  This file is part of Cloud-based 3DPC Extraction platform
 *
 *  Cloud-based 3DPC Extraction platform is free software:
 *  you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Cloud-based 3DPC Extraction platform is distributed in the hope
 *  that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Cloud-based 3DPC Extraction platform.  If not, see <http://www.gnu.org/licenses/>.
 *
 * *****************************************************************************/


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
#include <cloud_3dpc_extractor_msgs/New3DPCExtractor.h>
#include <fstream>
#include <queue>
#include <limits>

namespace cloud_3DPC_extractor {

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

  class StereoCamBuffer {

  private:

    /**
     * @brief model Camera model
     */
    image_geometry::StereoCameraModel model;

    /**
     * @brief heartbeat_period bond heartbeat information
     */
    double heartbeat_period;
    double heartbeat_timeout;

    /**
     * @brief bonds Bond information with the 3DPC Extractor nodes
     */
    std::queue<bond::Bond*> bonds;

    /**
     * @brief check_if_broken Check if the bond is broken
     */
    bool check_if_broken;

    /**
     * @brief new_nodes_sub Subscriber and callback queue for new nodes
     */
    ros::Subscriber new_nodes_sub;
    ros::CallbackQueue new_nodes_callback_queue;

    /**
     * @brief topics Maps 3DPC Extractor node with a topic id
     */
    std::map< std::string, std::string > topics;

    /**
     * @brief Maps 3DPC Extractor Topic ID to a Camera Publisher
     */
    std::map< std::string, image_transport::CameraPublisher > left_cams;
    std::map< std::string, image_transport::CameraPublisher > right_cams;

    /**
     * @brief spinner Stereo pair reception spinner
     */
    ros::AsyncSpinner *spinner;

    /**
     * @brief last_update_time Timestamp from the last received stereo pair
     */
    ros::Time last_update_time;

    /**
     * @brief camera_sub_ Subscription to stereo camera's image and info topic
     */
    image_transport::SubscriberFilter left_sub_, right_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_, right_info_sub_;

    /**
     * @brief exact_sync_ Synchronization of the stereo pairs
     */
    boost::shared_ptr<ExactSync> exact_sync_;
    boost::shared_ptr<ApproximateSync> approximate_sync_;
    ros::WallTimer check_synced_timer_;
    int left_received_, right_received_, left_info_received_, right_info_received_, all_received_;

    /**
     * @brief queue_size_ Size of the queue
     */
    int queue_size_;

    /*
     * Profiling
     */

    /**
     * @brief measure_time Flag for profiling
     */
    bool measure_time;

    /**
     * @brief time_file_proc Files to save the profiling information
     */
    std::ofstream time_file_proc;
    std::ofstream time_file_comm;

    static void increment(int* value)
    {
      ++(*value);
    }

    /**
     * @brief forward_stereo_frame Forwards a new stereo frame to 3DPC Extractors
     * @param l_image_msg
     * @param r_image_msg
     * @param l_info_msg
     * @param r_info_msg
     */
    void forward_stereo_frame(const sensor_msgs::ImageConstPtr& l_image_msg,
		const sensor_msgs::ImageConstPtr& r_image_msg,
		const sensor_msgs::CameraInfoConstPtr& l_info_msg,
		const sensor_msgs::CameraInfoConstPtr& r_info_msg);

    /**
     * @brief checkInputsSynchronized Checks a stereo pair is complete
     */
    void checkInputsSynchronized();

    /**
     * @brief process_new_stereo_pair Callback of a new stereo frame
     * @param l_image_msg
     * @param r_image_msg
     * @param l_info_msg
     * @param r_info_msg
     */
    void process_new_stereo_pair(const sensor_msgs::ImageConstPtr& l_image_msg,
		       const sensor_msgs::ImageConstPtr& r_image_msg,
		       const sensor_msgs::CameraInfoConstPtr& l_info_msg,
		       const sensor_msgs::CameraInfoConstPtr& r_info_msg);

    /**
     * @brief New3DPCExtractorCallback Callback of a new 3DPC Extractor
     * @param rec_msg
     */
    void New3DPCExtractorCallback (const cloud_3dpc_extractor_msgs::New3DPCExtractorConstPtr& rec_msg);

  public:

    StereoCamBuffer();

    /**
     * @brief start Start buffering and subscriptions
     */
    void start();

  };
};

#endif // STEREO_CAM_BUFFER_H
