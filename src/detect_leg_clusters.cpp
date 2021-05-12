/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
// ROS 
#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/imgproc/types_c.h>

// Local headers
#include <leg_tracker/laser_processor.h>
#include <leg_tracker/cluster_features.h>

// Custom messages
#include <leg_tracker/Leg.h>
#include <leg_tracker/LegArray.h>

#include <cstdlib>
#include <spencer_tracking_msgs/DetectedPerson.h>
#include <spencer_tracking_msgs/DetectedPersons.h>


/**
* @brief Detects clusters in laser scan with leg-like shapes
*/
class DetectLegClusters
{
public:
  /**
  * @brief Constructor
  */
  DetectLegClusters(ros::NodeHandle &n):
  scan_num_(0),
  num_prev_markers_published_(0),
  nh_(n)
  {  
    // Get ROS parameters  
    std::string forest_file;
    std::string scan_topic;
    if (!nh_.getParam("forest_file", forest_file))
      ROS_ERROR("ERROR! Could not get random forest filename");
    nh_.param("scan_topic", scan_topic, std::string("/scan"));
    nh_.param("fixed_frame", fixed_frame_, std::string("odom"));
    nh_.param("detection_threshold", detection_threshold_, -1.0);
    nh_.param("cluster_dist_euclid", cluster_dist_euclid_, 0.13);
    nh_.param("min_points_per_cluster", min_points_per_cluster_, 3);                
    nh_.param("max_detect_distance", max_detect_distance_, 10.0);   
    nh_.param("marker_display_lifetime", marker_display_lifetime_, 0.2);   
    nh_.param("use_scan_header_stamp_for_tfs", use_scan_header_stamp_for_tfs_, false);
    nh_.param("max_detected_clusters", max_detected_clusters_, -1);

    // Print back
    ROS_INFO("forest_file: %s", forest_file.c_str());
    ROS_INFO("scan_topic: %s", scan_topic.c_str());
    ROS_INFO("fixed_frame: %s", fixed_frame_.c_str());
    ROS_INFO("detection_threshold: %.2f", detection_threshold_);
    ROS_INFO("cluster_dist_euclid: %.2f", cluster_dist_euclid_);
    ROS_INFO("min_points_per_cluster: %d", min_points_per_cluster_);
    ROS_INFO("max_detect_distance: %.2f", max_detect_distance_);    
    ROS_INFO("marker_display_lifetime: %.2f", marker_display_lifetime_);
    ROS_INFO("use_scan_header_stamp_for_tfs: %d", use_scan_header_stamp_for_tfs_);    
    ROS_INFO("max_detected_clusters: %d", max_detected_clusters_);    

    // Load random forst
    forest = cv::ml::StatModel::load<cv::ml::RTrees>(forest_file);
    feat_count_ = forest->getVarCount();

    latest_scan_header_stamp_with_tf_available_ = ros::Time::now();

    // ROS subscribers + publishers
    scan_sub_ =  nh_.subscribe(scan_topic, 10, &DetectLegClusters::laserCallback, this);
    markers_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 20);
    detected_leg_clusters_pub_ = nh_.advertise<leg_tracker::LegArray>("/detected_leg_clusters", 20);

    m_detectedPersonsPublisher = nh_.advertise<spencer_tracking_msgs::DetectedPersons>("/spencer/perception_internal/detected_persons/laser_front", 1);
    m_lastDetectionId = 0;
    m_detectionIdIncrement = 1;
    m_poseVariance = 0.1;
  }

private:
  tf::TransformListener tfl_;

  cv::Ptr< cv::ml::RTrees > forest = cv::ml::RTrees::create();

  int feat_count_;

  ClusterFeatures cf_;

  int scan_num_;
  bool use_scan_header_stamp_for_tfs_;
  ros::Time latest_scan_header_stamp_with_tf_available_;

  ros::NodeHandle nh_;
  ros::Publisher markers_pub_;
  ros::Publisher detected_leg_clusters_pub_;
  ros::Publisher m_detectedPersonsPublisher;
  ros::Subscriber scan_sub_;

  std::string fixed_frame_;
  
  double detection_threshold_;
  double cluster_dist_euclid_;
  int min_points_per_cluster_;  
  double max_detect_distance_;
  double marker_display_lifetime_;
  int max_detected_clusters_;

  int num_prev_markers_published_;

  int m_lastDetectionId,m_detectionIdIncrement;
  double m_poseVariance;

  /**
  * @brief Clusters the scan according to euclidian distance, 
  *        predicts the confidence that each cluster is a human leg and publishes the results
  * 
  * Called every time a laser scan is published.
  */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {         
    laser_processor::ScanProcessor processor(*scan); 
    processor.splitConnected(cluster_dist_euclid_);        
    processor.removeLessThan(min_points_per_cluster_);    

    // OpenCV matrix needed to use the OpenCV random forest classifier
    CvMat* tmp_mat = cvCreateMat(1, feat_count_, CV_32FC1); 
    
    leg_tracker::LegArray detected_leg_clusters;
    detected_leg_clusters.header.frame_id = scan->header.frame_id;
    detected_leg_clusters.header.stamp = scan->header.stamp;

    // Find out the time that should be used for tfs
    bool transform_available;
    ros::Time tf_time;
    // Use time from scan header
    if (use_scan_header_stamp_for_tfs_)
    {
      tf_time = scan->header.stamp;

      try
      {
        tfl_.waitForTransform(fixed_frame_, scan->header.frame_id, tf_time, ros::Duration(1.0));
        transform_available = tfl_.canTransform(fixed_frame_, scan->header.frame_id, tf_time);
      }
      catch(tf::TransformException ex)
      {
        ROS_INFO("Detect_leg_clusters: No tf available");
        transform_available = false;
      }
    }
    else
    {
      // Otherwise just use the latest tf available
      tf_time = ros::Time(0);
      transform_available = tfl_.canTransform(fixed_frame_, scan->header.frame_id, tf_time);
    }
    
    // Store all processes legs in a set ordered according to their relative distance to the laser scanner
    std::set <leg_tracker::Leg, CompareLegs> leg_set;
    std::vector<leg_tracker::Leg> new_leg_array;
    if (!transform_available)
    {
      ROS_INFO("Not publishing detected leg clusters because no tf was available");
    }
    else // transform_available
    {
      int num_clusters = 0;
      // Iterate through all clusters
      for (std::list<laser_processor::SampleSet*>::iterator cluster = processor.getClusters().begin();
       cluster != processor.getClusters().end();
       cluster++)
      {   
        // Get position of cluster in laser frame
        tf::Stamped<tf::Point> position((*cluster)->getPosition(), tf_time, scan->header.frame_id);
        float rel_dist = pow(position[0]*position[0] + position[1]*position[1], 1./2.);
        
        // Only consider clusters within max_distance.
        if (rel_dist < max_detect_distance_)
        {
          // Classify cluster using random forest classifier
          std::vector<float> f = cf_.calcClusterFeatures(*cluster, *scan);
          for (int k = 0; k < feat_count_; k++)
            tmp_mat->data.fl[k] = (float)(f[k]);

          
          #if (CV_VERSION_MAJOR <= 3 && CV_VERSION_MINOR <= 2)
            // Output of forest->predict is [-1.0, 1.0] so we scale to reach [0.0, 1.0]
            float probability_of_leg = 0.5 * (1.0 + forest->predict(cv::cvarrToMat(tmp_mat)));
          #else
            // The forest->predict funciton has been removed in the latest versions of OpenCV so we'll do the calculation explicitly.
            cv::Mat result;
            forest->getVotes(cv::cvarrToMat(tmp_mat), result, 0);
            int positive_votes = result.at<int>(1, 1);
            int negative_votes = result.at<int>(1, 0);
            float probability_of_leg = positive_votes / static_cast<double>(positive_votes + negative_votes);
          #endif

          // Consider only clusters that have a confidence greater than detection_threshold_                 
          if (probability_of_leg > detection_threshold_)
          { 
            // Transform cluster position to fixed frame
            // This should always be succesful because we've checked earlier if a tf was available
            bool transform_successful_2;
            try
            {
              tfl_.transformPoint(fixed_frame_, position, position);
              transform_successful_2 = true;
            }
            catch (tf::TransformException ex)
            {
              ROS_ERROR("%s",ex.what());
              transform_successful_2 = false;
            }

            if (transform_successful_2)
            {  
              // Add detected cluster to set of detected leg clusters, along with its relative position to the laser scanner
              leg_tracker::Leg new_leg;
              new_leg.position.x = position[0];
              new_leg.position.y = position[1];
              new_leg.confidence = probability_of_leg;
              new_leg.paired = false;
              new_leg.indices = processor.laser_indice_cluster[num_clusters];
              new_leg.first_indice = new_leg.indices[0];
              new_leg.last_indice = new_leg.indices[new_leg.indices.size()-1];
              new_leg_array.push_back(new_leg);
              leg_set.insert(new_leg);
            }
          }
        }

        num_clusters++;
      } 
    }    
 

    // Publish detected legs to /detected_leg_clusters and to rviz
    // They are ordered from closest to the laser scanner to furthest  
    int clusters_published_counter = 0;
    int id_num = 1;      
    for (std::set<leg_tracker::Leg>::iterator it = leg_set.begin(); it != leg_set.end(); ++it)
    {
      // Publish to /detected_leg_clusters topic
      leg_tracker::Leg leg = *it;
      detected_leg_clusters.legs.push_back(leg);
      clusters_published_counter++;

      // Publish marker to rviz
      visualization_msgs::Marker m;
      m.header.stamp = scan->header.stamp;
      m.header.frame_id = fixed_frame_;
      m.ns = "LEGS";
      m.id = id_num++;
      m.type = m.CUBE;
      m.pose.position.x = leg.position.x ;
      m.pose.position.y = leg.position.y;
      m.pose.position.z = 0.2;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.13;
      m.scale.y = 0.13;
      m.scale.z = 0.13;
      m.color.a = 1;
      m.color.r = 0;
      m.color.g = leg.confidence;
      m.color.b = leg.confidence;
      // m.lifetime = ros::Duration(0.05);
      markers_pub_.publish(m);

      // Comparison using '==' and not '>=' is important, as it allows <max_detected_clusters_>=-1 
      // to publish infinite markers
      if (clusters_published_counter == max_detected_clusters_) 
        break;
    }

    float distance_current_leg=0.,distance_next_leg=0.,distance_next_next_leg=0.0;
    spencer_tracking_msgs::DetectedPersons detectedPersons;
    detectedPersons.header = scan->header;
    bool found_person;
    std::vector<uint32_t> person_laser_indices;

    if(new_leg_array.size()>1){
      // for(int no_leg=0; no_leg<new_leg_array.size();no_leg++){
      //   printf("#%d || x: %f y: %f paired: %d\n",no_leg,new_leg_array[no_leg].position.x,new_leg_array[no_leg].position.y,new_leg_array[no_leg].paired);
      // }
      // printf("----------------\n");
      // for(int no_leg=0; no_leg<new_leg_array.size()-2;no_leg++){
        // if(new_leg_array[no_leg].paired!=true){

        //   distance_current_leg = pow(new_leg_array[no_leg].position.x*new_leg_array[no_leg].position.x + new_leg_array[no_leg].position.y*new_leg_array[no_leg].position.y, 1./2.); 
        //   distance_next_leg = pow(new_leg_array[no_leg+1].position.x*new_leg_array[no_leg+1].position.x + new_leg_array[no_leg+1].position.y*new_leg_array[no_leg+1].position.y, 1./2.); 
        //   distance_next_next_leg = pow(new_leg_array[no_leg+2].position.x*new_leg_array[no_leg+2].position.x + new_leg_array[no_leg+2].position.y*new_leg_array[no_leg+2].position.y, 1./2.); 
        //   found_person = false;

        //   visualization_msgs::Marker m;

        //   // && std::abs(int(new_leg_array[no_leg].last_indice-new_leg_array[no_leg+1].first_indice))<=10
        //   if(std::abs(distance_current_leg-distance_next_leg)<0.5){
        //     printf("Found pair of leg!!!!\n");
        //     found_person=true;
        //     new_leg_array[no_leg].paired=true;
        //     new_leg_array[no_leg+1].paired=true;

        //     // visualization_msgs::Marker m;
        //     m.header.stamp = scan->header.stamp;
        //     m.header.frame_id = fixed_frame_;
        //     m.ns = "LEGS";
        //     m.id = id_num++;
        //     m.type = m.CYLINDER;
        //     m.pose.position.x = (new_leg_array[no_leg].position.x+new_leg_array[no_leg+1].position.x)/2 ;
        //     m.pose.position.y = (new_leg_array[no_leg].position.y+new_leg_array[no_leg+1].position.y)/2;
        //     m.pose.position.z = 0.8;
        //     m.pose.orientation.w = 1.0;
        //     m.scale.x = 0.3;
        //     m.scale.y = 0.3;
        //     m.scale.z = 0.3;
        //     m.color.a = 1;
        //     m.color.r = 0;
        //     m.color.g = (new_leg_array[no_leg].confidence+new_leg_array[no_leg+1].confidence)/2;
        //     m.color.b = (new_leg_array[no_leg].confidence+new_leg_array[no_leg+1].confidence)/2;
        //     person_laser_indices.insert(person_laser_indices.end(), new_leg_array[no_leg].indices.begin(), new_leg_array[no_leg].indices.end());
        //     person_laser_indices.insert(person_laser_indices.end(), new_leg_array[no_leg+1].indices.begin(), new_leg_array[no_leg+1].indices.end());
        //     markers_pub_.publish(m);
        //   }
        //   else if(std::abs(distance_current_leg-distance_next_next_leg)<0.5){
        //     printf("Found pair of leg!!!!\n");
        //     found_person=true;
        //     new_leg_array[no_leg].paired=true;
        //     new_leg_array[no_leg+2].paired=true;

        //     // visualization_msgs::Marker m;
        //     m.header.stamp = scan->header.stamp;
        //     m.header.frame_id = fixed_frame_;
        //     m.ns = "LEGS";
        //     m.id = id_num++;
        //     m.type = m.CYLINDER;
        //     m.pose.position.x = (new_leg_array[no_leg].position.x+new_leg_array[no_leg+2].position.x)/2 ;
        //     m.pose.position.y = (new_leg_array[no_leg].position.y+new_leg_array[no_leg+2].position.y)/2;
        //     m.pose.position.z = 0.8;
        //     m.pose.orientation.w = 1.0;
        //     m.scale.x = 0.3;
        //     m.scale.y = 0.3;
        //     m.scale.z = 0.3;
        //     m.color.a = 1;
        //     m.color.r = 0;
        //     m.color.g = (new_leg_array[no_leg].confidence+new_leg_array[no_leg+2].confidence)/2;
        //     m.color.b = (new_leg_array[no_leg].confidence+new_leg_array[no_leg+2].confidence)/2;
        //     person_laser_indices.insert(person_laser_indices.end(), new_leg_array[no_leg].indices.begin(), new_leg_array[no_leg].indices.end());
        //     person_laser_indices.insert(person_laser_indices.end(), new_leg_array[no_leg+2].indices.begin(), new_leg_array[no_leg+2].indices.end());
        //     markers_pub_.publish(m);
        //   }
        //   else if(new_leg_array[no_leg].confidence>0.1){
        //     printf("Found leg but not pair!!!!\n");
        //     found_person=true;
        //     printf("(%f,%f) -- (%f,%f) = %f\n",new_leg_array[no_leg].position.x,new_leg_array[no_leg].position.y,new_leg_array[no_leg+1].position.x,new_leg_array[no_leg+1].position.y,std::abs(distance_current_leg-distance_next_leg));
        //     // visualization_msgs::Marker m;
        //     m.header.stamp = scan->header.stamp;
        //     m.header.frame_id = fixed_frame_;
        //     m.ns = "LEGS";
        //     m.id = id_num++;
        //     m.type = m.CYLINDER;
        //     m.pose.position.x = new_leg_array[no_leg].position.x ;
        //     m.pose.position.y = new_leg_array[no_leg].position.y;
        //     m.pose.position.z = 0.8;
        //     m.pose.orientation.w = 1.0;
        //     m.scale.x = 0.3;
        //     m.scale.y = 0.3;
        //     m.scale.z = 0.3;
        //     m.color.a = 1;
        //     m.color.r = 0;
        //     m.color.g = new_leg_array[no_leg].confidence;
        //     m.color.b = new_leg_array[no_leg].confidence;
        //     person_laser_indices.insert(person_laser_indices.end(), new_leg_array[no_leg].indices.begin(), new_leg_array[no_leg].indices.end());
        //     markers_pub_.publish(m);
        //   }

        //   if(found_person){
        //     spencer_tracking_msgs::DetectedPerson detectedPerson;
        //     m_lastDetectionId += m_detectionIdIncrement;
        //     detectedPerson.detection_id = m_lastDetectionId;
        //     detectedPerson.modality = spencer_tracking_msgs::DetectedPerson::MODALITY_GENERIC_LASER_2D;
        //     detectedPerson.confidence = m.color.g;
        //     detectedPerson.pose.pose.position.x = m.pose.position.x;
        //     detectedPerson.pose.pose.position.y = -m.pose.position.y;
        //     detectedPerson.pose.pose.position.z = 0.0;

        //     const double LARGE_VARIANCE = 999999999;
        //     for(size_t d = 0; d < 2; d++) detectedPerson.pose.covariance[d*6 + d] = m_poseVariance;
        //     for(size_t d = 2; d < 6; d++) detectedPerson.pose.covariance[d*6 + d] = LARGE_VARIANCE;

        //     detectedPerson.indices = person_laser_indices;
        //     person_laser_indices.clear();
        //     detectedPersons.detections.push_back(detectedPerson);
        //   }
          for(int no_leg=0; no_leg<new_leg_array.size();no_leg++){
            spencer_tracking_msgs::DetectedPerson detectedPerson;
            m_lastDetectionId += m_detectionIdIncrement;
            detectedPerson.detection_id = m_lastDetectionId;
            detectedPerson.modality = spencer_tracking_msgs::DetectedPerson::MODALITY_GENERIC_LASER_2D;
            detectedPerson.confidence = new_leg_array[no_leg].confidence;
            detectedPerson.pose.pose.position.x = new_leg_array[no_leg].position.x;
            detectedPerson.pose.pose.position.y = -new_leg_array[no_leg].position.y;
            detectedPerson.pose.pose.position.z = 0.0;

            const double LARGE_VARIANCE = 999999999;
            for(size_t d = 0; d < 2; d++) detectedPerson.pose.covariance[d*6 + d] = m_poseVariance;
            for(size_t d = 2; d < 6; d++) detectedPerson.pose.covariance[d*6 + d] = LARGE_VARIANCE;

            detectedPerson.indices = new_leg_array[no_leg].indices;
            detectedPersons.detections.push_back(detectedPerson);
        }
      }
    // }

    for(int no_leg=0; no_leg<new_leg_array.size();no_leg++){
      printf("#%d || x: %f y: %f paired: %d\n",no_leg,new_leg_array[no_leg].position.x,new_leg_array[no_leg].position.y,new_leg_array[no_leg].paired);
    }
    printf("----------------\n");

    // Clear remaining markers in Rviz
    for (int id_num_diff = num_prev_markers_published_-id_num; id_num_diff > 0; id_num_diff--)
    {
      visualization_msgs::Marker m;
      m.header.stamp = scan->header.stamp;
      m.header.frame_id = fixed_frame_;
      m.ns = "LEGS";
      m.id = id_num_diff + id_num;
      m.action = m.DELETE;
      markers_pub_.publish(m);
    }
    num_prev_markers_published_ = id_num; // For the next callback

    m_detectedPersonsPublisher.publish(detectedPersons);
    detected_leg_clusters_pub_.publish(detected_leg_clusters);
    cvReleaseMat(&tmp_mat);
  }


  /**
  * @brief Comparison class to order Legs according to their relative distance to the laser scanner
  */
  class CompareLegs
  {
  public:
      bool operator ()(const leg_tracker::Leg &a, const leg_tracker::Leg &b)
      {
          float rel_dist_a = pow(a.position.x*a.position.x + a.position.y*a.position.y, 1./2.);
          float rel_dist_b = pow(b.position.x*b.position.x + b.position.y*b.position.y, 1./2.);          
          return rel_dist_a < rel_dist_b;
      }
  };

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_leg_clusters");
  ros::NodeHandle n("~");
  DetectLegClusters dlc(n);
  ros::spin();
  return 0;
}

