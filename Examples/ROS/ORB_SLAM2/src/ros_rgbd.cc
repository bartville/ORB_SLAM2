/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "tf/transform_broadcaster.h"

using namespace std;

std::string tf_frame, tf_child_frame;
Eigen::Matrix4f Camera_T(Eigen::Matrix4f::Identity());

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD [params] " << endl << endl;
  
    ros::NodeHandle nh("~");


    std::string vocabulary, settings, rgb_topic, depth_topic;
    bool use_gui;
    nh.param("vocabulary", vocabulary, std::string(""));
    nh.param("settings", settings, std::string(""));
    nh.param("rgb_topic", rgb_topic, std::string("/camera/rgb/image_raw"));
    nh.param("depth_topic", depth_topic, std::string("/camera/depth/image_raw"));
    nh.param("tf_frame", tf_frame, std::string("/camera"));
    nh.param("tf_child_frame", tf_child_frame, std::string("/base_link"));
    nh.param("use_gui", use_gui, bool(false));
    
    std::cerr<<"_vocabulary:=       "<<vocabulary<<std::endl;
    std::cerr<<"_settings:=         "<<settings<<std::endl;
    std::cerr<<"_rgb_topic:=        "<<rgb_topic<<std::endl;
    std::cerr<<"_depth_topic:=      "<<depth_topic<<std::endl;
    std::cerr<<"_tf_frame:=         "<<tf_frame<<std::endl;
    std::cerr<<"_tf_child_frame:=   "<<tf_child_frame<<std::endl;
    std::cerr<<"_use_gui:=          "<<use_gui<<std::endl;

    ORB_SLAM2::System SLAM(vocabulary, settings ,ORB_SLAM2::System::RGBD,use_gui);
    ImageGrabber igb(&SLAM);
    
    
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, rgb_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat curr_T =  mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    Eigen::Matrix4f Tr, Tr_inverse;
    for(size_t i=0; i<4; ++i)
      for(size_t j=0; j<4; ++j)
	Tr(i,j) = curr_T.at<float>(i,j);

    Tr_inverse = Tr.inverse();

    Eigen::Matrix3f rot = Tr_inverse.block<3,3>(0,0);
    
    static tf::TransformBroadcaster tf_broadcaster;
    tf::Transform tf_content;
    tf::Vector3 tf_translation(Tr_inverse(0,3), Tr_inverse(1,3), Tr_inverse(2,3));
    Eigen::Quaternionf qi(rot);
    tf::Quaternion tf_quaternion(qi.x(), qi.y(), qi.z(), qi.w());
    tf_content.setOrigin(tf_translation);
    tf_content.setRotation(tf_quaternion);

    tf::StampedTransform tf_msg(tf_content, cv_ptrRGB->header.stamp, tf_frame, tf_child_frame);
    tf_broadcaster.sendTransform(tf_msg);

}


