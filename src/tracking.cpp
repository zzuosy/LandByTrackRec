#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include<tf/transform_datatypes.h> //转换函数头文件
#include <tf/tf.h>

#include<landing/trackpose.h>

#include "cvdrawingutils.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "fractaldetector.h"
#include "aruco_cvversioning.h"
#include <sophus/se3.h>
#include <sophus/so3.h>


using namespace std;
using namespace cv;
using namespace aruco;
using namespace Sophus;
// cv_bridge::CvImagePtr cv_ptr;

class PlatformDetection {
private:

    // nodehandle
    ros::NodeHandle nh_;

    // param flag
    bool para_b_sim;
    bool para_b_plot;
    bool is_detect;
    bool is_cam_info_set_;

    cv::Mat image_, image_display_;

    // to get time stamp of frame
    ros::Time stamp_;
    uint32_t seq_;

    // Topics
    std::string image_topic_,pose_pub_topic_;

    // config file path
    std::string camera_info_path_;

    // ROS PUBLISHERS
    // ros::Publisher platform_position_in_ardrone_pub_, platform_position_in_world_pub_;
    ros::Publisher landmark_position_in_drone_pub_;

    // image Subscriber 
    image_transport::Subscriber image_sub_;

    // real landmark size
    double marker_size_;

// aruco
    aruco::CameraParameters CamParam;
    aruco::FractalDetector FDetector;

    // track pose
    landing::trackpose trackpose_;
        

    // 3d positions
    tf::Vector3 position_in_camera_, position_in_ardrone_, position_in_world_;

    void computeVerticalDistToPlatform();

public:
    PlatformDetection();
    // void camInfoCallback(const sensor_msgs::CameraInfo& cam_info_msg);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    void computeRelativePose();

    void camPoseinLandmarkCoord(  cv::Mat rvec, cv::Mat tvec);

    Vector3d positioncam2drone(Vector3d cam_position);

}; //end of class platform_Detection

PlatformDetection::PlatformDetection() {
    ROS_INFO("Init Class: Platform_Detection");

    ros::NodeHandle nh_("~");
    image_transport::ImageTransport it_(nh_);

    // GETTING ROS PARAMETERS
    nh_.param("is_sim", para_b_sim, true);
    nh_.param("is_plot",para_b_plot,true);
    nh_.param("pose_pub_topic",pose_pub_topic_,std::string("/mav_local_position"));
    nh_.param("marker_size",marker_size_,0.845);
    ROS_INFO("marker_size: %lf",marker_size_);

    if(para_b_sim)
         nh_.getParam("image_topic_raw_sim",image_topic_);
    else 
        nh_.getParam("image_topic_raw_real",image_topic_);
    
    ROS_INFO("image_tpoic: %s",image_topic_.c_str());
    nh_.getParam("camera_info_path", camera_info_path_);
    ROS_INFO("camera_info_path: %s",camera_info_path_.c_str());
    CamParam.readFromXMLFile(camera_info_path_);
    FDetector.setConfiguration("FRACTAL_5L_6");
    if (CamParam.isValid())
        {
            //CamParam.resize(im.size());
            FDetector.setParams(CamParam, marker_size_);
        }else ROS_ERROR("CamParam invalid");

    // --TOPICS -- //
    image_sub_     = it_.subscribe(image_topic_, 1, &PlatformDetection::imageCb, this);
    landmark_position_in_drone_pub_ = nh_.advertise<landing::trackpose>(pose_pub_topic_, 1);


    ROS_INFO("Setup completed");
}

void PlatformDetection::imageCb(const sensor_msgs::ImageConstPtr& msg){

    // get time stamp for frame
    stamp_ = ros::Time::now();
    seq_ = stamp_.toSec();
    cv_bridge::CvImagePtr cv_ptr;

        try{
            //将收到的消息使用cv_bridge转移到全局变量图像指针cv_ptr中，其成员变量image就是Mat型的图片
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            image_ = cv_ptr->image;
            image_display_ = image_.clone();
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //处理图片信息
        computeRelativePose();

        image_ = Mat();
    }

    void PlatformDetection::computeRelativePose(){

        trackpose_.iffind = false;   
        trackpose_.pose = geometry_msgs::Pose();

        if(FDetector.detect(image_)){
                FDetector.drawMarkers(image_);
            }
        //Pose estimation
         if(FDetector.poseEstimation()){
                trackpose_.iffind = true;
                //Calc distance to marker
                cv::Mat tvec = FDetector.getTvec();
                cv::Mat rvec = FDetector.getRvec();
                trackpose_.pose.position.x = tvec.at<double>(0,0);
                trackpose_.pose.position.y = tvec.at<double>(1,0);
                trackpose_.pose.position.z = tvec.at<double>(2,0);

                // camPoseinLandmarkCoord(rvec,tvec);
                ROS_INFO("POSE FIND");
                // cout<<"x:"<<tvec.at<double>(0,0)<<" y: "<<tvec.at<double>(1,0)<<" z: "<<tvec.at<double>(2,0)<<endl;

                geometry_msgs::Quaternion quaternion;//定义四元数
	            quaternion=tf::createQuaternionMsgFromRollPitchYaw(rvec.at<double>(0,0),rvec.at<double>(1,0),rvec.at<double>(2,0)); //欧拉角
                trackpose_.pose.orientation = quaternion;

                FDetector.draw3d(image_); //3d
            }
            else
                FDetector.draw2d(image_); //Ok, show me at least the inner corners!
            landmark_position_in_drone_pub_.publish(trackpose_);

            if(para_b_plot)
            {
                cv::imshow("image",image_);
                cv::waitKey(1);
            }
        
    }

    void PlatformDetection::camPoseinLandmarkCoord(cv::Mat rvec, cv::Mat tvec)
    {
        // cout<<"S:"<<s<<endl;
                cv::Mat rotM;
                cv::Rodrigues(rvec,rotM);
                Eigen::Matrix3d Rvec;
                Rvec<<rotM.at<double>(0,0), rotM.at<double>(0,1), rotM.at<double>(0,2),
                        rotM.at<double>(1,0), rotM.at<double>(1,1), rotM.at<double>(1,2),
                        rotM.at<double>(2,0), rotM.at<double>(2,1), rotM.at<double>(2,2);

                Sophus::SO3 SO3_R(Rvec);

                SE3 T_l_c_estimated_ = SE3(
                        SO3_R,
                        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
                );
		        // Vector6d se3 = T_l_c_estimated_.log(); // 使用 "对数映射" 获得它的李代数se3
		        // cout<<"se3 = "<<se3.transpose()<<endl;
                SE3 T_c_l = T_l_c_estimated_.inverse();
                //cout<<"T_l_c:\n"<<T_l_c_estimated_.matrix()<<endl;
                // cout<<"T_c_l:\n"<<T_c_l.matrix()<<endl;
               //  cout<<"trans in cam coord:\n"<<T_c_l.translation()<<endl;
                Vector3d tcl = T_c_l.translation();
                Vector3d cam_position,drone_position;
                double yaw = rvec.at<double>(1,0);
                Eigen::Matrix3d Rl2c;
                Rl2c<< cos(yaw),sin(yaw),0,
                            -sin(yaw),cos(yaw),0,
                            0,0,1;
                cam_position = Rl2c * tcl;
                drone_position = positioncam2drone(cam_position);
                cout<<"tvec x:"<<tvec.at<double>(0,0)<<" y: "<<tvec.at<double>(1,0)<<" z: "<<tvec.at<double>(2,0)<<endl;
                cout<<"tcl x:"<<tcl(0,0)<<" y: "<<tcl(1,0)<<" z: "<<tcl(2,0)<<endl;
                cout<<"cam_pose x:"<<drone_position(0,0)<<" y: "<<drone_position(1,0)<<" z: "<<drone_position(2,0)<<endl;
                trackpose_.pose.position.x =drone_position(0,0);
                trackpose_.pose.position.y = drone_position(1,0);
                trackpose_.pose.position.z = -drone_position(2,0); // 高度改成正的比较方便控制使用
    }

    Vector3d PlatformDetection::positioncam2drone(Vector3d cam_position)
    {
        // rotation mat drone2cam is
        //  [0 -1 0;-1 0 0; 0 0 -1 ] >>>> Pd.x = -Pc.y; Pd.y = -Pc.x
        return Vector3d(-cam_position(1,0), -cam_position(0,0), -cam_position(2,0));
    }

int main(int argc, char *argv[])
{
    /* code for main function */
     ros::init(argc, argv, "tracking_node");
    ROS_INFO("start tracking node");
    PlatformDetection pd;    
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = 2;

    ros::spin();
   
    return 0;
}