#include<ros/ros.h>
#include <landing/trackpose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>

#include<iostream>

using namespace std;


mavros_msgs::State current_state;
int land_mode = 0;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void joyCallBack( const std_msgs::String::ConstPtr& str){ 
	// call back
	if(str->data == "d"||str->data=="D"){
		land_mode = 1;
		}
	std::cout<<"str.data : "<< str->data <<std::endl;
	}

geometry_msgs::PoseStamped local_position;
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
}

landing::trackpose landmark;
void pose_cb(const landing::trackpose::ConstPtr& msg){
    landmark = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landing_pose_node");
    ros::NodeHandle nh;
    ROS_INFO("start landing node");

    double kp_x,kp_y;
    std::string pose_topic;

    nh.param("pose_pub_topic",pose_topic,std::string("/mav_local_position"));
    nh.param("kp_x",kp_x,5.0);
    nh.param("kp_y",kp_y,5.0);

    cout<<"param kp_x:"<<kp_x<<endl;
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, position_cb);
    ros::Subscriber center_sub = nh.subscribe(pose_topic,1,pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	
	ros::Subscriber joy_sub = nh.subscribe("/keys",10,&joyCallBack);	// subscribe


 
    ros::Rate rate(20.0);
    int land_cnt = 0;
 
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
 
    geometry_msgs::PoseStamped pose;//姿态控制
    pose.pose.position.x = local_position.pose.position.x;
	pose.pose.position.y = local_position.pose.position.y;
	pose.pose.position.z = 2;
    
    geometry_msgs::TwistStamped vel;//速度控制
 
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
 
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
 
    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
            //起飞
        while(ros::ok())
        {
            if(land_mode == 1)
            {
                ROS_INFO("keyboard interrupt");
                break;
            }
            
            if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
            else if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
            else if(fabs( local_position.pose.position.z - 2 )<0.1)
            {
                ROS_INFO("tracking landing start");
                break;
            }
            ROS_INFO("local_pos: x %f, z %f",local_position.pose.position.x,local_position.pose.position.z);
            local_pos_pub.publish(pose);
    
            ros::spinOnce();
            rate.sleep();
        }
        
        //控制降落部分
        while(ros::ok())
        {
            if(land_mode == 1)
            {
                ROS_INFO("keyboard interrupt");
                break;
            }
            //高度低于0.3时转为降落模式
            if(local_position.pose.position.z < 0.3&&landmark.pose.position.z<0.18)
                break;
            //如果找到地标，控制方向
            if(landmark.iffind)
            {
                ROS_INFO("find mark");
                //计算两方向err
                double err_x = -landmark.pose.position.y;
                double err_y = -landmark.pose.position.x;
                ROS_INFO_STREAM("state="<<err_x<<" "<<err_y);
                //速度控制
                vel.twist.linear.x = err_x/kp_x;
                vel.twist.linear.y = err_y/kp_y;
                //如果位置很正开始降落
                if(fabs(err_x) < 0.1 &&fabs(err_y) < 0.1)
                {
                    if(local_position.pose.position.z < 0.3)
                    {
                        if(fabs(err_x) < 0.05 &&fabs(err_y) < 0.05)
                        {
                                ROS_INFO("good 2 decend stable");
                                vel.twist.linear.z = -0.2;
                        }
                        else
                            vel.twist.linear.z = 0;
                    }
                    else{
                            land_cnt=0;
                            ROS_INFO("good 2 decend");
                            vel.twist.linear.z = -0.2;
                        }
                }	
                else
                    vel.twist.linear.z = 0;

                local_vel_pub.publish(vel);
                ros::spinOnce();
                rate.sleep();
            }
            //如果找不到矩形地标，回到2m高度
            else
            {
                ROS_INFO("lose mark: back up");
                pose.pose.position.x = local_position.pose.position.x;
                pose.pose.position.y = local_position.pose.position.y;
                pose.pose.position.z = 2;
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
            }
        }
        
        offb_set_mode.request.custom_mode = "AUTO.LAND";
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
            ROS_INFO("land enabled");
            last_request = ros::Time::now();
        }
        last_request = ros::Time::now();
        ros::Duration(10.0).sleep();
    }

 
    return 0;
}
