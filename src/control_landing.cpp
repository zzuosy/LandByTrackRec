#include<iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <landing/center.h> // 导入自定义消息类型
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>



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

landing::center landmark;
void center_cb(const landing::center::ConstPtr& msg){
    landmark = *msg;
}

double  tfmini_height;
void tfmini_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    sensor_msgs::Range tfmini_range = *msg;
	tfmini_height = tfmini_range.range;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_landing_node");
    ros::NodeHandle nh;
    ROS_INFO("start landing node");
 	
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, position_cb);
    ros::Subscriber tf_distance_sub = nh.subscribe<sensor_msgs::Range>
			("/mavros/distance_sensor/tfmini_pub", 10, tfmini_cb);
    ros::Subscriber center_sub = nh.subscribe<landing::center>
            ("center", 10, center_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	
	ros::Subscriber joy_sub = nh.subscribe("/keys",10,&joyCallBack);	// subscribe
    
    double kp_pixelx,kp_pixely,min_landing_height,near_landing_height,max_height;
	double min_tf_height;
    double decend_vel_far,decend_vel_near;
    int min_err_pixel_x,min_err_pixel_y;
    
    nh.param("kp_pixelx",kp_pixelx,399.);
    nh.param("kp_pixely",kp_pixely,399.);
    nh.param("min_landing_height",min_landing_height,0.14);
	nh.param("min_tf_height",min_tf_height,0.4);
    nh.param("near_landing_height",near_landing_height,0.39);
    nh.param("max_height",max_height,1.99);
    nh.param("decend_vel_far",decend_vel_far,0.19);
    nh.param("decend_vel_near",decend_vel_near,0.19);
    nh.param("min_err_pixel_x",min_err_pixel_x,19);
    nh.param("min_err_pixel_y",min_err_pixel_y,19);
    
    ROS_INFO("kp_pixelx: %lf",kp_pixelx);
    ROS_INFO("kp_pixely: %lf",kp_pixely);
    ROS_INFO("min_landing_height: %lf",min_landing_height);
	ROS_INFO("min_tf_height: %lf",min_tf_height);
    ROS_INFO("near_landing_height: %lf",near_landing_height);
    ROS_INFO("decend_vel_far: %lf",decend_vel_far);
    ROS_INFO("decend_vel_near: %lf", decend_vel_near);
    ROS_INFO("min_err_pixel_x: %d",min_err_pixel_x);
    ROS_INFO("min_err_pixel_y: %d",min_err_pixel_y);
    ROS_INFO("max_height: %lf",max_height);
 
    ros::Rate rate(20.0);
 
    while(ros::ok() && current_state.connected){
		ROS_INFO("--------connected------");
        ros::spinOnce();
        rate.sleep();
    }
 
    geometry_msgs::PoseStamped pose;//姿态控制
    pose.pose.position.x = local_position.pose.position.x;
    pose.pose.position.y = local_position.pose.position.y;
    pose.pose.position.z = max_height;
    
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
    
    int land_cnt=0;
 
    ros::Time last_request = ros::Time::now();
 	
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
				pose.pose.position.x = local_position.pose.position.x;
				pose.pose.position.y = local_position.pose.position.y;
				pose.pose.position.z = max_height;
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
        else if(fabs( local_position.pose.position.z - max_height )<0.15)
        {
            ROS_INFO("tracking landing start");
            break;
        }
		ROS_INFO("local_pos: x %f, z %f",local_position.pose.position.x,local_position.pose.position.z);
		ROS_INFO("tfmini height: %f", tfmini_height);
        local_pos_pub.publish(pose);
 
        ros::spinOnce();
        rate.sleep();
    }
    
    //逛一圈
	// last_request = ros::Time::now();
	// while(ros::ok())
	// {
	// 	if(ros::Time::now() - last_request > ros::Duration(5.0))
	// 		break;
	//     vel.twist.linear.x = 1;
	// 	vel.twist.linear.y = 0;
	// 	vel.twist.linear.z = 0;
	// 	local_vel_pub.publish(vel);
	// 	ros::spinOnce();
    // 	rate.sleep();
	// }
	// last_request = ros::Time::now();
	// while(ros::ok())
	// {
	// 	if(ros::Time::now() - last_request > ros::Duration(5.0))
	// 		break;
	//     vel.twist.linear.x = 0;
	// 	vel.twist.linear.y = 1;
	// 	vel.twist.linear.z = 0;
	// 	local_vel_pub.publish(vel);
	// 	ros::spinOnce();
    // 	rate.sleep();
	// }
	// last_request = ros::Time::now();
	// while(ros::ok())
	// {
	// 	if(ros::Time::now() - last_request > ros::Duration(5.0))
	// 		break;
	//     vel.twist.linear.x = -1;
	// 	vel.twist.linear.y = 0;
	// 	vel.twist.linear.z = 0;
	// 	local_vel_pub.publish(vel);
	// 	ros::spinOnce();
    // 	rate.sleep();
	// }
	// last_request = ros::Time::now();
	// while(ros::ok())
	// {
	// 	if(ros::Time::now() - last_request > ros::Duration(5.0))
	// 		break;
	//     vel.twist.linear.x = 0;
	// 	vel.twist.linear.y = -1;
	// 	vel.twist.linear.z = 0;
	// 	local_vel_pub.publish(vel);
	// 	ros::spinOnce();
    // 	rate.sleep();
	// }
	
	//控制降落部分
	while(ros::ok())
	{
		if(land_mode == 1)
		{
			ROS_INFO("keyboard interrupt");
			break;
		}
		//高度低于0.3时转为降落模式
		if(local_position.pose.position.z < min_landing_height && tfmini_height<min_tf_height&& tfmini_height>0)
			break;
		//如果找到地标，控制方向
		if(landmark.iffind)
		{
			ROS_INFO("find mark");
			//计算两方向err
			double err_x = landmark.height/2.0 - landmark.x;
			double err_y = landmark.width/2.0 - landmark.y;
			
			ROS_INFO_STREAM("state="<<err_x<<" "<<err_y);
			//速度控制
			vel.twist.linear.x = err_x/kp_pixelx;
			vel.twist.linear.y = err_y/kp_pixely;
			vel.twist.linear.z = 0.;
			//如果位置很正开始降落
			//if((fabs(err_x) < 12 &&fabs(err_y) < 12)||(local_position.pose.position.z > 0.3&&fabs(err_x) < 18 &&fabs(err_y) < 18))
			if(fabs(err_x) < min_err_pixel_x &&fabs(err_y) < min_err_pixel_y)
			{
				if(local_position.pose.position.z < near_landing_height)
				{
				    if(land_cnt<=5)
					land_cnt ++;
				    else
				    {
					land_cnt = 0;
					ROS_INFO("good 2 decend-near");
				        vel.twist.linear.z = -decend_vel_near;
				    }
				    
				}
				else
				{// height is higer than 0.4m
				    land_cnt=0;
				    ROS_INFO("good 2 decend");
				    vel.twist.linear.z = -decend_vel_far;
				}

			}	
			else
			{
				land_cnt=0;
				vel.twist.linear.z = 0;
			}
				
			local_vel_pub.publish(vel);
			ros::spinOnce();
			rate.sleep();
    	}
    	//如果找不到矩形地标，回到2m高度
    	else
    	{
		ROS_INFO("lose mark: back up");
		land_cnt=0;
    		pose.pose.position.x = local_position.pose.position.x;
			pose.pose.position.y = local_position.pose.position.y;
			pose.pose.position.z = max_height;
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
 
    return 0;
}
