//
// Created by kook on 22-11-27.
//

#include "hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller{

    HeroChassisController::~HeroChassisController(){
        sub_command_.shutdown();
    }

    bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                     ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {

        //get parameter through the NodeHandle
        controller_nh.getParam("Wheel_track",Wheel_track);
        controller_nh.getParam("Wheel_base",Wheel_base);


        //get joint information and handle from hardware_interface
        front_left_joint_ =
                effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ =
                effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ =
                effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ =
                effort_joint_interface->getHandle("right_back_wheel_joint");

        //load PID controllers
        pid_controller_1.init(ros::NodeHandle(controller_nh,"pid_1"));
        pid_controller_2.init(ros::NodeHandle(controller_nh,"pid_2"));
        pid_controller_3.init(ros::NodeHandle(controller_nh,"pid_3"));
        pid_controller_4.init(ros::NodeHandle(controller_nh,"pid_4"));
        //Initialiaze velocitizes
        Vx=0.0;
        Vy=0.0;
        Vw=0.0;

        // Start realtime state publisher
        // warning: use std::make_unique
        controller_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher
                                                       <control_msgs::JointControllerState>>(root_nh, "state", 1);

        //initialize the subscriber
        sub_command_ =
                root_nh.subscribe<geometry_msgs::Twist>
                        ("cmd_vel", 1 , &HeroChassisController::return_state,this);

        //start the odom publisher
        odom_pub = root_nh.advertise<nav_msgs::Odometry>("odom", 50);

        return true;

    }

    void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {


        //get realtime velocity
        vel_wheel_act[1] = front_left_joint_.getVelocity();
        vel_wheel_act[2] = back_left_joint_.getVelocity();
        vel_wheel_act[3] = back_right_joint_.getVelocity();
        vel_wheel_act[4] = front_right_joint_.getVelocity();

        //odometry
        odometer();

        //calculate velocity of the mecanum wheel
        //Through your expected vx vy vw , which is the velocity of the chassis
        compute_mecanum_vel();
        //calculate error
        double error[5];
        for ( int i = 1 ; i < 5 ; i++ ) {
            error[i] = vel_wheel_exp[i] - vel_wheel_act[i];
        }
        //compute and set effort command by PID controller
        wheel_cmd[1] = pid_controller_1.computeCommand(error[1],period);
        wheel_cmd[2] = pid_controller_2.computeCommand(error[2],period);
        wheel_cmd[3] = pid_controller_3.computeCommand(error[3],period);
        wheel_cmd[4] = pid_controller_4.computeCommand(error[4],period);



        front_left_joint_.setCommand(wheel_cmd[1]);
        back_left_joint_.setCommand(wheel_cmd[2]);
        back_right_joint_.setCommand(wheel_cmd[3]);
        front_right_joint_.setCommand(wheel_cmd[4]);


        if(loop_count_ % 10 == 0)
        {
            if(controller_state_publisher_ && controller_state_publisher_->trylock())
            {
                controller_state_publisher_->msg_.header.stamp = time;
                controller_state_publisher_->msg_.set_point = vel_wheel_exp[1];
                controller_state_publisher_->msg_.process_value = front_left_joint_.getVelocity();
                controller_state_publisher_->msg_.error = error[1];
                controller_state_publisher_->msg_.time_step = period.toSec();
                controller_state_publisher_->msg_.command = wheel_cmd[1];

                double dummy;
                bool antiwindup;
                pid_controller_1.getGains(controller_state_publisher_->msg_.p,
                                          controller_state_publisher_->msg_.i,
                                          controller_state_publisher_->msg_.d,
                                          controller_state_publisher_->msg_.i_clamp,
                                          dummy,
                                          antiwindup);
                controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
                controller_state_publisher_->unlockAndPublish();
            }
        }
        loop_count_++;
    }
    void HeroChassisController::return_state(const geometry_msgs::TwistConstPtr &state_msg) {
        Vx = state_msg->linear.x;
        Vy = state_msg->linear.y;
        Vw = state_msg->angular.z;

    }
    void HeroChassisController::compute_mecanum_vel() {
        //wheels's order is anti-clockwise
        vel_wheel_exp[1] = ( Vx - Vy - Vw * ( Wheel_base + Wheel_track )/2 )/ Wheel_R;
        vel_wheel_exp[2] = ( Vx + Vy - Vw * ( Wheel_base + Wheel_track )/2 )/ Wheel_R;
        vel_wheel_exp[3] = ( Vx - Vy + Vw * ( Wheel_base + Wheel_track )/2 )/ Wheel_R;
        vel_wheel_exp[4] = ( Vx + Vy + Vw * ( Wheel_base + Wheel_track )/2 )/ Wheel_R;
    }
    void HeroChassisController::compute_chassis_vel() {
        Vx_chassis = ( vel_wheel_act[1] + vel_wheel_act[2] + vel_wheel_act[3] + vel_wheel_act[4] ) / Wheel_R * 2;
        Vy_chassis = (-vel_wheel_act[1] + vel_wheel_act[2] - vel_wheel_act[3] + vel_wheel_act[4] ) / Wheel_R * 2;
        Vw_chassis = (-vel_wheel_act[1] - vel_wheel_act[2] + vel_wheel_act[3] + vel_wheel_act[4] ) / Wheel_R * 2 / (Wheel_track + Wheel_base);
    }
    void HeroChassisController::odometer() {

        current_time = ros::Time::now();
        last_time = ros::Time::now();

        compute_chassis_vel();

        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (Vx_chassis * cos(th) - Vy_chassis * sin(th)) * dt;
        double delta_y = (Vx_chassis * sin(th) + Vy_chassis * cos(th)) * dt;
        double delta_th = Vw_chassis * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = Vx_chassis;
        odom.twist.twist.linear.y = Vy_chassis;
        odom.twist.twist.angular.z = Vw_chassis;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;
    }


}//namespace

PLUGINLIB_EXPORT_CLASS( hero_chassis_controller::HeroChassisController,controller_interface::ControllerBase)