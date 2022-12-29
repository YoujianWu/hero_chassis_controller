//
// Created by kook on 22-11-27.
//

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h> //Allow us to get and use joints ' infomation
#include <control_toolbox/pid.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace hero_chassis_controller {

    class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
        //JointCommandInterface for commanding effort-based joints
    public:
        HeroChassisController() = default;
        ~HeroChassisController() override ;
        // The meaning of override is that I will write its codes again

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
        //This step is to load the connected joint and get the Param from the Param server

        void update(const ros::Time &time, const ros::Duration &period) override;
        //This meaning of it may be equal to while(1){}

        hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;



    private:
        //Nodehandle Pattern
        int loop_count_;
        std::unique_ptr<realtime_tools::RealtimePublisher<
                control_msgs::JointControllerState> > controller_state_publisher_ ;
        //Internal PID controller
        control_toolbox::Pid pid_controller_1,pid_controller_2,pid_controller_3,pid_controller_4;

        ros::Subscriber sub_command_;
        //callback function for subscriber
        void return_state(const geometry_msgs::TwistConstPtr &state_msg);



        //Mecanum wheels pattern
        //expected velocity of the chassis(through chassis to calculate the motor)
        double Vx,Vy,Vw;
        //actual velocity of wheels
        double vel_wheel_act[5];
        //expected vocity of four wheels
        double vel_wheel_exp[5];
        //comand velocity of four wheels through effort command
        double wheel_cmd[5];
        double Wheel_track;
        double Wheel_base;
        double Wheel_R = 0.07625;
        //through chassis to calculate the motor
        void compute_mecanum_vel();

        //Time Pattern
        int state_{};
        ros::Time last_change_;

    };

}//namespace hero_chassis_controller
#endif //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
