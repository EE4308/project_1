#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include "common.hpp"
#include <fstream>

bool target_changed = false;
Position target;
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target.x = msg->point.x;
    target.y = msg->point.y;
}

Position pos_rbt(0, 0);
double ang_rbt = 10; // set to 10, because ang_rbt is between -pi and pi, and integer for correct comparison while waiting for motion to load
void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    pos_rbt.x = p.x;
    pos_rbt.y = p.y;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    ang_rbt = atan2(siny_cosp, cosy_cosp);
}

double check_threshold(double error_ang, double error_threshold)
{
    // // Using Normal Curve to Return 1 for values close to 0, and 0 for values far from it
    // // Error Threshold affects the standard deviation of the curve (higher threshold = greater width)
    // return exp(-0.5*pow(error_ang/error_threshold, 2))/(error_threshold*sqrt(2*M_PI));
    if (abs(error_ang) < error_threshold)
    {
        return 1 - abs(error_ang) / error_threshold;
    }
    else
    {
        return 0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;

    std::ofstream data_file;
    data_file.open("/home/rishab/a456w/data.txt");

    // Get ROS Parameters
    bool enable_move;
    if (!nh.param("enable_move", enable_move, true))
        ROS_WARN(" TMOVE : Param enable_move not found, set to true");
    bool verbose;
    if (!nh.param("verbose_move", verbose, false))
        ROS_WARN(" TMOVE : Param verbose_move not found, set to false");
    double Kp_lin;
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
        ROS_WARN(" TMOVE : Param Kp_lin not found, set to 1.0");
    double Ki_lin;
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
        ROS_WARN(" TMOVE : Param Ki_lin not found, set to 0");
    double Kd_lin;
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
        ROS_WARN(" TMOVE : Param Kd_lin not found, set to 0");
    double max_lin_vel;
    if (!nh.param("max_lin_vel", max_lin_vel, 0.22))
        ROS_WARN(" TMOVE : Param max_lin_vel not found, set to 0.22");
    double max_lin_acc;
    if (!nh.param("max_lin_acc", max_lin_acc, 1.0))
        ROS_WARN(" TMOVE : Param max_lin_acc not found, set to 1");
    double Kp_ang;
    if (!nh.param("Kp_ang", Kp_ang, 1.0))
        ROS_WARN(" TMOVE : Param Kp_ang not found, set to 1.0");
    double Ki_ang;
    if (!nh.param("Ki_ang", Ki_ang, 0.0))
        ROS_WARN(" TMOVE : Param Ki_ang not found, set to 0");
    double Kd_ang;
    if (!nh.param("Kd_ang", Kd_ang, 0.0))
        ROS_WARN(" TMOVE : Param Kd_ang not found, set to 0");
    double max_ang_vel;
    if (!nh.param("max_ang_vel", max_ang_vel, 2.84))
        ROS_WARN(" TMOVE : Param max_ang_vel not found, set to 2.84");
    double max_ang_acc;
    if (!nh.param("max_ang_acc", max_ang_acc, 4.0))
        ROS_WARN(" TMOVE : Param max_ang_acc not found, set to 4");
    double move_iter_rate;
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
        ROS_WARN(" TMOVE : Param move_iter_rate not found, set to 25");
    double error_threshold;
    if (!nh.param("error_threshold", error_threshold, 1.0))
        ROS_WARN(" TMOVE : Param error_threshold not found, set to 1");
    bool enable_reverse;
    if (!nh.param("enable_reverse", enable_reverse, false))
        ROS_WARN(" TMOVE: Turn reverse move off by default");

    // Subscribers
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);

    // Publishers
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    // prepare published messages
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // Setup rate
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic

    // wait for other nodes to load
    ROS_INFO(" TMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && ang_rbt == 10) // not dependent on main.cpp, but on motion.cpp
    {
        rate.sleep();
        ros::spinOnce(); // update the topics
    }

    // Setup variables
    double cmd_lin_vel = 0, cmd_ang_vel = 0;
    double dt;
    double prev_time = ros::Time::now().toSec();

    ////////////////// DECLARE VARIABLES HERE //////////////////

    // Positional Error
    double lin_error = dist_euc(pos_rbt, target);
    double lin_error_prev = 0;
    double lin_error_sum = lin_error;
    double P_lin;
    double I_lin;
    double D_lin;
    double PID_lin_sum;
    double U_lin;

    // Angular Error
    double ang_error = limit_angle(heading(pos_rbt, target) - ang_rbt);
    double ang_error_prev = 0;
    double ang_error_sum = ang_error;
    double P_ang;
    double I_ang;
    double D_ang;
    double PID_ang_sum;
    double U_ang;

    double control_lin_acceleration;
    double control_ang_acceleration;

    ROS_INFO(" TMOVE : ===== BEGIN =====");

    // main loop
    if (enable_move)
    {
        while (ros::ok() && nh.param("run", true))
        {
            // update all topics
            ros::spinOnce();

            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0) // ros doesn't tick the time fast enough
                continue;
            prev_time += dt;

            ////////////////// MOTION CONTROLLER HERE //////////////////

            // PID for Linear Velocity
            lin_error = dist_euc(pos_rbt, target);
            lin_error_sum += lin_error * dt;
            P_lin = Kp_lin * lin_error;
            I_lin = Ki_lin * lin_error_sum;
            D_lin = Kd_lin * (lin_error - lin_error_prev) / dt;
            lin_error_prev = lin_error;
            PID_lin_sum = P_lin + I_lin + D_lin;

            // PID for Angular Velocity
            ang_error = limit_angle(heading(pos_rbt, target) - ang_rbt);
            bool reverse = false;
            if (enable_reverse)
            {
                if (ang_error > M_PI / 2)
                {
                    reverse = true;
                    ang_error = -(M_PI - ang_error);
                } else if (ang_error < -M_PI /2) {
                    reverse = true;
                    ang_error = M_PI + ang_error; // Suppose heading is 10deg, expected is 105 deg, angular  error is -95 deg
                }
                // Reverse teh PID Effort direction if reverse is true
                if (reverse) {
                    ROS_INFO("REVERSED MOTION");
                    PID_lin_sum = - PID_lin_sum;
                }
            }

            ang_error_sum += ang_error * dt;
            P_ang = Kp_ang * ang_error;
            I_ang = Ki_ang * ang_error_sum;
            D_ang = Kd_ang * (ang_error - ang_error_prev) / dt;
            PID_ang_sum = P_ang + I_ang + D_ang;
            ang_error_prev = ang_error;

            // Coupling Angular Error with Linear Velocity
            /**
             * Trianlge function curve in check_threshold() returns 1 for values at Ek,ang = 0
             * and 0 for values greater than error_threshold in both directions. For values in between
             * error_threshold and Ek,ang, returns value closer to 0, the greater the magnitude of Ek,ang.
             */
            double threshold_value = check_threshold(ang_error, error_threshold);
            ROS_INFO("[U_lin] ang_error: %f\t check_threshold: %f", ang_error, threshold_value);
            U_lin = PID_lin_sum * threshold_value;
            U_ang = PID_ang_sum;

            // Constraint Control Signal Acceleration & Velocity
            /**
             * sat() is implemented in common.cpp and declaration added to common.hpp
             */

            control_lin_acceleration = (U_lin - cmd_lin_vel) / dt;
            control_lin_acceleration = sat(control_lin_acceleration, max_lin_acc);
            cmd_lin_vel = sat(U_lin + control_lin_acceleration * dt, max_lin_vel);
            ROS_INFO("[cmd_lin_vel] control_lin_acceleration: %f\t cmd_lin_vel: %f", control_lin_acceleration, cmd_lin_vel);

            control_ang_acceleration = (U_ang - cmd_ang_vel) / dt;
            control_ang_acceleration = sat(control_ang_acceleration, max_ang_acc);
            cmd_ang_vel = sat(U_ang + control_ang_acceleration * dt, max_ang_vel);
            ROS_INFO("[cmd_ang_vel] control_ang_acceleration: %f\t cmd_ang_vel: %f", control_ang_acceleration, cmd_ang_vel);

            // publish speeds
            msg_cmd.linear.x = cmd_lin_vel;
            msg_cmd.angular.z = cmd_ang_vel;
            pub_cmd.publish(msg_cmd);


            // LOG to datafile
            data_file << ros::Time::now().toSec() << "\t" << lin_error << "\t" << ang_error << "\t" << PID_ang_sum << "\t" << PID_lin_sum << std::endl;

            // verbose
            if (verbose)
            {
                ROS_INFO(" TMOVE :  FV(%6.3f) AV(%6.3f)", cmd_lin_vel, cmd_ang_vel);
            }

            // wait for rate
            rate.sleep();
        }
    }

    //Update and save data file
    data_file.close();

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    ROS_INFO(" TMOVE : ===== END =====");
    return 0;
}