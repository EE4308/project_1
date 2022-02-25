#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include "common.hpp"

double imu_ang_vel = -10, imu_lin_acc = 0; // unlikely to be spinning at -10 at the start
void cbImu(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_ang_vel = msg->angular_velocity.z;
    imu_lin_acc = msg->linear_acceleration.x;
}

double wheel_l = 10, wheel_r = 10; // init as 10 bcos both are unlikely to be exactly 10 (both can start at non-zero if u reset the sim). whole doubles can also be exactly compared
void cbWheels(const sensor_msgs::JointState::ConstPtr &msg)
{
    wheel_l = msg->position[1]; // double check the topic. it is labelled there
    wheel_r = msg->position[0]; // double check the topic. it is labelled there
}

nav_msgs::Odometry msg_odom;
void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    msg_odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_motion");
    ros::NodeHandle nh;

    // Parse ROS parameters
    bool use_internal_odom;
    if (!nh.param("use_internal_odom", use_internal_odom, true))
        ROS_WARN(" TMOVE : Param use_internal_odom not found, set to true");
    bool verbose;
    if (!nh.param("verbose_motion", verbose, false))
        ROS_WARN(" TMOVE : Param verbose_motion not found, set to false");

    // Publisher
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 1, true);

    // Prepare published message
    geometry_msgs::PoseStamped pose_rbt;
    pose_rbt.header.frame_id = "map"; // for rviz

    if (use_internal_odom)
    { // subscribes to odom topic --> is the exact simulated position in gazebo; when used in real life, is derived from wheel encoders (no imu).
        // Subscriber
        ros::Subscriber sub_odom = nh.subscribe("odom", 1, &cbOdom);

        // initialise rate
        ros::Rate rate(25);

        // wait for dependent nodes to load (check topics)
        ROS_INFO("TMOTION: Waiting for topics");
        while (ros::ok() && nh.param("run", true) && msg_odom.header.seq == 0) // dependent on odom
        {
            rate.sleep();
            ros::spinOnce(); // update the topics
        }

        ROS_INFO("TMOTION: ===== BEGIN =====");

        // begin loop
        while (ros::ok() && nh.param("run", true))
        {
            // update topics
            ros::spinOnce();

            // write to published message
            pose_rbt.pose = msg_odom.pose.pose;

            // publish pose
            pub_pose.publish(pose_rbt);

            if (verbose)
            {
                // get ang_rbt from quaternion
                auto &q = pose_rbt.pose.orientation;
                double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
                double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);

                ROS_INFO("TMOTION: Pos(%7.3f, %7.3f)  Ang(%6.3f)  FVel(%6.3f)  AVel(%6.3f)",
                         pose_rbt.pose.position.x, pose_rbt.pose.position.y, atan2(siny_cosp, cosy_cosp),
                         msg_odom.twist.twist.linear.x, msg_odom.twist.twist.angular.z);
            }

            rate.sleep();
        }
    }
    else
    {
        // Parse additional ROS parameters
        Position pos_rbt;
        if (!nh.param("initial_x", pos_rbt.x, 0.0))
            ROS_WARN(" TMOVE : Param initial_x not found, set to 0.0");
        if (!nh.param("initial_y", pos_rbt.y, 0.0))
            ROS_WARN(" TMOVE : Param initial_y not found, set to 0.0");
        double wheel_radius;
        if (!nh.param("wheel_radius", wheel_radius, 0.033))
            ROS_WARN(" TMOVE : Param wheel_radius not found, set to 0.033");
        double axle_track;
        if (!nh.param("axle_track", axle_track, 0.16))
            ROS_WARN(" TMOVE : Param axle_track not found, set to 0.16");
        double weight_odom_v;
        if (!nh.param("weight_odom_v", weight_odom_v, 0.5))
            ROS_WARN(" TMOVE : Param weight_odom_v not found, set to 0.5");
        double weight_odom_w;
        if (!nh.param("weight_odom_w", weight_odom_w, 0.5))
            ROS_WARN(" TMOVE : Param weight_odom_w not found, set to 0.5");
        double weight_imu_v = 1 - weight_odom_v;
        double weight_imu_w = 1 - weight_odom_w;
        double straight_thresh;
        if (!nh.param("straight_thresh", straight_thresh, 0.05))
            ROS_WARN(" TMOVE : Param straight_thresh not found, set to 0.05");
        double motion_iter_rate;
        if (!nh.param("motion_iter_rate", motion_iter_rate, 50.0))
            ROS_WARN(" TMOVE : Param motion_iter_rate not found, set to 50");
        bool auto_tune;
        if (!nh.param("auto_tune", auto_tune, false))
            ROS_WARN(" auto_tune : Param auto_tune not found, set to false");

        // Subscribers
        ros::Subscriber sub_wheels = nh.subscribe("joint_states", 1, &cbWheels);
        ros::Subscriber sub_imu = nh.subscribe("imu", 1, &cbImu);
        ros::Subscriber sub_odom = nh.subscribe("odom", 1, &cbOdom); // USED FOR COMPARISON WITH CALCULATED

        // initialise rate
        ros::Rate rate(motion_iter_rate); // higher rate for better estimation

        // initialise message for publishing
        pose_rbt.header.frame_id = "world"; // for rviz to visualise model wrt space
        pose_rbt.pose.orientation.x = 0;    // 3 DOF robot; the default is zero anyway, so this line is unnecessary. but kept it here to highlight 3DOF
        pose_rbt.pose.orientation.y = 0;    // 3 DOF robot; the default is zero anyway, so this line is unnecessary. but kept it here to highlight 3DOF

        // wait for dependent nodes to load (check topics)
        ROS_INFO("TMOTION: Waiting for topics");
        while (ros::ok() && nh.param("run", true) && (wheel_l == 10 || wheel_r == 10 || imu_ang_vel == -10)) // dependent on imu and wheels
        {
            rate.sleep();
            ros::spinOnce(); // update the topics
        }

        ROS_INFO("TMOTION: ===== BEGIN =====");

        // declare / initialise other variables
        double ang_rbt = 0; // robot always start at zero.
        double lin_vel = 0, ang_vel = 0;
        double prev_time = ros::Time::now().toSec();
        double dt = 0;
        ////////////////// DECLARE VARIABLES HERE //////////////////
        // odom model
        double prev_wheel_l = wheel_l;
        double prev_wheel_r = wheel_r;
        double thetha_wheel_l = 0;
        double thetha_wheel_r = 0;
        double odom_lin_vel = 0;
        double odom_ang_vel = 0;

        // imu model
        double imu_lin_vel = 0;

        // calculated lin and ang vel
        double vt = 0;
        double wt = 0;

        // displacement
        double prev_ang_rbt = ang_rbt;
        double turn_radius = 0;

        // tuning
        double d_x = 0;
        double d_y = 0;
        double d_a = 0;
        double tick = 0;

        // loop
        while (ros::ok() && nh.param("run", true))
        {
            // update topics
            ros::spinOnce();

            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0) // ros doesn't tick the time fast enough
                continue;
            prev_time += dt;

            ////////////////// MOTION FILTER HERE //////////////////

            thetha_wheel_l = wheel_l - prev_wheel_l;
            thetha_wheel_r = wheel_r - prev_wheel_r;
            prev_wheel_l = wheel_l;
            prev_wheel_r = wheel_r;

            // odom model
            odom_lin_vel = wheel_radius * (thetha_wheel_l + thetha_wheel_r) / (2 * dt);
            odom_ang_vel = wheel_radius * (thetha_wheel_r - thetha_wheel_l) / (axle_track * dt);

            // imu model
            imu_lin_vel = vt + imu_lin_acc * dt;

            // weighted average calc
            vt = weight_odom_v * odom_lin_vel + weight_imu_v * imu_lin_vel;
            wt = weight_odom_w * odom_ang_vel + weight_imu_w * imu_ang_vel;
            ang_rbt = prev_ang_rbt + wt* dt;
            turn_radius = vt / wt;

            pos_rbt.x = abs(wt) > straight_thresh ? pos_rbt.x + turn_radius * (-sin(prev_ang_rbt) + sin(ang_rbt))
                                                  : pos_rbt.x + vt * dt * cos(prev_ang_rbt);

            pos_rbt.y = abs(wt) > straight_thresh ? pos_rbt.y + turn_radius * (cos(prev_ang_rbt) - cos(ang_rbt))
                                                  : pos_rbt.y + vt * dt * sin(prev_ang_rbt);

            prev_ang_rbt = ang_rbt;

            ROS_INFO("odom_vt(%3.3f)  imu_vt(%4.3f)", odom_lin_vel, imu_lin_vel);
            ROS_INFO("odom_wt(%1.2f)  imu_wt(%2.2f)", odom_ang_vel, imu_ang_vel);

            auto &q = msg_odom.pose.pose.orientation;
            double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            double true_ang = atan2(siny_cosp, cosy_cosp);
            d_x += abs(pos_rbt.x - msg_odom.pose.pose.position.x);
            d_y += abs(pos_rbt.y - msg_odom.pose.pose.position.y);
            d_a += abs(ang_rbt - true_ang);
            tick += 1;

            ROS_INFO("weight_odom_V(%2.2f)  weight_odom_W(%3.2f)  time(%4.1f)",
                     weight_odom_v, weight_odom_w, prev_time);
            ROS_INFO("D_X_Live(%7.3f)  D_Y_Live(%6.3f)  D_A_Live(%5.3f)",
                     abs(pos_rbt.x - msg_odom.pose.pose.position.x), abs(pos_rbt.y - msg_odom.pose.pose.position.y), abs(ang_rbt - true_ang));
            ROS_INFO("D_X_Avg(%7.3f)   D_Y_Avg(%6.3f)   D_A_Avg(%5.3f)",
                     d_x / tick, d_y / tick, d_a / tick);
            ROS_INFO("comparison: True_X(%7.3f)  True_Y(%6.3f)  True_A(%5.3f)",
                     msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y, true_ang);
            ROS_INFO("comparison: Calc_X(%7.3f)  Calc_Y(%6.3f)  Calc_A(%5.3f)", pos_rbt.x, pos_rbt.y, ang_rbt);
            if (auto_tune)
            {
                // FIND THE DIRECTION OF MINIMA & update weight odom w
                // Calculate Linear Velocity from Pos
                double lin_vel_real = msg_odom.twist.twist.linear.x;
                double diff_vt = abs(vt - msg_odom.twist.twist.linear.x);

                ROS_INFO("REAL_LIN_VEL(%2.2f) CALC_LIN_VEL(%3.2f) DIFF_VT(%4.2f)", lin_vel_real, vt, diff_vt);

                bool weight_imu_v_ok = (weight_imu_v > 0) && (weight_imu_v < 1);
                bool weight_odom_v_ok = (weight_odom_v > 0) && (weight_odom_v < 1);
                bool weight_imu_w_ok = (weight_imu_w > 0) && (weight_imu_w < 1);
                bool weight_odom_w_ok = (weight_odom_w > 0) && (weight_odom_w < 1);
                if (d_a > 0.1 || diff_vt > 0.1)
                { // Use 0.05 to avoid tuning to noise
                    // Calculate diff btw true angle against odom_wt and imu_wt
                    double odom_true_ang_diff = abs(true_ang - odom_ang_vel);
                    double imu_true_ang_diff = abs(true_ang - imu_ang_vel);

                    double odom_true_lin_diff = abs(lin_vel_real - odom_lin_vel);
                    double imu_true_lin_diff = abs(lin_vel_real - imu_lin_vel);

                    if (weight_odom_w_ok && weight_imu_w_ok)
                    {
                        // If Odom_True_Ang_Diff > Imu_True_Ang_Diff, increase weight for IMU
                        if (odom_true_ang_diff > imu_true_ang_diff)
                        {
                            if (weight_imu_w <= 0.95)
                            {
                                weight_imu_w += 0.05;
                                weight_odom_w = 1 - weight_imu_w;
                            }
                        }
                        // If Imu_true_ang_diff > Odom_true_ang_diff, increase weight for Odom
                        if (imu_true_ang_diff > odom_true_ang_diff)
                        {
                            if (weight_odom_w <= 0.95)
                            {
                                weight_odom_w += 0.05;
                                weight_imu_w = 1 - weight_odom_w;
                            }
                        }
                    }
                    if (weight_imu_v_ok && weight_odom_v_ok)
                    {
                        // Similarly Update Weights for V_diff
                        if (odom_true_lin_diff > imu_true_lin_diff)
                        {
                            if (weight_imu_v <= 0.95)
                            {
                                weight_imu_v += 0.05;
                                weight_odom_v = 1 - weight_imu_v;
                            }
                        }
                        if (imu_true_lin_diff > odom_true_lin_diff)
                        {
                            if (weight_odom_v <= 0.95)
                            {
                                weight_odom_v += 0.05;
                                weight_imu_v = 1 - weight_odom_v;
                            }
                        }
                    }

                    ROS_INFO("UPDATED WEIGHTS_wt: new_weight_imu_w(%2.2f) new_weight_odom_w(%3.2f)", weight_imu_w, weight_odom_w);
                    ROS_INFO("UPDATED WEIGHTS_vt: new_weight_imu_v(%2.2f) new_weight_odom_v(%3.2f)", weight_imu_v, weight_odom_v);
                }
            }

            // publish the pose
            // inject position and calculate quaternion for pose message, and publish
            pose_rbt.pose.position.x = pos_rbt.x;
            pose_rbt.pose.position.y = pos_rbt.y;
            pose_rbt.pose.orientation.w = cos(ang_rbt / 2);
            pose_rbt.pose.orientation.z = sin(ang_rbt / 2);
            pub_pose.publish(pose_rbt);

            if (verbose)
            {
                ROS_INFO("TMOTION: Pos(%7.3f, %7.3f)  Ang(%6.3f)",
                         pos_rbt.x, pos_rbt.y, ang_rbt);
            }

            // sleep until the end of the required frequency
            rate.sleep();
        }
    }

    ROS_INFO("TMOTION: ===== END =====");
    return 0;
}