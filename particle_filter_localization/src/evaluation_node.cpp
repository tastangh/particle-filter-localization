#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <cmath>

geometry_msgs::PoseStamped latest_estimated_pose;
geometry_msgs::Pose latest_ground_truth_pose;
bool has_estimate = false;
bool has_truth = false;

std::ofstream log_file;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    latest_estimated_pose = *msg;
    has_estimate = true;
}

void truthCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // Assume husky is at index 13 (verify with rostopic echo once!)
    int husky_index = -1;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "husky") {
            husky_index = i;
            break;
        }
    }
    if (husky_index == -1) return;

    latest_ground_truth_pose = msg->pose[husky_index];
    has_truth = true;
}

void computeError() {
    double dx = latest_ground_truth_pose.position.x - latest_estimated_pose.pose.position.x;
    double dy = latest_ground_truth_pose.position.y - latest_estimated_pose.pose.position.y;
    double pos_error = std::sqrt(dx * dx + dy * dy);

    tf::Quaternion q1, q2;
    tf::quaternionMsgToTF(latest_ground_truth_pose.orientation, q1);
    tf::quaternionMsgToTF(latest_estimated_pose.pose.orientation, q2);
    double yaw1 = tf::getYaw(q1);
    double yaw2 = tf::getYaw(q2);
    double yaw_error = std::fabs(yaw1 - yaw2);

    log_file << latest_estimated_pose.header.stamp << ","
             << latest_ground_truth_pose.position.x << ","
             << latest_ground_truth_pose.position.y << ","
             << latest_estimated_pose.pose.position.x << ","
             << latest_estimated_pose.pose.position.y << ","
             << pos_error << ","
             << yaw_error << std::endl;

    ROS_INFO("Pos Error: %.2f m | Yaw Error: %.2f rad", pos_error, yaw_error);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "particle_filter_evaluator");
    ros::NodeHandle nh;

    log_file.open("evaluation_log.csv");
    log_file << "time,gt_x,gt_y,est_x,est_y,pos_error,yaw_error\n";

    ros::Subscriber pose_sub = nh.subscribe("/particle_pose", 1, poseCallback);
    ros::Subscriber truth_sub = nh.subscribe("/gazebo/model_states", 1, truthCallback);

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        if (has_estimate && has_truth) {
            computeError();
            has_estimate = false;
            has_truth = false;
        }
        rate.sleep();
    }

    log_file.close();
    return 0;
}
