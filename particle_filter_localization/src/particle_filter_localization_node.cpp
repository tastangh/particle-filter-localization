#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <random>
#include <vector>
#include <cmath>

struct Particle {
    double x, y, theta;
    double weight;
};

class ParticleFilter {
public:
    ParticleFilter(int num_particles) : num_particles_(num_particles) {
        initParticles();
    }

    void initParticles() {
        std::default_random_engine gen;
        std::uniform_real_distribution<double> x_dist(-5.0, 5.0);
        std::uniform_real_distribution<double> y_dist(-5.0, 5.0);
        std::uniform_real_distribution<double> t_dist(-M_PI, M_PI);

        particles_.clear();
        for (int i = 0; i < num_particles_; ++i) {
            particles_.push_back({x_dist(gen), y_dist(gen), t_dist(gen), 1.0});
        }
    }

    void motionUpdate(const nav_msgs::Odometry& odom_msg) {
        for (auto& p : particles_) {
            p.x += 0.01 * std::cos(p.theta);
            p.y += 0.01 * std::sin(p.theta);
            // Optional: Add noise and theta update if desired
        }
    }

    void sensorUpdate(const sensor_msgs::LaserScan& scan_msg) {
        for (auto& p : particles_) {
            p.weight = 1.0; // Uniform weights, placeholder
        }
    }

    Particle estimatePose() {
        Particle best = particles_[0];
        for (const auto& p : particles_) {
            if (p.weight > best.weight) {
                best = p;
            }
        }
        return best;
    }

    Particle getBestEstimate() {
        return estimatePose();
    }

private:
    int num_particles_;
    std::vector<Particle> particles_;
};

// Global
ParticleFilter* pf_ptr = nullptr;
ros::Publisher pose_pub;
tf::TransformBroadcaster* tf_broadcaster_ptr = nullptr;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (pf_ptr) {
        pf_ptr->motionUpdate(*msg);
    }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (pf_ptr) {
        pf_ptr->sensorUpdate(*msg);

        Particle est = pf_ptr->getBestEstimate();

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = msg->header.stamp;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = est.x;
        pose_msg.pose.position.y = est.y;
        pose_msg.pose.position.z = 0;
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = sin(est.theta / 2.0);
        pose_msg.pose.orientation.w = cos(est.theta / 2.0);

        pose_pub.publish(pose_msg);

        // TF yayınla
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(est.x, est.y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, est.theta);
        transform.setRotation(q);
        tf_broadcaster_ptr->sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "base_link"));
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "particle_filter_node");
    ros::NodeHandle nh;

    int num_particles = 100;
    nh.getParam("num_particles", num_particles);
    ParticleFilter pf(num_particles);
    pf_ptr = &pf;

    tf::TransformBroadcaster tf_broadcaster;
    tf_broadcaster_ptr = &tf_broadcaster;

    ros::Subscriber odom_sub = nh.subscribe("/husky_velocity_controller/odom", 1, odomCallback);
    ros::Subscriber scan_sub = nh.subscribe("/front/scan", 1, scanCallback);
    
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/particle_pose", 1);

    ros::spin();
    return 0;
}
