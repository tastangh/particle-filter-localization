// particle_filter_localization_node.cpp

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <random>
#include <vector>
#include <cmath>

struct Particle {
    double x, y, theta, weight;
};

const int NUM_PARTICLES = 500;
const double ALPHA1 = 0.001, ALPHA2 = 0.001;
const double ALPHA3 = 0.001, ALPHA4 = 0.001;
const double ALPHA5 = 0.0001, ALPHA6 = 0.0001;
const double SIGMA_R = 0.5, SIGMA_PHI = 0.1, SIGMA_S = 0.05;

std::vector<Particle> particles;
ros::Publisher pose_pub, cloud_pub;
double dt = 0.1; // Varsayılan zaman adımı

double sample_normal(double stddev) {
    static std::default_random_engine gen;
    std::normal_distribution<double> dist(0.0, stddev);
    return dist(gen);
}

void motion_update(double nu, double omega) {
    for (auto& p : particles) {
        double nu_hat = nu + sample_normal(ALPHA1 * nu * nu + ALPHA2 * omega * omega);
        double omega_hat = omega + sample_normal(ALPHA3 * nu * nu + ALPHA4 * omega * omega);
        double gamma_hat = sample_normal(ALPHA5 * nu * nu + ALPHA6 * omega * omega);

        if (fabs(omega_hat) > 1e-5) {
            p.x += -nu_hat / omega_hat * sin(p.theta) + nu_hat / omega_hat * sin(p.theta + omega_hat * dt);
            p.y += nu_hat / omega_hat * cos(p.theta) - nu_hat / omega_hat * cos(p.theta + omega_hat * dt);
        } else {
            p.x += nu_hat * dt * cos(p.theta);
            p.y += nu_hat * dt * sin(p.theta);
        }
        p.theta += omega_hat * dt + gamma_hat * dt;
    }
}

void normalize_weights() {
    double sum = 0.0;
    for (const auto& p : particles) sum += p.weight;
    for (auto& p : particles) p.weight /= sum + 1e-6;
}

std::vector<Particle> resample_particles() {
    std::vector<Particle> new_particles;
    std::vector<double> cumulative;
    cumulative.push_back(particles[0].weight);
    for (size_t i = 1; i < particles.size(); ++i) {
        cumulative.push_back(cumulative[i-1] + particles[i].weight);
    }

    std::default_random_engine gen;
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (int i = 0; i < NUM_PARTICLES; ++i) {
        double r = dist(gen);
        for (size_t j = 0; j < cumulative.size(); ++j) {
            if (r <= cumulative[j]) {
                new_particles.push_back(particles[j]);
                break;
            }
        }
    }
    return new_particles;
}

void publish_estimate() {
    double x = 0, y = 0, cos_sum = 0, sin_sum = 0;
    for (const auto& p : particles) {
        x += p.x * p.weight;
        y += p.y * p.weight;
        cos_sum += cos(p.theta) * p.weight;
        sin_sum += sin(p.theta) * p.weight;
    }
    double theta = atan2(sin_sum, cos_sum);

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    tf::Quaternion q = tf::createQuaternionFromYaw(theta);
    tf::quaternionTFToMsg(q, pose.pose.orientation);
    pose_pub.publish(pose);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    double nu = msg->twist.twist.linear.x;
    double omega = msg->twist.twist.angular.z;
    motion_update(nu, omega);
    normalize_weights();
    particles = resample_particles();
    publish_estimate();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "particle_filter_localization_node");
    ros::NodeHandle nh;

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/particle_pose", 10);

    ros::Subscriber odom_sub = nh.subscribe("/husky_velocity_controller/odom", 10, odom_callback);

    // Parçacıkları başlangıçta rasgele yerleştir
    std::default_random_engine gen;
    std::uniform_real_distribution<double> dist_x(-20, 20);
    std::uniform_real_distribution<double> dist_y(-20, 20);
    std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);
    particles.resize(NUM_PARTICLES);
    for (auto& p : particles) {
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.0 / NUM_PARTICLES;
    }

    ros::spin();
    return 0;
}
