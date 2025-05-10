#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <random>
#include <vector>
#include <map>
#include <cmath>

struct Landmark {
    double x, y, reflectivity;
};

struct Particle {
    double x, y, theta;
    double weight;
};

std::vector<Landmark> landmarks = {
    {-1.5,  1.5, 0.95}, {18.5, 11.5, 0.90}, {-21.5,  1.5, 0.85}, {-1.0, 11.5, 0.80},
    {-16.5, -3.0, 0.75}, {-1.5, -8.5, 0.70}, {18.5, 0.0, 0.65}, {15.0, -10.0, 0.60},
    {-21.5, -12.0, 0.55}, {-9.0, -12.0, 0.50}, {8.5, 3.0, 0.45}
};

class ParticleFilter {
public:
    ParticleFilter(int num_particles) : num_particles_(num_particles) {
        particles_.resize(num_particles_);
        std::default_random_engine gen;
        std::uniform_real_distribution<double> x_dist(-20.0, 20.0);
        std::uniform_real_distribution<double> y_dist(-13.0, 13.0);
        std::uniform_real_distribution<double> theta_dist(-M_PI, M_PI);

        for (auto& p : particles_) {
            p.x = x_dist(gen);
            p.y = y_dist(gen);
            p.theta = theta_dist(gen);
            p.weight = 1.0;
        }
    }

    void motionUpdate(double v, double w, double dt) {
        std::default_random_engine gen;
        std::normal_distribution<double> noise(0.0, 0.05);

        for (auto& p : particles_) {
            double v_hat = v + noise(gen);
            double w_hat = w + noise(gen);
            double gamma = noise(gen);

            if (fabs(w_hat) > 1e-5) {
                p.x += -v_hat / w_hat * sin(p.theta) + v_hat / w_hat * sin(p.theta + w_hat * dt);
                p.y +=  v_hat / w_hat * cos(p.theta) - v_hat / w_hat * cos(p.theta + w_hat * dt);
            } else {
                p.x += v_hat * dt * cos(p.theta);
                p.y += v_hat * dt * sin(p.theta);
            }
            p.theta += w_hat * dt + gamma * dt;
        }
    }

    void sensorUpdate(const sensor_msgs::LaserScan& scan) {
        for (auto& p : particles_) {
            double total_weight = 1.0;
            for (size_t i = 0; i < scan.intensities.size(); ++i) {
                if (scan.intensities[i] >= 0.4) {
                    double refl = scan.intensities[i];
                    auto it = std::find_if(landmarks.begin(), landmarks.end(), [=](const Landmark& lm) {
                        return fabs(lm.reflectivity - refl) < 0.01;
                    });
                    if (it == landmarks.end()) continue;

                    double z = scan.ranges[i];
                    double angle = scan.angle_min + i * scan.angle_increment;

                    double lx = it->x, ly = it->y;
                    double dx = lx - p.x;
                    double dy = ly - p.y;
                    double r_hat = sqrt(dx*dx + dy*dy);
                    double phi_hat = atan2(dy, dx) - p.theta;

                    double r_err = z - r_hat;
                    double phi_err = angle - phi_hat;

                    double r_sigma = 0.5, phi_sigma = 0.1;
                    double weight = exp(-0.5 * (r_err*r_err / (r_sigma*r_sigma) + phi_err*phi_err / (phi_sigma*phi_sigma)));
                    total_weight *= weight;
                }
            }
            p.weight = total_weight;
        }

        normalizeWeights();
        resample();
    }

    Particle getBestEstimate() {
        return *std::max_element(particles_.begin(), particles_.end(),
                                 [](const Particle& a, const Particle& b) {
                                     return a.weight < b.weight;
                                 });
    }

private:
    void normalizeWeights() {
        double sum = 0.0;
        for (auto& p : particles_) sum += p.weight;
        for (auto& p : particles_) p.weight /= (sum + 1e-9);
    }

    void resample() {
        std::vector<Particle> new_particles;
        std::default_random_engine gen;
        std::discrete_distribution<> dist;
        std::vector<double> weights;
        for (const auto& p : particles_) weights.push_back(p.weight);
        dist = std::discrete_distribution<>(weights.begin(), weights.end());

        for (int i = 0; i < num_particles_; ++i) {
            new_particles.push_back(particles_[dist(gen)]);
        }
        particles_ = new_particles;
    }

    int num_particles_;
    std::vector<Particle> particles_;
};

// Global
ros::Publisher pose_pub;
ParticleFilter* pf = nullptr;
geometry_msgs::Twist last_cmd_vel;
ros::Time last_cmd_time;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!pf || last_cmd_time.isZero()) return;
    ros::Duration dt = msg->header.stamp - last_cmd_time;
    pf->motionUpdate(last_cmd_vel.linear.x, last_cmd_vel.angular.z, dt.toSec());
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    last_cmd_vel = *msg;
    last_cmd_time = ros::Time::now();
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (!pf) return;
    pf->sensorUpdate(*msg);

    Particle best = pf->getBestEstimate();
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = best.x;
    pose.pose.position.y = best.y;
    pose.pose.orientation.z = sin(best.theta / 2);
    pose.pose.orientation.w = cos(best.theta / 2);
    pose_pub.publish(pose);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "particle_filter_node");
    ros::NodeHandle nh;

    int num_particles = 200;
    nh.getParam("num_particles", num_particles);
    pf = new ParticleFilter(num_particles);

    ros::Subscriber odom_sub = nh.subscribe("/husky_velocity_controller/odom", 1, odomCallback);
    ros::Subscriber scan_sub = nh.subscribe("/front/scan", 1, scanCallback);
    ros::Subscriber vel_sub  = nh.subscribe("/husky_velocity_controller/cmd_vel", 1, cmdVelCallback);

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/particle_pose", 1);

    ros::spin();
    return 0;
}
