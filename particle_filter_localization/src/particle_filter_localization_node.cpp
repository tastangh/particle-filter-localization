#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <random>
#include <vector>
#include <cmath>
#include <map>

struct Particle {
    double x, y, theta;
    double weight;
};

struct Landmark {
    double x, y;
    int id;
};

class ParticleFilter {
public:
    ParticleFilter(int num_particles)
        : num_particles_(num_particles), gen_(rd_()) {
        landmarks_ = {
            {1, {5.0, 5.0}},
            {2, {-5.0, 5.0}},
            {3, {-5.0, -5.0}},
            {4, {5.0, -5.0}}
        };
        initParticles();
    }

    void initParticles() {
        std::uniform_real_distribution<double> x_dist(-5.0, 5.0);
        std::uniform_real_distribution<double> y_dist(-5.0, 5.0);
        std::uniform_real_distribution<double> theta_dist(-M_PI, M_PI);
        particles_.clear();
        for (int i = 0; i < num_particles_; ++i) {
            particles_.push_back({x_dist(gen_), y_dist(gen_), theta_dist(gen_), 1.0});
        }
    }

    void motionUpdate(const nav_msgs::Odometry& odom_msg) {
        double dt = 0.1;
        double v = odom_msg.twist.twist.linear.x;
        double w = odom_msg.twist.twist.angular.z;

        double a1 = 0.01, a2 = 0.01, a3 = 0.01, a4 = 0.01, a5 = 0.01, a6 = 0.01;

        for (auto& p : particles_) {
            double v_hat = v + sample(a1 * v * v + a2 * w * w);
            double w_hat = w + sample(a3 * v * v + a4 * w * w);
            double gamma_hat = sample(a5 * v * v + a6 * w * w);

            if (std::fabs(w_hat) > 1e-5) {
                p.x += -v_hat / w_hat * sin(p.theta) + v_hat / w_hat * sin(p.theta + w_hat * dt);
                p.y += v_hat / w_hat * cos(p.theta) - v_hat / w_hat * cos(p.theta + w_hat * dt);
            } else {
                p.x += v_hat * dt * cos(p.theta);
                p.y += v_hat * dt * sin(p.theta);
            }
            p.theta += w_hat * dt + gamma_hat * dt;
        }
    }

    void sensorUpdate(const sensor_msgs::LaserScan& scan_msg) {
        double sigma_r = 0.5;
        double sigma_phi = 0.5;
        double sigma_s = 0.1;

        for (auto& p : particles_) {
            double total_weight = 1.0;
            for (size_t i = 0; i < scan_msg.ranges.size(); ++i) {
                double r = scan_msg.ranges[i];
                double phi = scan_msg.angle_min + i * scan_msg.angle_increment;
                int s = 1; // dummy reflectance
                int c = 1; // known correspondence: always detect landmark 1

                Landmark lm = landmarks_[c];
                double dx = lm.x - p.x;
                double dy = lm.y - p.y;
                double r_hat = std::sqrt(dx * dx + dy * dy);
                double phi_hat = std::atan2(dy, dx) - p.theta;

                double w_r = gaussian(r - r_hat, sigma_r);
                double w_phi = gaussian(phi - phi_hat, sigma_phi);
                double w_s = 1.0; // always correct correspondence

                total_weight *= w_r * w_phi * w_s;
            }
            p.weight = total_weight;
        }
        resample();
    }

    void resample() {
        std::vector<Particle> new_particles;
        std::vector<double> weights;
        for (const auto& p : particles_) weights.push_back(p.weight);

        std::discrete_distribution<> dist(weights.begin(), weights.end());
        for (int i = 0; i < num_particles_; ++i) {
            new_particles.push_back(particles_[dist(gen_)]);
        }
        particles_ = new_particles;
    }

    Particle getBestEstimate() {
        Particle best = particles_[0];
        for (const auto& p : particles_) {
            if (p.weight > best.weight) best = p;
        }
        return best;
    }

private:
    double sample(double stddev) {
        std::normal_distribution<double> dist(0.0, std::sqrt(stddev));
        return dist(gen_);
    }

    double gaussian(double mu, double sigma) {
        return exp(-mu * mu / (2 * sigma * sigma)) / (sigma * sqrt(2 * M_PI));
    }

    int num_particles_;
    std::vector<Particle> particles_;
    std::map<int, Landmark> landmarks_;
    std::random_device rd_;
    std::default_random_engine gen_;
};

ParticleFilter* pf_ptr = nullptr;
ros::Publisher pose_pub;
tf::TransformBroadcaster* tf_broadcaster_ptr = nullptr;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (pf_ptr) pf_ptr->motionUpdate(*msg);
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
        pose_msg.pose.orientation.z = sin(est.theta / 2.0);
        pose_msg.pose.orientation.w = cos(est.theta / 2.0);

        pose_pub.publish(pose_msg);

        tf::Transform tf;
        tf.setOrigin(tf::Vector3(est.x, est.y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, est.theta);
        tf.setRotation(q);
        tf_broadcaster_ptr->sendTransform(tf::StampedTransform(tf, msg->header.stamp, "map", "base_link"));
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
