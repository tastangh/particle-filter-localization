#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt

odom_positions = []
particle_positions = []

def odom_callback(msg):
    pos = msg.pose.pose.position
    odom_positions.append((pos.x, pos.y))

def particle_callback(msg):
    pos = msg.pose.position
    particle_positions.append((pos.x, pos.y))

if __name__ == "__main__":
    rospy.init_node("pose_comparator", anonymous=True)

    rospy.Subscriber("/husky_velocity_controller/odom", Odometry, odom_callback)
    rospy.Subscriber("/particle_pose", PoseStamped, particle_callback)

    rospy.loginfo("Veri toplanıyor... (10 saniye)")
    rospy.sleep(10)

    rospy.loginfo("Çizim başlatılıyor...")

    odom_xs, odom_ys = zip(*odom_positions) if odom_positions else ([], [])
    particle_xs, particle_ys = zip(*particle_positions) if particle_positions else ([], [])

    plt.figure(figsize=(10, 6))
    plt.plot(odom_xs, odom_ys, label='Gerçek Odom', color='blue')
    plt.plot(particle_xs, particle_ys, label='Tahmin /particle_pose', color='green', linestyle='--')
    plt.title("Gerçek Pozisyon vs. Particle Filter Tahmini")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()
