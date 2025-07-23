#!/usr/bin/env python3  
import rospy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf
import tf.transformations
import math

class OdomPublisher:
    def __init__(self):
        rospy.init_node('fake_odometry_publisher')

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.last_time = rospy.Time.now()

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        self.rate = rospy.Rate(10.0)

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z

    def update(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()

            delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
            delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
            delta_th = self.vth * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

            # Publish the transform over TF
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.0),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            # Publish the odometry message
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = odom_quat[0]
            odom.pose.pose.orientation.y = odom_quat[1]
            odom.pose.pose.orientation.z = odom_quat[2]
            odom.pose.pose.orientation.w = odom_quat[3]

            odom.twist.twist.linear.x = self.vx
            odom.twist.twist.linear.y = self.vy
            odom.twist.twist.angular.z = self.vth

            self.odom_pub.publish(odom)
            self.last_time = current_time
            self.rate.sleep()

if __name__ == '__main__':
    try:
        odom_node = OdomPublisher()
        odom_node.update()
    except rospy.ROSInterruptException:
        pass

