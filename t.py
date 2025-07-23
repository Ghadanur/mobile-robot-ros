#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/ultrasonic_distance', Float32, self.distance_callback)

        self.safe_distance = 20.0  # cm
        self.obstacle_detected = False
        self.state = 'GO_FORWARD'

        self.forward_start_time = None
        self.forward_duration = 2.0  # seconds(rviz)
        #self.forward_duration = 1.0  #seconds(physical)

        self.turn_start_time = None
        self.turn_duration = 3.1  # seconds(rviz)
        #self.turn_duration = 0.4  # seconds(physical)

        self.loop_count = 0  # Count completed sides of square (max 4)
        self.rate = rospy.Rate(10)  # 10 Hz

    def distance_callback(self, msg):
        distance = msg.data
        rospy.loginfo(f"Ultrasonic Distance: {distance:.2f} cm")
        self.obstacle_detected = distance < self.safe_distance

    def go_forward(self):
        move_cmd = Twist()
        move_cmd.linear.x = -0.7  # Adjust speed
        move_cmd.angular.z = 0.0
        self.cmd_pub.publish(move_cmd)

    def turn(self):
        turn_cmd = Twist()
        turn_cmd.linear.x = 0.0
        turn_cmd.angular.z = 0.5  # Adjust turn speed
        self.cmd_pub.publish(turn_cmd)

    def stop(self):
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)

    def run(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            if self.state == 'GO_FORWARD':
                if self.obstacle_detected:
                    rospy.logwarn("Obstacle detected! Restarting square path.")
                    self.stop()
                    rospy.sleep(0.5)
                    self.state = 'TURN'
                    self.turn_start_time = rospy.Time.now()
                    self.loop_count = -1  # Reset loop count to restart square
                    self.forward_start_time = None
                else:
                    if self.forward_start_time is None:
                        self.forward_start_time = now
                    if (now - self.forward_start_time).to_sec() < self.forward_duration:
                        self.go_forward()
                    else:
                        rospy.loginfo("Completed forward duration.")
                        self.forward_start_time = None
                        self.state = 'TURN'
                        self.turn_start_time = now

            elif self.state == 'TURN':
                if self.turn_start_time is None:
                    self.turn_start_time = now
                if (now - self.turn_start_time).to_sec() < self.turn_duration:
                    self.turn()
                else:
                    rospy.loginfo("Completed turn.")
                    self.turn_start_time = None
                    self.loop_count += 1
                    rospy.loginfo(f"Completed sides: {self.loop_count}/4")
                    self.state = 'GO_FORWARD'

            if self.loop_count >= 4:
                rospy.loginfo("Square path completed. Stopping robot.")
                self.stop()
                break

            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot = ObstacleAvoidance()
        robot.run()
    except rospy.ROSInterruptException:
        pass

