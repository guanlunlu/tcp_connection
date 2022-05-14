#!/usr/bin/python3

import rospy
import socket
import json

from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32

class tcp_client:
    def __init__(self):
        self.HOST = '192.168.50.195'
        self.PORT = 7001

        self.robot_pose = [0,0,0]
        self.start_running = 1
        self.pose_sub = rospy.Subscriber("ekf_pose", PoseWithCovarianceStamped, self.poseCallback)
        self.start_sub = rospy.Subscriber("start_running", Int32, self.startCallback)

        self.ally_pub = rospy.Publisher("tera_pose", PoseWithCovarianceStamped, queue_size=10)
        self.score_pub = rospy.Publisher("add_Point", Int32, queue_size= 10)
        self.clientInitialize()

    def clientInitialize(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.HOST, self.PORT)) 
        
        while True and not rospy.is_shutdown():
            # send current pose to ally
            send_data = []
            send_data = self.robot_pose.copy()
            send_data.append(self.start_running)
            send_data = str(send_data)
            print('send: ' + send_data)
            s.send(send_data.encode())

            # receive ally's current pose
            indata = s.recv(1024)
            if len(indata) == 0: # connection closed
                s.close()
                print('server closed connection.')
                break
            recv_data = indata.decode()
            print('recv: ' + recv_data)
            recv_data = json.loads(recv_data)
            self.allyPublish(recv_data)
            print("---")
            rospy.Rate(30).sleep()

    def poseCallback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        quat = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.robot_pose = [x, y, yaw]

    def startCallback(self, data):
        self.start_running = data.data

    def allyPublish(self, ally_pose):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose.position.x = ally_pose[0]
        msg.pose.pose.position.y = ally_pose[1]
        quaternion = quaternion_from_euler(0, 0, ally_pose[2])
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]
        self.ally_pub.publish(msg)

    def allyScorePublish(self, recv_data):
        msg = Int32()
        msg.data = recv_data[3]
        self.score_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("tcp_client", anonymous = True)
    tcp_client()
    rospy.spin()
