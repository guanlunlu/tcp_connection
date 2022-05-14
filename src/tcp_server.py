#!/usr/bin/python3

import rospy
import socket
import json

from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32
from std_srvs.srv import Empty

class tcp_server:
    def __init__(self) -> None:
        self.HOST = '192.168.50.195'
        self.PORT = 7001

        self.robot_pose = [0,0,0]
        self.score = 0
        self.pose_sub = rospy.Subscriber("ekf_pose", PoseWithCovarianceStamped, self.poseCallback)
        self.score_sub = rospy.Subscriber("add_Point", Int32, self.scoreCallback)
        self.if_tera_started = False

        self.ally_pub = rospy.Publisher("pico_pose", PoseWithCovarianceStamped, queue_size=10)
        self.serverInitialize()

    def serverInitialize(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.HOST, self.PORT))
        s.listen(5)

        print('server start at: %s:%s' % (self.HOST, self.PORT))
        print('wait for connection...')

        while True and not rospy.is_shutdown():
            conn, addr = s.accept()
            rospy.loginfo('connected by ' + str(addr))

            while True and not rospy.is_shutdown():
                # receive ally's current pose
                indata = conn.recv(1024)
                if len(indata) == 0: # connection closed
                    conn.close()
                    rospy.loginfo('client closed connection.')
                    break
                recv_data = indata.decode()
                print('recv: ' + recv_data)
                recv_data = json.loads(recv_data)
                self.allyPublish(recv_data)
                self.triggerAlly(recv_data)

                # send current pose to ally
                send_data = []
                send_data = self.robot_pose.copy()
                send_data.append(self.score)
                send_data = str(send_data)
                print('send: ' + send_data)
                print("---")
                conn.send(send_data.encode())

    def poseCallback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        quat = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.robot_pose = [x, y, yaw]

    def scoreCallback(self, data):
        self.score = data.data

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

    def triggerAlly(self, data):
        if data[3] == 1 and self.if_tera_started == False:
            rospy.ServiceProxy("Tera_startRunning", Empty)
            self.if_tera_started = True
        else:
            pass

if __name__ == "__main__":
    rospy.init_node("tcp_server", anonymous = True)

    tcp_server()

    rospy.spin()
