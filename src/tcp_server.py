#!/usr/bin/python3

import rospy
import numpy as np
import socket
import json

from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import CircleObstacle
from std_msgs.msg import Int32
from std_srvs.srv import Empty

class tcp_server:
    def __init__(self) -> None:
        self.HOST = '192.168.50.2'
        self.PORT = 7001
        self.HOST = rospy.get_param("tcp_host", '192.168.50.2')
        self.PORT = rospy.get_param("tcp_port", 7001)

        self.robot_pose = [0,0,0]
        self.score = 0
        self.if_tera_started = False
        self.obstacle_list = []
        self.ally_obstacle_list = []

        self.pose_sub = rospy.Subscriber("ekf_pose", PoseWithCovarianceStamped, self.poseCallback)
        self.score_sub = rospy.Subscriber("add_Point", Int32, self.scoreCallback)
        self.ally_pub = rospy.Publisher("pico_pose", PoseWithCovarianceStamped, queue_size=10)

        self.obs_sub = rospy.Subscriber("obstacles_to_map", Obstacles, self.obsCallback)
        self.ally_obs_pub = rospy.Publisher("ally_obstacle_to_map", Obstacles, queue_size=10)


        self.serverInitialize()

    def serverInitialize(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.HOST, self.PORT))
        s.listen(5)

        # print('server start at: %s:%s' % (self.HOST, self.PORT))
        # print('wait for connection...')
        rospy.loginfo("server start at: %s:%s", self.HOST, self.PORT)
        rospy.loginfo("wait for connection ...")

        while True and not rospy.is_shutdown():
            try:
                conn, addr = s.accept()
                rospy.loginfo('connected by ' + str(addr))
                self.stateReset()
            except KeyboardInterrupt:
                pass
            except socket.timeout:
                rospy.logwarn("Timeout")
                pass

            while True and not rospy.is_shutdown():
                # receive ally's current pose
                indata = conn.recv(1024)
                if len(indata) == 0: # connection closed
                    conn.close()
                    rospy.logerr_throttle(0.1, 'Lost connection from Client')
                    break

                # recv_data = indata.decode()
                # rospy.loginfo_throttle(0.1,'[Tera] recv: ' + recv_data)
                # recv_data = json.loads(recv_data)
                # self.allyPublish(recv_data)
                # self.triggerAlly(recv_data)
                self.messageDecode(indata)

                # send current pose to ally
                # send_data = []
                # send_data = self.robot_pose.copy()
                # send_data.append(self.score)
                # send_data = str(send_data)
                # rospy.loginfo_throttle(0.1,'[Tera] send: ' + send_data)
                # rospy.loginfo_throttle(0.1,"---")
                # conn.send(send_data.encode())
                conn.send(self.messageEncode())

    def messageEncode(self):
        send_data = []
        send_data = self.robot_pose.copy()
        send_data.append(self.score)

        for obs in self.obstacle_list:
            send_data.append(round(obs[0],4))
            send_data.append(round(obs[1],4))

        send_data = str(send_data)
        rospy.loginfo_throttle(0.1,'[Tera] send: ' + send_data)
        rospy.loginfo_throttle(0.1,"---")
        msg = send_data.encode()
        self.score = -1
        return msg

    def messageDecode(self, raw_data):
        recv_data = raw_data.decode()
        rospy.loginfo_throttle(0.1,'[Tera] recv: ' + recv_data)
        recv_data = json.loads(recv_data)
        if len(recv_data) != 0:
            self.allyPublish(recv_data)
            self.triggerAlly(recv_data)

            obs_list = []
            obs = []
            
            for i in range(len(recv_data)):
                if i > 3 and i%2 == 0:
                    obs_list.append([recv_data[i], recv_data[i+1]])

            self.ally_obstacle_list = obs_list.copy()
            self.allyObstaclePublish()
        
    def stateReset(self):
        self.if_tera_started = False

    def poseCallback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        quat = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.robot_pose = [round(x,4), round(y,4), round(yaw,4)]

    def scoreCallback(self, data):
        self.score = data.data

    def obsCallback(self, data):
        del self.obstacle_list[:]
        for obs in data.circles:
            x = obs.center.x
            y = obs.center.y
            self.obstacle_list.append([x, y])

    def allyPublish(self, ally_pose):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = ally_pose[0]
        msg.pose.pose.position.y = ally_pose[1]
        quaternion = quaternion_from_euler(0, 0, ally_pose[2])
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]
        self.ally_pub.publish(msg)

    def allyObstaclePublish(self):
        msg = Obstacles()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        for i in self.ally_obstacle_list:
            cir_obs = CircleObstacle()
            cir_obs.center.x = i[0]
            cir_obs.center.y = i[1]
            cir_obs.radius = 0.05
            cir_obs.true_radius = 0.05
            msg.circles.append(cir_obs)
        self.ally_obs_pub.publish(msg)
        del self.ally_obstacle_list[:]

    def triggerAlly(self, data):
        if data[3] == 1 and self.if_tera_started == False:
            rospy.wait_for_service('Tera_startRunning')
            try:
                srv = rospy.ServiceProxy("Tera_startRunning", Empty)
                srv()
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            self.if_tera_started = True
        else:
            pass

if __name__ == "__main__":
    rospy.init_node("tcp_server", anonymous = True, disable_signals=True)

    tcp_server()

    rospy.spin()
