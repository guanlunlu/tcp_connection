#!/usr/bin/python3

import rospy
import socket, pickle
import json

from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import CircleObstacle
from std_msgs.msg import Int32

class tcp_client:
    def __init__(self):
        self.HOST = '192.168.50.2'
        self.PORT = 7001
        self.HOST = rospy.get_param("~tcp_host", '192.168.50.195')
        self.PORT = rospy.get_param("~tcp_port", 7001)
        rospy.logwarn("[TCP CLIENT] Parameters set, Host on %s:%s", self.HOST, self.PORT)

        self.robot_pose = [0,0,0]
        self.start_running = 0
        
        self.pose_sub = rospy.Subscriber("ekf_pose", PoseWithCovarianceStamped, self.poseCallback)
        self.start_sub = rospy.Subscriber("start_running", Int32, self.startCallback)

        self.ally_pub = rospy.Publisher("tera_pose", PoseWithCovarianceStamped, queue_size=10)
        self.score_pub = rospy.Publisher("tera_add_Point", Int32, queue_size= 10)

        self.obstacle_list = []
        self.ally_obstacle_list = []
        self.obs_sub = rospy.Subscriber("obstacles_to_map", Obstacles, self.obsCallback)
        self.ally_obs_pub = rospy.Publisher("ally_obstacle_to_map", Obstacles, queue_size=10)

        self.clientInitialize()

    def clientInitialize(self):
        while True and not rospy.is_shutdown():
            socket_ = self.initConnection(self.HOST, self.PORT)
            try:
                while True and not rospy.is_shutdown():
                    # send current pose to ally
                    # send_data = []
                    # send_data = self.robot_pose.copy()
                    # send_data.append(self.start_running)
                    # send_data = str(send_data)
                    # socket_.send(send_data.encode())
                    # rospy.loginfo_throttle(0.1, '[Pico] send: ' + send_data)
                    socket_.send(self.messageEncode())

                    # receive ally's current pose
                    indata = socket_.recv(1024)
                    if len(indata) == 0: # connection closed
                        socket_.close()
                        print('server closed connection.')
                        break
                    
                    # recv_data = indata.decode()
                    # rospy.loginfo_throttle(0.1, '[Pico] recv: ' + recv_data)
                    # recv_data = json.loads(recv_data)
                    # self.allyPosePublish(recv_data)
                    # self.allyScorePublish(recv_data)

                    # rospy.loginfo_throttle(0.1, "---")
                    self.messageDecode(indata)
                    rospy.Rate(10).sleep()

            except socket.error:
                rospy.logwarn("No connection from Server [ %s:%s ], Reconnecting ...", self.HOST, self.PORT)
                rospy.sleep(0.5)
                self.initConnection

            except KeyboardInterrupt:
                break

    def messageEncode(self):
        send_data = []
        send_data = self.robot_pose.copy()
        send_data.append(self.start_running)

        for obs in self.obstacle_list:
            send_data.append(round(obs[0],4))
            send_data.append(round(obs[1],4))

        # send_data = str(send_data)
        rospy.loginfo_throttle(0.1,'[Pico] send: %s', send_data)
        rospy.loginfo_throttle(0.1,"---")
        # msg = send_data.encode()
        msg = pickle.dumps(send_data)
        return msg

    def messageDecode(self, raw_data):
        # recv_data = raw_data.decode()
        recv_data = pickle.loads(raw_data)
        rospy.loginfo_throttle(0.1,'[Pico] recv: %s', recv_data)
        # recv_data = json.loads(recv_data)
        if len(recv_data) != 0:
            self.allyPosePublish(recv_data)
            self.allyScorePublish(recv_data)

            obs_list = []
            obs = []
            for i in range(len(recv_data)):
                if i > 3 and i%2 == 0:
                    obs_list.append([recv_data[i], recv_data[i+1]])

            self.ally_obstacle_list = obs_list.copy()
            self.allyObstaclePublish()
        
    def initConnection(self, host, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try :
            sock.connect((host,port))
        except :
            pass
        return sock

    def obsCallback(self, data):
        del self.obstacle_list[:]
        for obs in data.circles:
            x = obs.center.x
            y = obs.center.y
            self.obstacle_list.append([x, y])

    def poseCallback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        quat = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        self.robot_pose = [round(x,4), round(y,4), round(yaw,4)]

    def startCallback(self, data):
        self.start_running = data.data

    def allyPosePublish(self, ally_pose):
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

    def allyScorePublish(self, recv_data):
        if recv_data[3] != -1:
            msg = Int32()
            msg.data = recv_data[3]
            self.score_pub.publish(msg)

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
        

if __name__ == "__main__":
    rospy.init_node("tcp_client", anonymous = True, disable_signals=True)
    tcp_client()
    rospy.spin()
