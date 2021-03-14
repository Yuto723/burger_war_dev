#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This is ALL SENSOR use node.
Mainly echo sensor value in tarminal.
Please Use for your script base.

by Takuya Yamaguchi @dashimaki360
'''

import rospy
import random
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2






class MyRunBot(object):
    def __init__(self, 
                 use_lidar=False, use_camera=False, use_imu=False,
                 use_odom=False, use_joint_states=False):

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # lidar scan subscriber
        if use_lidar:
            self.scan = LaserScan()
            self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # camera subscribver
        # please uncoment out if you use camera
        if use_camera:
            # for convert image topic to opencv obj
            cols = 640
            rows = 480
            self.img = np.full((rows, cols, 3), 0, dtype=np.uint8)
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
            self.mom_red_x = 0
            self.mom_red_y = 0
            self.mom_yel_x = 0
            self.mom_yel_y = 0
            self.mom_gre_x = 0
            self.mom_gre_y = 0
            self.mom_blu_x = 0
            self.mom_blu_y = 0
            self.speed = 0.2
            self.state = "go"
            self.back_start_time = 0
            self.blue_count = 0
            self.target_flag = False
        # imu subscriber
        if use_imu:
            self.imu_sub = rospy.Subscriber('imu', Imu, self.imuCallback)

        # odom subscriber
        if use_odom:
            self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)

        # joint_states subscriber
        if use_joint_states:
            self.odom_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)

    def strategy(self):
        '''
        calc Twist and publish cmd_vel topic
        '''
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            twist = Twist()
            
            # 青真ん中を向くように方向転換
            diff_pix = self.mom_blu_x - 320
            if abs(diff_pix) < 320 and diff_pix > 20:
                anglular_z = -0.3
            elif abs(diff_pix) < 320 and diff_pix < -20:
                anglular_z = 0.3
            else:
                anglular_z = 0.0
            #最初の青は直進で必ず取りに行く            
            if self.blue_count == 0:
                self.state = "go"
            else:
                # 緑真ん中を向くように方向転換
                diff_pix = self.mom_gre_x - 320
                if abs(diff_pix) < 320 and diff_pix > 20:
                    anglular_z = -0.3
                elif abs(diff_pix) < 320 and diff_pix < -20:
                    anglular_z = 0.3
                else:
                    anglular_z = 0.0

                # 黄色反対を向くように方向転換
                # 近い(重心が下よりの場合のみ)
                # print ("yel.y:%f",self.mom_yel_y)
                if self.mom_yel_y > 370:
                    diff_pix = self.mom_yel_x - 320
                    if abs(diff_pix) < 320 and diff_pix > 0:
                        anglular_z = 0.5
                    elif abs(diff_pix) < 320 and diff_pix <= 0:
                        anglular_z = -0.5
                    else:
                        anglular_z = 0.0

                #黄色を目指して直進で奥を目指す
                elif self.mom_yel_y > 0 and self.target_flag == False:
                    diff_pix = self.mom_yel_x - 320
                    if abs(diff_pix) < 320 and diff_pix > 20:
                        anglular_z = -0.3
                    elif abs(diff_pix) < 320 and diff_pix < -20:
                        anglular_z = 0.3
                    else:
                        anglular_z = 0.0

            if self.state == "go":
                linear_x = self.speed
            elif self.state == "back":
                linear_x = -self.speed * 2
                anglular_z = 0
            elif self.state == "turnL":
                linear_x = 0
                anglular_z = 1.5
            elif self.state == "turnR":
                linear_x = 0
                anglular_z = -1.5

            print self.state

            twist.linear.x = linear_x; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = anglular_z

            # publish twist topic
            self.vel_pub.publish(twist)
            r.sleep()


    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        # rospy.loginfo(self.scan)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        cv2.imshow("Image window", self.img)
        cv2.waitKey(1)
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        lower_red = np.array([-40, 100, 0]) # red
        upper_red = np.array([20, 255, 255]) # red
        lower_yel = np.array([20, 150, 0]) # yellow
        upper_yel = np.array([40, 255, 255]) # yellow
        lower_blu = np.array([100, 150, 0]) # blue
        upper_blu = np.array([180, 255, 255]) # blue
        lower_gre = np.array([40, 80, 0]) # green
        upper_gre = np.array([80, 255, 255]) # green
        lower_gra = np.array([0, 0, 100]) #  gray
        upper_gra = np.array([255, 100, 255]) # gray
        self.target_flag = False #何かしらのターゲットを見つけたらTrue

        #moment      

        #red moment
        bin_img_red = cv2.inRange(hsv, lower_red, upper_red)
        #cv2.imshow("bin red window", bin_img_red)
          # 重心を求める
        mom = cv2.moments(bin_img_red)
        red_area_per = 0
        if "m00" in mom and "m10" in mom and "m01" in mom and mom["m00"] <> 0:
            self.mom_red_x = int(mom["m10"]/mom["m00"])
            self.mom_red_y = int(mom["m01"]/mom["m00"])
            # 面積が一定以上なら検出中
            whole_area = bin_img_red.size
            white_area = cv2.countNonZero(bin_img_red)
            red_area_per = 100 * white_area/whole_area
            if red_area_per > 1:
                self.target_flag = True
        else:
            self.mom_red_x = -1
            self.mom_red_y = -1
 
        #yellow moment
        bin_img_yel= cv2.inRange(hsv, lower_yel, upper_yel)
        #cv2.imshow("bin yel window", bin_img_yel)
          # 重心を求める
        mom = cv2.moments(bin_img_yel)
        if "m00" in mom and "m10" in mom and "m01" in mom and mom["m00"] <> 0:
            self.mom_yel_x = int(mom["m10"]/mom["m00"])
            self.mom_yel_y = int(mom["m01"]/mom["m00"])
        else:
            self.mom_yel_x = -1
            self.mom_yel_y = -1  
      
        #blue moment
        bin_img_blu = cv2.inRange(hsv, lower_blu, upper_blu)
        #cv2.imshow("bin blu window", bin_img_blu)
          # 重心を求める
        mom = cv2.moments(bin_img_blu)
        blu_area_per = 0
        if "m00" in mom and "m10" in mom and "m01" in mom and mom["m00"] <> 0:
            self.mom_blu_x = int(mom["m10"]/mom["m00"])
            self.mom_blu_y = int(mom["m01"]/mom["m00"])
            # 面積が一定以上なら検出中
            whole_area = bin_img_blu.size
            white_area = cv2.countNonZero(bin_img_blu)
            blu_area_per = 100 * white_area/whole_area
            # print("blue area%f",blu_area_per)
            if blu_area_per > 1:
                self.target_flag = True
            if blu_area_per > 4:
                self.blue_count += 1

        else:
            self.mom_blu_x = -1
            self.mom_blu_y = -1

        #green moment
        bin_img_gre = cv2.inRange(hsv, lower_gre, upper_gre)
        gre_area_per = 0
        #cv2.imshow("bin gre window", bin_img_gre)
          # 重心を求める
        mom = cv2.moments(bin_img_gre)
        if "m00" in mom and "m10" in mom and "m01" in mom and mom["m00"] <> 0:
            self.mom_gre_x = int(mom["m10"]/mom["m00"])
            self.mom_gre_y = int(mom["m01"]/mom["m00"])
          # 面積が一定以上なら検出中
            whole_area = bin_img_gre.size
            white_area = cv2.countNonZero(bin_img_gre)
            gre_area_per = 100 * white_area/whole_area
            # print("green area%f",gre_area_per)
            if gre_area_per > 1:
                self.target_flag = True
        else:
            self.mom_gre_x = -1
            self.mom_gre_y = -1

        #gray       
        bin_img_gra = cv2.inRange(hsv, lower_gra, upper_gra)
        whole_area = bin_img_gra.size
        white_area = cv2.countNonZero(bin_img_gra)
        gra_area_per = 100 * white_area/whole_area
        # print("gray area%f",gra_area_per)
        #cv2.imshow("bin gra window", bin_img_gra)


        #壁 or 的の面積が大きくなったらバック
        if gra_area_per > 80 or gra_area_per < 3 or blu_area_per > 8 or gre_area_per > 8:
            self.back_start_time = time.time()
        else:
            #スタック対策でランダムにバック
            xr_int = random.randint(0,10)             
            if (xr_int == 0 and time.time() - self.back_start_time > 10):
                self.back_start_time = time.time()

        # 移動状態
        if (time.time() - self.back_start_time < 4):
            self.state = "back"
        elif (time.time() - self.back_start_time < 5):
            if self.target_flag == True:
                self.state = "turnR"
            else:
                self.state = "turnL"
        elif (time.time() - self.back_start_time > 8):
            self.state = "go"      

        
        '''
          # 先程二値化した画像をマスク画像としてBGR画像を切り抜き
        processed_image = cv2.bitwise_and(self.img, self.img, mask=bin_img_gre)
          # 求めた重心の位置を示すために紫色の点を描画
        color = (255, 0, 255)
        processed_image = cv2.circle(processed_image, (self.mom_gre_x, self.mom_gre_y), 3, color, -1)
        cv2.imshow("Process Window", processed_image)
        '''

    # imu call back sample
    # update imu state

    def imuCallback(self, data):
        self.imu = data
        # rospy.loginfo(self.imu)

    # odom call back sample
    # update odometry state

    def odomCallback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        # rospy.loginfo("odom pose_x: {}".format(self.pose_x))
        # rospy.loginfo("odom pose_y: {}".format(self.pose_y))

    # jointstate call back sample
    # update joint state

    def jointstateCallback(self, data):
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]
        # rospy.loginfo("joint_state R: {}".format(self.wheel_rot_r))
        # rospy.loginfo("joint_state L: {}".format(self.wheel_rot_l))

if __name__ == '__main__':
    rospy.init_node('my_run')
    bot = MyRunBot(use_lidar=True, use_camera=True, use_imu=True,
                       use_odom=True, use_joint_states=True)
    bot.strategy()
