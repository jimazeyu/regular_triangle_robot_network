#!/usr/bin/python
# coding=utf8

# ROS stage仿真

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# laser

from tf.transformations import euler_from_quaternion
import numpy as np
import math
from delaunay2D import Delaunay2D
from sensor_msgs.msg import LaserScan
# matplotlib用于debug
import matplotlib.pyplot as plt


# 机器人类，包含机器人的位置信息，以及机器人的运动控制
class robot:
    def __init__(self, name, x, y, theta):
        self.name = name
        self.x = x
        self.y = y
        self.theta = theta
        # 增量更新，因此记录上一次odom信息
        self.last_x = x
        self.last_y = y
        self.last_theta = theta
        self.pub = rospy.Publisher('/%s/cmd_vel' % name, Twist, queue_size=10)

    # 使用里程计信息更新机器人位姿
    def update_with_odom(self, data):
        # 计算位置增量
        delta_x = data.pose.pose.position.x - self.last_x
        delta_y = data.pose.pose.position.y - self.last_y
        # 计算角度增量
        theta_now = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                             data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]
        delta_theta = theta_now - self.last_theta
        # 更新位姿
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        # 更新上一时刻位姿
        self.last_x = data.pose.pose.position.x
        self.last_y = data.pose.pose.position.y
        self.last_theta = theta_now

    # 使用后端信息更新机器人位姿
    def update_with_optimization(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    # 发布控制命令
    def move(self, vx, vy, vtheta):
        move_cmd = Twist()
        move_cmd.linear.x = vx
        move_cmd.linear.y = vy
        move_cmd.angular.z = vtheta
        self.pub.publish(move_cmd)
    

# 工具类，订阅真实位姿，计算虚拟位姿，以及相互测距结果（矩阵形式，1代表可以相互测距,0代表机器人间没有信号）
class msg_generator:
    pass

# 工具类，对机器人集群进行全局优化
class global_optimization:
    pass

# 几何工具包
class geometry_tool:
    # 计算三角形重心
    def triangle_center(self, p1, p2, p3):
        return (p1+p2+p3)/3
    
    # 计算两个向量的夹角
    def angle_between(self, v1, v2):
        v1_u = v1 / np.linalg.norm(v1)
        v2_u = v2 / np.linalg.norm(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    # 计算delaunay三角形
    def delaunay(self, point_list):
        dt = Delaunay2D()
        # Insert all seeds one by one
        for point in point_list:
            dt.addPoint(point)
        return dt.exportTriangles()


# 机器人集群类，包含多个机器人，以及机器人间的相互位置信息
class robot_group:
    def __init__(self, robot_list):
        self.robot_list = robot_list
        self.robot_num = len(robot_list)
        # 机器人下一次的速度控制量
        self.robot_vx = np.zeros(self.robot_num)
        self.robot_vy = np.zeros(self.robot_num)
        self.robot_vtheta = np.zeros(self.robot_num)
        # 二维数组，用于记录相互测距结果
        self.robot_dist = np.zeros((self.robot_num, self.robot_num))
        # 是否可视相互作用
        self.if_visualization = False
        # 参数
        self.dist_min = 0.5
        self.dist_max = 2.0
        self.max_v = 0.2
        self.laser_max = 2.0

    # 距离墙体的距离系数，距离越近，系数越大
    def laser_force(self, dist):
        return 1/(dist+0.1)

    # 机器人相互距离系数，小于dist_min时，越近越排斥，大于dist_max时，越远越吸引，中间时，距离越近，系数越大
    def robot_force(self, dist, dist_min, dist_max):
        if dist < dist_min:
            return 1/(dist+0.1)
        elif dist > dist_max:
            return -dist
        else:
            return 1/(dist+0.1)
    

    # 用odom信息更新机器人位置
    def update_robots_with_odom(self):
        for robot in self.robot_list:
            data = rospy.wait_for_message('/%s/odom' % robot.name, Odometry)
            robot.update_with_odom(data)

    # 用全局测距信息更新机器人位置
    def update_robots_with_optimization(self):
        # 更新全局位置，并加入噪声
        for robot in self.robot_list:
            data = rospy.wait_for_message('/%s/base_pose_ground_truth' % robot.name, Odometry)
            x = data.pose.pose.position.x+np.random.normal(0, 0.0)
            y = data.pose.pose.position.y+np.random.normal(0, 0.0)
            theta = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                                    data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]
            robot.update_with_optimization(x,y,theta)

    # 更新全局相互测距结果
    def update_robot_dist(self):
        for i in range(self.robot_num):
            for j in range(self.robot_num):
                if i != j:
                    distance = math.sqrt((self.robot_list[i].x-self.robot_list[j].x)**2+(self.robot_list[i].y-self.robot_list[j].y)**2)
                    if distance < 1.5 and distance > 0:
                        self.robot_dist[i][j] = distance
                    else:
                        self.robot_dist[i][j] = -1
        

    # 控制所有机器人运动
    def controll_all_robots(self):
        # 按照机器人顺序加入机器人坐标，并生成delaunay三角形
        point_list = []
        for robot in self.robot_list:
            point_list.append([robot.x, robot.y])
        dt = geometry_tool().delaunay(point_list)

        # 三个方向（旋转方向，伸缩方向，墙推方向）
        # 机器人分解力数组，里面存储四元组（作用机器人，力类型，力方向，力大小）
        robot_component_forces = []

        # 根据delaunay三角形，计算每个机器人的速度控制量
        for triangle in dt:
            # 计算三角形重心以重心到顶点的向量为基础，加入距离墙体的影响，加入距离其他机器人的影响
            center = geometry_tool().triangle_center(np.array(point_list[triangle[0]]), np.array(point_list[triangle[1]]), np.array(point_list[triangle[2]]))
            for i in range(3):
                # 计算重心到顶点的向量
                vector = np.array(point_list[triangle[i]])-center
                # 根据距离计算系数
                robot_force =  self.robot_force(math.sqrt(vector[0]**2+vector[1]**2), self.dist_min, self.dist_max)
                # vector归一化
                vector = vector/math.sqrt(vector[0]**2+vector[1]**2)
                # 添加伸缩力
                robot_component_forces.append([triangle[i], "stretch", vector, robot_force])


                # 计算速度(先归一化，再乘以系数)
                # v = vector*robot_force
                # self.robot_vx[triangle[i]] += v[0]
                # self.robot_vy[triangle[i]] += v[1]
                # self.robot_vtheta[triangle[i]] += 0

                # 计算和另外两条中线的夹角，沿着vector向量的法线往夹角大的方向运动
                vector1 = np.array(point_list[triangle[(i+1)%3]])-center
                vector2 = np.array(point_list[triangle[(i+2)%3]])-center
                vector1 = vector1/math.sqrt(vector1[0]**2+vector1[1]**2)
                vector2 = vector2/math.sqrt(vector2[0]**2+vector2[1]**2)
                angle1 = geometry_tool().angle_between(vector, vector1)
                angle2 = geometry_tool().angle_between(vector, vector2)
                # 根据夹角计算系数
                robot_angle_force = math.fabs(angle1-angle2)/math.pi*2.0
                # 添加旋转分力
                if angle1 > angle2:
                    robot_component_forces.append([triangle[i], "rotate", np.array([-vector[1], vector[0]]), robot_angle_force])
                else:
                    robot_component_forces.append([triangle[i], "rotate", np.array([vector[1], -vector[0]]), robot_angle_force])
        
        # 订阅scan，并根据laser数据计算墙的推力
        for i in range(self.robot_num):
            # 订阅scan
            scan = rospy.wait_for_message("/robot_"+str(i)+"/base_scan", LaserScan)
            # 计算每个激光点的坐标
            laser_points = []
            for j in range(len(scan.ranges)):
                laser_points.append([scan.ranges[j]*math.cos(scan.angle_min+j*scan.angle_increment), scan.ranges[j]*math.sin(scan.angle_min+j*scan.angle_increment)])
            #合成推力
            composed_force = [0, 0]
            for laser_point in laser_points:
                # 计算每个激光点的推力
                dist = math.sqrt(laser_point[0]**2+laser_point[1]**2)
                if(dist > self.laser_max):
                    continue
                robot_force =  self.laser_force(dist)
                robot_force_vector = np.array(laser_point)/dist
                # 合成推力
                composed_force[0] += -robot_force_vector[0]*robot_force
                composed_force[1] += -robot_force_vector[1]*robot_force
            robot_component_forces.append([i, "push", composed_force, robot_force*0.3])


        # 计算合成速度方向
        for i in range(self.robot_num):
            # 计算机器人i的合成速度
            vx = 0
            vy = 0
            for force in robot_component_forces:
                if force[0] == i:
                    vx += force[2][0]*force[3]
                    vy += force[2][1]*force[3]
            # 计算机器人i的合成速度
            self.robot_vx[i] = vx
            self.robot_vy[i] = vy
            self.robot_vtheta[i] = 0
        # 速度归一化
        for i in range(self.robot_num):
            v = math.sqrt(self.robot_vx[i]**2+self.robot_vy[i]**2)
            if v > self.max_v:
                self.robot_vx[i] = self.robot_vx[i]/v*self.max_v
                self.robot_vy[i] = self.robot_vy[i]/v*self.max_v
                self.robot_vtheta[i] = 0
            
        # 根据合成速度发布控制命令
        for i in range(self.robot_num):
            self.robot_list[i].move(self.robot_vx[i], self.robot_vy[i], self.robot_vtheta[i])

        if self.if_visualization:
            # 绘制机器人位置(黑色)
            plt.clf()
            plt.xlim(-10, 10)
            plt.ylim(-10, 10)
            for i in range(self.robot_num):
                plt.plot(self.robot_list[i].x, self.robot_list[i].y, 'ko')
            # 绘制三角形(黄色)
            for triangle in dt:
                plt.plot([point_list[triangle[0]][0], point_list[triangle[1]][0], point_list[triangle[2]][0], point_list[triangle[0]][0]], [point_list[triangle[0]][1], point_list[triangle[1]][1], point_list[triangle[2]][1], point_list[triangle[0]][1]], 'y-')
            # 绘制分解力（带箭头）
            for robot_component_force in robot_component_forces:
                # 伸缩力（红色）
                if robot_component_force[1] == "stretch":
                    plt.arrow(point_list[robot_component_force[0]][0], point_list[robot_component_force[0]][1], robot_component_force[3]*robot_component_force[2][0], robot_component_force[3]*robot_component_force[2][1], head_width=0.2, head_length=0.2, fc='r', ec='r')
                # 绘制旋转力（蓝色）
                elif robot_component_force[1] == "rotate":
                    plt.arrow(point_list[robot_component_force[0]][0], point_list[robot_component_force[0]][1], robot_component_force[3]*robot_component_force[2][0], robot_component_force[3]*robot_component_force[2][1], head_width=0.2, head_length=0.2, fc='b', ec='b')
                # 绘制墙推力（绿色）
                else:
                    plt.arrow(point_list[robot_component_force[0]][0], point_list[robot_component_force[0]][1], robot_component_force[3]*robot_component_force[2][0], robot_component_force[3]*robot_component_force[2][1], head_width=0.2, head_length=0.2, fc='g', ec='g')
            # 绘制合成速度（带箭头，紫色）
            for i in range(self.robot_num):
                plt.arrow(self.robot_list[i].x, self.robot_list[i].y, self.robot_vx[i], self.robot_vy[i], head_width=0.2, head_length=0.2, fc='m', ec='m')

            plt.show()            


    
# main
if __name__ == '__main__':
    rospy.init_node('robot_network_controller', anonymous=True)
    robot_list = []
    robot_num = 9
    for i in range(robot_num):
        robot_list.append(robot('robot_'+str(i), 0, 0, 0))
    robots = robot_group(robot_list)
    # 更新机器人位置
    robots.update_robots_with_optimization()
    rate = rospy.Rate(10)
    step = 0 # 用于判断是否进行全局优化
    while not rospy.is_shutdown():
        #robots.update_robots_with_odom()
        robots.update_robots_with_optimization()
        robots.update_robot_dist()
        step += 1
        if step == 10:
            #robots.update_robots_with_optimization()
            step = 0
        robots.controll_all_robots()
        rate.sleep()

