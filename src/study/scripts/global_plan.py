#!/usr/bin/env python3
import time

import carla
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from time import sleep
from visualization_msgs.msg import MarkerArray
from carla import Location
from std_msgs.msg import String,Float64
import tf
import numpy as np
jiao=None
pos=None
aim_pos=None
def get_spawn_points(client):
    world = client.get_world()
    spawn_points = world.get_map().get_spawn_points()
    return spawn_points

def plan_route(client, origin, destination):
    from agents.navigation.global_route_planner import GlobalRoutePlanner
    world = client.get_world()
    carla_map = world.get_map()
    grp = GlobalRoutePlanner(carla_map, sampling_resolution=2.0)
    route = grp.trace_route(origin, destination)  # 返回路径点列表
    return route

def publish_path(route):
    path_pub = rospy.Publisher('/carla/global_path', Path, queue_size=10,latch=True)
    path_msg = Path()
    path_msg.header.frame_id = 'map'
    path_msg.header.stamp = rospy.Time.now()
    for waypoint in route:
        pose = PoseStamped()
        pose.pose.position.x = waypoint[0].transform.location.x
        pose.pose.position.y = -waypoint[0].transform.location.y
        pose.pose.position.z = waypoint[0].transform.location.z
        path_msg.poses.append(pose)
    path_pub.publish(path_msg)
def mark_callback(msg):
    global pos,jiao
    pos=msg.markers[0].pose.position
    pos=Location(x=pos.x, y=-pos.y, z=pos.z)
    quaternion = (
        msg.markers[0].pose.orientation.x,
        msg.markers[0].pose.orientation.y,
        msg.markers[0].pose.orientation.z,
        msg.markers[0].pose.orientation.w
    )
    jiao = tf.transformations.euler_from_quaternion(quaternion)[2]
def aim_pose_callback(msg):
    global aim_pos
    aim_pos=msg.pose.position
    aim_pos =Location(x=aim_pos.x, y=-aim_pos.y, z=aim_pos.z)
def main():
    rospy.init_node('global_plan', anonymous=True)
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    rospy.Subscriber("/carla/markers", MarkerArray, mark_callback)
    rospy.Subscriber("/move_base_simple/goal",PoseStamped,aim_pose_callback)
    while pos==None or aim_pos==None:
        pass
    while True:
        # 选择起点和终点
        origin = pos # 起点
        destination = aim_pos  # 终点
        # 规划路径
        route = plan_route(client, origin, destination)
        # 发布路径到ROS
        publish_path(route)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass