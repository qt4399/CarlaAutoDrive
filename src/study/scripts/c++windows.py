import rospkg
rp = rospkg.RosPack()
pkg_path = rp.get_path('study')
autoviz_path = f"{pkg_path}/c++/build"
import sys
sys.path.append(f"{autoviz_path}")
import autoviz
import cv2
import carla
import time
import math
import rospy
from sensor_msgs.msg import Image,Imu
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped,Point
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Float32
import numpy as np
import threading
viz = autoviz.Visualizer()
car = autoviz.Car()
radius = 200.0
center_x, center_y = 800, 600
x_start=0
y_start=0
pos = None
aim_pose=None
angle = None
color_img=None
global_path=[]
def on_reset():
    print(1)
def set_goal():
    aim_goal=PoseStamped()
    aim_goal.pose.position.x=viz.goal_x
    aim_goal.pose.position.y = viz.goal_y
    aim_goal.pose.position.z = 0
    goal_pub.publish(aim_goal)
def image_callback(img_msg):
    global color_img
    bridge = CvBridge()
    color_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
def pos_callback(pos_msgs):
    global pos,angle
    pos=[0,0,0]
    angle=0
    pos[0]=pos_msgs.pose.pose.position.x
    pos[1]=pos_msgs.pose.pose.position.y
    angle = math.atan2(2 * (
                pos_msgs.pose.pose.orientation.w * pos_msgs.pose.pose.orientation.z + pos_msgs.pose.pose.orientation.x * pos_msgs.pose.pose.orientation.y),
                       1 - 2 * (pos_msgs.pose.pose.orientation.y ** 2 + pos_msgs.pose.pose.orientation.z ** 2))
    angle = ((angle) / np.pi) * 180
def aim_pos_callback(msg):
    global aim_pose
    aim_pose=[msg.pose.position.x,msg.pose.position.y]
def path_callback(msg):
    global global_path
    global_path=[]
    last_x=0
    last_y=0
    jump_num=0
    if len(msg.poses)<30:
        for i in range(0,len(msg.poses)):
            if msg.poses[i].pose.position.x==last_x and msg.poses[i].pose.position.y==last_y:
                jump_num+=1
            else:
                viz.ego_car.set_global_x(i-jump_num,msg.poses[i].pose.position.x)
                viz.ego_car.set_global_y(i-jump_num,msg.poses[i].pose.position.y)
            last_x=msg.poses[i].pose.position.x
            last_y=msg.poses[i].pose.position.y
        viz.ego_car.global_path_num =len(msg.poses)-jump_num
    else:
        for i in range(0,30):
            if msg.poses[i].pose.position.x==last_x and msg.poses[i].pose.position.y==last_y:
                jump_num+=1
            else:
                viz.ego_car.set_global_x(i-jump_num,msg.poses[i].pose.position.x)
                viz.ego_car.set_global_y(i-jump_num,msg.poses[i].pose.position.y)
            last_x = msg.poses[i].pose.position.x
            last_y = msg.poses[i].pose.position.y
        viz.ego_car.global_path_num =30-jump_num
    publish_path_marker()
def speed_callback(msg):
    viz.ego_car.speed=msg.data
def refresh_hanshu():
    while True:
        viz.refresh()
        if not viz.handle_events():
            break
def calculate_map_bounds(carla_map):
        if not carla_map:
            return None, None, None, None
        waypoints = carla_map.generate_waypoints(5.0)  # 每5米生成一个路点

        min_x = float('inf')
        max_x = float('-inf')
        min_y = float('inf')
        max_y = float('-inf')
        for waypoint in waypoints:
            x = waypoint.transform.location.x
            y = waypoint.transform.location.y

            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)
        return min_x, max_x, min_y, max_y
def publish_path_marker():
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "path_marker"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 2  # 线宽为 0.1 米（根据需要调整）
    marker.scale.y = 0.0
    marker.scale.z = 0.0
    marker.color.r = 0.0  # 红色分量
    marker.color.g = 1.0  # 绿色分量
    marker.color.b = 0.0  # 蓝色分量
    marker.color.a = 0.2
    path_points = []
    for i in range(15):
            point = Point()
            point.x = viz.ego_car.get_predict_x()[i]
            point.y = viz.ego_car.get_predict_y()[i]
            point.z = 0.0
            path_points.append(point)
    marker.points = path_points
    marker_pub.publish(marker)
    control_cmd = CarlaEgoVehicleControl()
    if viz.ego_car.AutoDriveOn==True:
        if viz.ego_car.power>=0:
            control_cmd.throttle=viz.ego_car.power
            control_cmd.brake=0
        else:
            control_cmd.throttle = 0
            control_cmd.brake=-viz.ego_car.power
        control_cmd.steer=viz.ego_car.steer
    else:
        control_cmd.throttle = 0
        control_cmd.brake = 1
        control_cmd.steer = 0

    cmd_pub.publish(control_cmd)

rospy.init_node("c++windows")
rospy.Subscriber("carla_map_image", Image, image_callback,queue_size=1)
rospy.Subscriber("/carla/ego_vehicle/odometry",Odometry,pos_callback)
rospy.Subscriber("/move_base_simple/goal",PoseStamped,aim_pos_callback)
rospy.Subscriber("/carla/global_path",Path,path_callback)
rospy.Subscriber("/carla/ego_vehicle/speedometer",Float32,speed_callback)
marker_pub = rospy.Publisher('BasePath', Marker, queue_size=10)
goal_pub=rospy.Publisher("/move_base_simple/goal",PoseStamped)
cmd_pub=rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd',CarlaEgoVehicleControl,queue_size=10)
# 设置按钮回调
viz.set_reset_callback1(on_reset)
viz.set_goal_callback(set_goal)
refresh_task=threading.Thread(target=refresh_hanshu)
while type(pos)==type(None) or type(color_img)==type(None) or type(global_path)==type(None):
    pass
try:
    client = carla.Client("localhost", 2000)
    world = client.get_world()
    carla_map = world.get_map()
    min_x, max_x, min_y, max_y = calculate_map_bounds(carla_map)
    viz.map_pos_start_y=min_y
    viz.map_pos_start_x = min_x
    refresh_task.start()
    viz.update_map_wh(color_img.shape[1],color_img.shape[0])
    while True:
        viz.update_car(pos[0],pos[1],angle)
        x_start=-viz.moveX
        y_start=-viz.moveY
        publish_img=cv2.cvtColor(color_img[y_start:y_start+300,x_start:x_start+520],cv2.COLOR_BGR2GRAY)
        height, width = publish_img.shape
        publish_data = publish_img.flatten().tolist()
        viz.set_image_data(width, height, publish_data)
        viz.ego_car.PlanAndControl(viz.ego_car)
        if not viz.handle_events():
            break
except KeyboardInterrupt:
    pass
