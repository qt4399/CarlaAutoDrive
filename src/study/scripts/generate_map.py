#!/usr/bin/env python3

import carla
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
class CarlaMapVisualizer:
    def __init__(self):
        rospy.init_node('generate_map', anonymous=True)
        self.host = rospy.get_param('~host', 'localhost')
        self.port = rospy.get_param('~port', 2000)
        self.timeout = rospy.get_param('~timeout', 10.0)
        self.map_resolution = rospy.get_param('~map_resolution', 0.3)
        self.border_padding = rospy.get_param('~border_padding', 5)
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(self.timeout)
        self.window_name = 'CARLA Map (OpenCV)'
        self.map_width = 0
        self.map_height = 0
        self.map_origin_x = 0
        self.map_origin_y = 0
        self.map_pub = rospy.Publisher('carla_map', OccupancyGrid, queue_size=1, latch=True)
        self.image_pub = rospy.Publisher('carla_map_image', Image, queue_size=1, latch=True)
        self.bridge = CvBridge()

    def get_carla_map(self):
        try:
            world = self.client.get_world()
            carla_map = world.get_map()
            return carla_map
        except Exception as e:
            rospy.logerr(f"无法获取CARLA地图: {e}")
            return None

    def calculate_map_bounds(self, carla_map):
        if not carla_map:
            return None, None, None, None

        waypoints = carla_map.generate_waypoints(5.0)

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
        min_x -= self.border_padding
        max_x += self.border_padding
        min_y -= self.border_padding
        max_y += self.border_padding

        return min_x, max_x, min_y, max_y

    def initialize_map_dimensions(self, carla_map):
        min_x, max_x, min_y, max_y = self.calculate_map_bounds(carla_map)
        if min_x is None:
            return False

        # 计算地图尺寸(像素)
        self.map_width = int((max_x - min_x) / self.map_resolution)
        self.map_height = int((max_y - min_y) / self.map_resolution)

        # 存储地图原点(世界坐标)
        self.map_origin_x = min_x
        self.map_origin_y = min_y

        rospy.loginfo(f"地图尺寸: {self.map_width}x{self.map_height} 像素")
        rospy.loginfo(f"地图原点: ({self.map_origin_x}, {self.map_origin_y}) 米")

        return True

    def generate_occupancy_grid(self, carla_map):
        if not carla_map or self.map_width == 0 or self.map_height == 0:
            return None
        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = 'map'
        grid.info.resolution = self.map_resolution
        grid.info.width = self.map_width
        grid.info.height = self.map_height
        grid.info.origin = Pose()
        grid.info.origin.position.x = self.map_origin_x
        grid.info.origin.position.y = self.map_origin_y

        grid.data = [-1] * (self.map_width * self.map_height)

        waypoints = carla_map.generate_waypoints(2.0)

        for waypoint in waypoints:

            x = int((waypoint.transform.location.x - self.map_origin_x) / self.map_resolution)
            y = int((waypoint.transform.location.y - self.map_origin_y) / self.map_resolution)
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                index = y * self.map_width + x
                grid.data[index] = 0
                lane_width = waypoint.lane_width
                width_cells = int(lane_width / (2 * grid.info.resolution))

                for dx in range(-width_cells, width_cells + 1):
                    for dy in range(-width_cells, width_cells + 1):
                        nx = x + dx
                        ny = y + dy
                        if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                            index = ny * self.map_width + nx
                            if grid.data[index] == -1:
                                grid.data[index] = 0

        return grid

    def generate_opencv_image(self, carla_map):
        if not carla_map or self.map_width == 0 or self.map_height == 0:
            return None

        image = np.zeros((self.map_height, self.map_width), dtype=np.uint8)

        waypoints = carla_map.generate_waypoints(2.0)

        road_color = 255
        for waypoint in waypoints:

            x = int((waypoint.transform.location.x - self.map_origin_x) / self.map_resolution)
            y = int((waypoint.transform.location.y - self.map_origin_y) / self.map_resolution)

            if 0 <= x < self.map_width and 0 <= y < self.map_height:

                lane_width = waypoint.lane_width
                width_pixels = max(1, int(lane_width / self.map_resolution))  # 至少1像素宽

                cv2.circle(image, (x, y), width_pixels // 2, road_color, -1)  # -1表示填充
        image = cv2.flip(image, 0)

        return image

    def show_opencv_image(self, image):
        if image is not None:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 600, 600)
            cv2.imshow(self.window_name, image)
            cv2.waitKey(1)

    def publish_map_as_image(self, cv_image):
        if cv_image is None:
            rospy.logwarn("没有可用的地图图像可发布")
            return

        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="mono8")
            ros_image.header.stamp = rospy.Time.now()
            ros_image.header.frame_id = "map"
            self.image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr(f"图像转换失败: {e}")

    def run(self):
        rate = rospy.Rate(1.0)  # 1Hz

        while not rospy.is_shutdown():
            # 获取地图
            carla_map = self.get_carla_map()

            if carla_map:
                if self.map_width == 0 or self.map_height == 0:
                    if not self.initialize_map_dimensions(carla_map):
                        rospy.logerr("无法初始化地图尺寸")
                        rate.sleep()
                        continue
                self.occupancy_grid = self.generate_occupancy_grid(carla_map)

                if self.occupancy_grid:
                    self.occupancy_grid.header.stamp = rospy.Time.now()
                    self.map_pub.publish(self.occupancy_grid)
                    # rospy.loginfo("已发布CARLA占用栅格地图")

                cv_image = self.generate_opencv_image(carla_map)
                cv_image = cv2.flip(cv_image, 0)
                #self.show_opencv_image(cv_image)

                self.publish_map_as_image(cv_image)
            rate.sleep()


if __name__ == '__main__':
    try:
        visualizer = CarlaMapVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
