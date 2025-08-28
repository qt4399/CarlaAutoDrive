#!/usr/bin/env python3
import rospy
import carla
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
import math
import numpy as np
class CarlaRoadVisualizer:
    def __init__(self):
        rospy.init_node('draw_line', anonymous=True)
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.marker_pub = rospy.Publisher('/carla/road_network', MarkerArray, queue_size=10, latch=True)
        self.ego_pose_sub = rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.ego_pose_callback)
        self.frame_id = 'map'
        self.line_width = 0.05
        self.z_offset = 0.05
        self.colors = {
            'road_edge': ColorRGBA(0.8, 0.8, 0.8, 0.9),
            'lane_solid': ColorRGBA(1.0, 1.0, 1.0, 0.95),
            'lane_broken': ColorRGBA(1.0, 1.0, 1.0, 0.9),
            'crosswalk': ColorRGBA(0.9, 0.9, 0.1, 0.9),
            'road_surface': ColorRGBA(0.3, 0.3, 0.3, 0.8)
        }
        self.marker_id_counter = 0
        self.waypoints = self.map.generate_waypoints(1)
        self.topology = self.map.get_topology()
        self.cached_markers = None

        self.ego_position = None
        self.view_radius = 50.0

    def ego_pose_callback(self, msg):
        self.ego_position = carla.Location(
            x=msg.pose.pose.position.x,
            y=-msg.pose.pose.position.y,
            z=msg.pose.pose.position.z
        )

    def is_in_view_range(self, location):
        if self.ego_position is None:
            return False

        dx = location.x - self.ego_position.x
        dy = location.y - self.ego_position.y
        distance = math.sqrt(dx * dx + dy * dy)

        return distance <= self.view_radius

    def carla_to_ros_point(self, carla_location):
        return Point(
            x=carla_location.x,
            y=-carla_location.y,  # Y轴反转
            z=carla_location.z + self.z_offset
        )

    def create_line_marker(self, points, ns, color, width=None, is_dashed=False):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        marker.type = Marker.LINE_LIST if is_dashed else Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = width if width else self.line_width
        marker.color = color
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0)
        marker.frame_locked = True
        if is_dashed:
            dashed_points = []
            segment_length = 0.5
            gap_length = 0.5
            start = points[0]
            end = points[1]
            direction = end - start
            total_length = math.sqrt(direction.x ** 2 + direction.y ** 2)
            direction_unit = direction / total_length
            current_pos = 0.0
            while current_pos < total_length:
                next_pos = current_pos + segment_length
                if next_pos > total_length:
                    next_pos = total_length

                p1 = start + direction_unit * current_pos
                p2 = start + direction_unit * next_pos
                dashed_points.extend([p1, p2])

                current_pos = next_pos + gap_length

            marker.points = [self.carla_to_ros_point(p) for p in dashed_points]
        else:
            marker.points = [self.carla_to_ros_point(p) for p in points]

        return marker

    def process_lane_markings(self, waypoint, marker_array):
        if not self.is_in_view_range(waypoint.transform.location):
            return
        left_data = self.get_lane_boundary_points(waypoint, 'left')
        if left_data['points'] and self.is_in_view_range(left_data['points'][0]):
            marker_array.markers.append(
                self.create_line_marker(
                    left_data['points'],
                    'lane_boundaries_left',
                    self.colors['lane_solid'] if not left_data['is_dashed'] else self.colors['lane_broken'],
                    self.line_width * 1.2,
                    is_dashed=left_data['is_dashed']
                )
            )
        right_data = self.get_lane_boundary_points(waypoint, 'right')
        if right_data['points'] and self.is_in_view_range(right_data['points'][0]):
            marker_array.markers.append(
                self.create_line_marker(
                    right_data['points'],
                    'lane_boundaries_right',
                    self.colors['lane_solid'] if not right_data['is_dashed'] else self.colors['lane_broken'],
                    self.line_width * 1.2,
                    is_dashed=right_data['is_dashed']
                )
            )

    def get_lane_boundary_points(self, waypoint, side):
        try:
            offset = waypoint.lane_width / 2 * (-1 if side == 'left' else 1)
            next_wps = waypoint.next(1.0)

            if not next_wps:
                return {'points': None, 'is_dashed': False}

            next_wp = next_wps[0]
            current_loc = waypoint.transform.location + waypoint.transform.get_right_vector() * offset
            next_loc = next_wp.transform.location + next_wp.transform.get_right_vector() * offset
            lane_marking = waypoint.left_lane_marking if side == 'left' else waypoint.right_lane_marking
            is_dashed = lane_marking.type == carla.LaneMarkingType.Broken

            return {
                'points': [current_loc, next_loc],
                'is_dashed': is_dashed
            }
        except Exception as e:
            rospy.logwarn(f"获取车道边界点失败: {str(e)}")
            return {'points': None, 'is_dashed': False}

    def draw_crosswalk(self, waypoint, marker_array):
        try:
            if not self.is_in_view_range(waypoint.transform.location):
                return

            crosswalk_points = []
            for i in range(-2, 3):
                offset = waypoint.transform.get_right_vector() * i * 0.5
                point = waypoint.transform.location + offset
                crosswalk_points.append(point)

            marker_array.markers.append(
                self.create_line_marker(
                    crosswalk_points,
                    'crosswalks',
                    self.colors['crosswalk'],
                    self.line_width * 2
                )
            )
        except Exception as e:
            rospy.logwarn(f"绘制人行横道失败: {str(e)}")

    def generate_road_markers(self):
        marker_array = MarkerArray()
        self.marker_id_counter = 0

        # 处理所有航点
        for waypoint in self.waypoints:
            try:
                # 检查waypoint是否在视野范围内
                if not self.is_in_view_range(waypoint.transform.location):
                    continue

                self.process_lane_markings(waypoint, marker_array)
            except Exception as e:
                rospy.logwarn(f"处理航点失败: {str(e)}")
                continue

        return marker_array

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                self.cached_markers = self.generate_road_markers()
                self.marker_pub.publish(self.cached_markers)
            except Exception as e:
                rospy.logerr(f"生成标记失败: {str(e)}")
            rate.sleep()


if __name__ == '__main__':
    try:
        visualizer = CarlaRoadVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass