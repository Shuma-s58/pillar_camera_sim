"""RViz2 scene, simulated monocular image, camera TF, and geometric estimate."""
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import estimate_from_two_centers, project_point


class Simulation(Node):
    def __init__(self):
        super().__init__('pillar_camera_sim')
        for key, default in {
            'pillar_distance_x': 3.0, 'pillar_spacing': 0.66,
            'pillar_width': 0.07, 'pillar_height': 1.65,
            'camera_start_x': 0.0, 'camera_start_y': 0.0,
            'camera_start_z': 0.9, 'camera_start_yaw': 0.0,
            'move_step': 0.10, 'yaw_step_deg': 5.0,
            'image_width': 800, 'image_height': 600,
            'focal_length_px': 700.0,
        }.items():
            self.declare_parameter(key, default)
        self.getp = lambda name: self.get_parameter(name).value
        self.x0 = float(self.getp('camera_start_x'))
        self.y0 = float(self.getp('camera_start_y'))
        self.z0 = float(self.getp('camera_start_z'))
        self.yaw0 = float(self.getp('camera_start_yaw'))
        self.reset()
        self.width = int(self.getp('image_width'))
        self.height = int(self.getp('image_height'))
        self.fx = float(self.getp('focal_length_px'))
        self.fy = self.fx
        if self.width < 100 or self.height < 100 or self.fx <= 0:
            raise ValueError('Invalid camera intrinsics')
        self.markers_pub = self.create_publisher(MarkerArray, '/scene/markers', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/camera/pose', 10)
        self.estimate_pub = self.create_publisher(String, '/camera/estimate', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(String, '/camera/key_command', self.on_key, 10)
        self.create_timer(0.1, self.render)
        self.get_logger().info('Camera simulation ready. Run ros2 run pillar_camera_sim keyboard_teleop in another terminal.')

    def reset(self):
        self.x, self.y, self.z, self.yaw = self.x0, self.y0, self.z0, self.yaw0

    def on_key(self, msg):
        key = msg.data.lower()
        ds = float(self.getp('move_step'))
        if key == ' ':
            self.reset()
        elif key == 'w':
            self.x += ds * math.cos(self.yaw)
            self.y += ds * math.sin(self.yaw)
        elif key == 's':
            self.x -= ds * math.cos(self.yaw)
            self.y -= ds * math.sin(self.yaw)
        elif key == 'a':
            self.x -= ds * math.sin(self.yaw)
            self.y += ds * math.cos(self.yaw)
        elif key == 'd':
            self.x += ds * math.sin(self.yaw)
            self.y -= ds * math.cos(self.yaw)
        elif key == 'q':
            self.yaw += math.radians(float(self.getp('yaw_step_deg')))
        elif key == 'e':
            self.yaw -= math.radians(float(self.getp('yaw_step_deg')))
        elif key == 'r':
            self.z += ds
        elif key == 'f':
            self.z -= ds

    @staticmethod
    def rgba(marker, r, g, b, a=1.0):
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = r, g, b, a

    def marker(self, marker_id, kind, stamp, ns='scene'):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.ns, m.id, m.type, m.action = ns, marker_id, kind, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def send_scene(self, stamp):
        arr = MarkerArray()
        distance = float(self.getp('pillar_distance_x'))
        baseline = float(self.getp('pillar_spacing'))
        pw = float(self.getp('pillar_width'))
        ph = float(self.getp('pillar_height'))
        floor = self.marker(0, Marker.CUBE, stamp)
        floor.pose.position.x = 1.5
        floor.pose.position.z = -0.015
        floor.scale.x, floor.scale.y, floor.scale.z = 8.0, 4.0, 0.03
        self.rgba(floor, 0.55, 0.58, 0.61, 0.6)
        arr.markers.append(floor)
        for n, py in enumerate((baseline / 2, -baseline / 2), start=1):
            pillar = self.marker(n, Marker.CUBE, stamp)
            pillar.pose.position.x, pillar.pose.position.y = distance, py
            pillar.pose.position.z = ph / 2
            pillar.scale.x, pillar.scale.y, pillar.scale.z = pw, pw, ph
            self.rgba(pillar, 0.95, 0.97, 1.0)
            arr.markers.append(pillar)
        cam = self.marker(10, Marker.CUBE, stamp, 'camera')
        cam.pose.position.x, cam.pose.position.y, cam.pose.position.z = self.x, self.y, self.z
        cam.pose.orientation.z = math.sin(self.yaw / 2)
        cam.pose.orientation.w = math.cos(self.yaw / 2)
        cam.scale.x, cam.scale.y, cam.scale.z = 0.19, 0.14, 0.11
        self.rgba(cam, 0.0, 0.7, 0.95)
        arr.markers.append(cam)
        lines = self.marker(11, Marker.LINE_LIST, stamp, 'camera')
        lines.scale.x = 0.012
        self.rgba(lines, 0.1, 0.7, 0.95, 0.8)
        # Four corners of a horizontal-plane camera view wedge.
        from geometry_msgs.msg import Point
        def pt(xx, yy, zz):
            p = Point()
            p.x, p.y, p.z = xx, yy, zz
            return p
        c = pt(self.x, self.y, self.z)
        fov = math.atan2(self.width / 2.0, self.fx)
        for direction in (-1, 1):
            angle = self.yaw + direction * fov
            lines.points.extend((c, pt(self.x + 1.0 * math.cos(angle), self.y + 1.0 * math.sin(angle), self.z)))
        arr.markers.append(lines)
        self.markers_pub.publish(arr)

    def send_tf(self, stamp):
        world_to_link = TransformStamped()
        world_to_link.header.stamp = stamp
        world_to_link.header.frame_id = 'map'
        world_to_link.child_frame_id = 'camera_link'
        world_to_link.transform.translation.x = self.x
        world_to_link.transform.translation.y = self.y
        world_to_link.transform.translation.z = self.z
        world_to_link.transform.rotation.z = math.sin(self.yaw / 2)
        world_to_link.transform.rotation.w = math.cos(self.yaw / 2)
        link_to_opt = TransformStamped()
        link_to_opt.header.stamp = stamp
        link_to_opt.header.frame_id = 'camera_link'
        link_to_opt.child_frame_id = 'camera_optical_frame'
        # optical Z=body X, optical X=-body Y, optical Y=-body Z
        link_to_opt.transform.rotation.x = -0.5
        link_to_opt.transform.rotation.y = 0.5
        link_to_opt.transform.rotation.z = -0.5
        link_to_opt.transform.rotation.w = 0.5
        self.tf_broadcaster.sendTransform([world_to_link, link_to_opt])
        pose = PoseStamped()
        pose.header = world_to_link.header
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z
        pose.pose.orientation = world_to_link.transform.rotation
        self.pose_pub.publish(pose)

    def render(self):
        stamp = self.get_clock().now().to_msg()
        self.send_tf(stamp)
        self.send_scene(stamp)
        w, h, fx = self.width, self.height, self.fx
        # Image: pale background, horizontal horizon line and ground region.
        img = np.full((h, w, 3), (238, 235, 231), dtype=np.uint8)
        img[h // 2:, :] = (218, 224, 226)
        cv2.line(img, (0, h // 2), (w - 1, h // 2), (115, 115, 115), 1)
        centers = []
        distance_x = float(self.getp('pillar_distance_x'))
        baseline = float(self.getp('pillar_spacing'))
        pw = float(self.getp('pillar_width'))
        ph = float(self.getp('pillar_height'))
        for y_center in (baseline / 2, -baseline / 2):
            # Project four vertical edges and fill their quadrilateral.
            vertices = [self.project(distance_x, y_center + offset, z)
                        for z, offset in ((0, -pw/2), (0, pw/2), (ph, pw/2), (ph, -pw/2))]
            mid = self.project(distance_x, y_center, ph / 2)
            if mid is not None and all(v is not None for v in vertices):
                poly = np.array([[round(v[0]), round(v[1])] for v in vertices], dtype=np.int32)
                cv2.fillConvexPoly(img, poly, (251, 251, 251))
                cv2.polylines(img, [poly], True, (32, 39, 44), 2)
                # Consider only visible pillar centers within the image.
                if 0 <= mid[0] < w and 0 <= mid[1] < h:
                    centers.append(mid[0])
        cv2.line(img, (w // 2, 0), (w // 2, h - 1), (40, 40, 218), 2)
        if len(centers) == 2:
            estimate = estimate_from_two_centers(centers[0], centers[1], baseline, fx, w)
            if estimate is not None:
                range_m, offset_px, offset_m = estimate
                x_center = int(round(sum(centers) / 2))
                cv2.line(img, (x_center, 0), (x_center, h-1), (220, 105, 40), 2)
                cv2.arrowedLine(img, (w // 2, h // 2 + 35),
                                (x_center, h // 2 + 35), (50, 120, 45), 2, tipLength=0.10)
                status = 'range_m=%.2f offset_px=%+.1f offset_m=%+.3f' % (range_m, offset_px, offset_m)
            else:
                status = 'range_m=N/A offset_px=N/A'
        else:
            status = 'range_m=N/A offset_px=N/A (two pillars not visible)'
        cv2.rectangle(img, (0, h - 43), (w, h), (245, 245, 245), -1)
        cv2.putText(img, status, (12, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 20), 2, cv2.LINE_AA)
        cv2.putText(img, 'Camera x=%.2f y=%.2f z=%.2f yaw=%.1f deg' %
                    (self.x, self.y, self.z, math.degrees(self.yaw)),
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (25, 25, 25), 2, cv2.LINE_AA)
        self.estimate_pub.publish(String(data=status))
        imgmsg = Image()
        imgmsg.header.stamp = stamp
        imgmsg.header.frame_id = 'camera_optical_frame'
        imgmsg.height, imgmsg.width = h, w
        imgmsg.encoding, imgmsg.is_bigendian = 'bgr8', 0
        imgmsg.step = w * 3
        imgmsg.data = img.tobytes()
        self.image_pub.publish(imgmsg)
        info = CameraInfo()
        info.header = imgmsg.header
        info.width, info.height = w, h
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [fx, 0.0, w / 2.0, 0.0, self.fy, h / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, w / 2.0, 0.0, 0.0, self.fy, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

    def project(self, x, y, z):
        return project_point(x, y, z, self.x, self.y, self.z,
                             self.yaw, self.fx, self.fy, self.width, self.height)


def main(args=None):
    rclpy.init(args=args)
    node = Simulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
