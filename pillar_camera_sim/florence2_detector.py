"""ROS 2 image adapter for florence2_work/florence2_phrase.py.

Like box_mission, calls Florence from its own venv as a subprocess.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def select_pillars(payload, width, height, min_height, min_ratio, max_boxes=2):
    """Return left-to-right validated bbox candidates from florence2_phrase JSON."""
    regions = payload.get('regions', [])
    if not isinstance(regions, list):
        raise ValueError('Florence JSON regions must be a list')
    found = []
    for entry in regions:
        raw = entry.get('bbox')
        if not isinstance(raw, (list, tuple)) or len(raw) != 4:
            continue
        try:
            x1, y1, x2, y2 = [float(x) for x in raw]
        except (TypeError, ValueError):
            continue
        if not all(math.isfinite(v) for v in (x1, y1, x2, y2)):
            continue
        x1, x2 = max(0.0, min(float(width), x1)), max(0.0, min(float(width), x2))
        y1, y2 = max(0.0, min(float(height), y1)), max(0.0, min(float(height), y2))
        bh, bw = y2-y1, x2-x1
        if bw <= 0 or bh < min_height or bh/bw < min_ratio:
            continue
        found.append({'bbox': [x1, y1, x2, y2], 'center_x': (x1+x2)/2,
                      'label': str(entry.get('label', 'pillar'))})
    # Prefer two tallest, then sort by image x coordinate.
    found.sort(key=lambda b: b['bbox'][3]-b['bbox'][1], reverse=True)
    return sorted(found[:max_boxes], key=lambda b: b['center_x'])


def compute_estimate(boxes, width, fx, spacing):
    estimate = {'valid': False, 'range_m': None, 'offset_px': None, 'offset_m': None}
    if len(boxes) != 2 or fx <= 0 or spacing <= 0:
        return estimate
    d = boxes[1]['center_x']-boxes[0]['center_x']
    if d <= 1:
        return estimate
    z = fx*spacing/d
    dx = (boxes[0]['center_x']+boxes[1]['center_x'])/2 - width/2
    return {'valid': True, 'range_m': z, 'offset_px': dx, 'offset_m': dx*z/fx}


class Detector(Node):
    def __init__(self):
        super().__init__('florence2_detector')
        defaults = {
            'python': '~/florence2_work/.venv/bin/python',
            'script': '~/florence2_work/florence2_phrase.py',
            'model': '',
            'device': 'auto',
            'prompt': 'A white vertical pillar.',
            'timeout': 300.0,
            'inference_period': 3.0,
            'pillar_spacing': 0.66,
            'focal_length_px': 700.0,
            'min_box_height': 25,
            'min_aspect_ratio': 2.0,
            'max_new_tokens': 256,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.latest = None
        self.busy = False
        self.lock = threading.Lock()
        self.fx = float(self.p('focal_length_px'))
        self.pub_image = self.create_publisher(Image, '/florence2/image_annotated', 10)
        self.pub_boxes = self.create_publisher(String, '/florence2/detections', 10)
        self.pub_estimate = self.create_publisher(String, '/florence2/estimate', 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 10)
        period = max(0.1, float(self.p('inference_period')))
        self.create_timer(period, self.schedule)
        self.get_logger().info('Florence-2 via external venv / florence2_phrase.py')

    def p(self, key):
        return self.get_parameter(key).value

    def on_info(self, msg):
        if msg.k[0] > 0:
            self.fx = float(msg.k[0])

    def on_image(self, msg):
        if msg.encoding not in ('bgr8', 'rgb8') or msg.step < msg.width*3:
            return
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            image = raw[:, :msg.width*3].reshape(msg.height, msg.width, 3).copy()
            if msg.encoding == 'rgb8':
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        except (ValueError, TypeError) as exc:
            self.get_logger().warning('Malformed camera frame: %s' % exc)
            return
        with self.lock:
            self.latest = (msg.header, image)

    def schedule(self):
        with self.lock:
            if self.busy or self.latest is None:
                return
            item = self.latest
            self.latest = None
            self.busy = True
        threading.Thread(target=self.process, args=(item,), daemon=True).start()

    def invoke_phrase(self, frame, workdir):
        python = Path(os.path.expandvars(os.path.expanduser(str(self.p('python')))))
        script = Path(os.path.expandvars(os.path.expanduser(str(self.p('script')))))
        if not python.is_file():
            raise FileNotFoundError('Florence python not found: %s' % python)
        if not script.is_file():
            raise FileNotFoundError('Florence script not found: %s' % script)
        input_path = Path(workdir)/'frame.png'
        output_path = Path(workdir)/'annotated.png'
        json_path = Path(workdir)/'detected.json'
        if not cv2.imwrite(str(input_path), frame):
            raise RuntimeError('Cannot save temporary input image')
        command = [str(python), str(script), str(input_path),
                   '--text', str(self.p('prompt')),
                   '--device', str(self.p('device')),
                   '--max-new-tokens', str(int(self.p('max_new_tokens'))),
                   '-o', str(output_path), '--json', str(json_path)]
        model = str(self.p('model')).strip()
        if model:
            command.extend(['--model', os.path.expandvars(os.path.expanduser(model))])
        result = subprocess.run(command, capture_output=True, text=True,
                                timeout=float(self.p('timeout')), check=False)
        if result.returncode != 0:
            raise RuntimeError('florence2_phrase exited %d; stderr: %s' %
                               (result.returncode, result.stderr[-1500:]))
        if not json_path.is_file():
            raise RuntimeError('florence2_phrase did not create JSON output')
        return json.loads(json_path.read_text(encoding='utf-8'))

    def process(self, item):
        header, frame = item
        height, width = frame.shape[:2]
        try:
            with tempfile.TemporaryDirectory(prefix='pillar_florence2_') as workdir:
                payload = self.invoke_phrase(frame, workdir)
            boxes = select_pillars(payload, width, height,
                                   float(self.p('min_box_height')),
                                   float(self.p('min_aspect_ratio')))
            estimate = compute_estimate(boxes, width, self.fx,
                                        float(self.p('pillar_spacing')))
            overlay = frame.copy()
            cv2.line(overlay, (width//2, 0), (width//2, height-1), (0, 0, 255), 2)
            for index, box in enumerate(boxes, start=1):
                x1, y1, x2, y2 = [int(x) for x in box['bbox']]
                cv2.rectangle(overlay, (x1,y1), (x2,y2), (0,200,0), 2)
                cv2.putText(overlay, 'pillar %d' % index, (x1,max(20,y1-8)),
                            cv2.FONT_HERSHEY_SIMPLEX, .6, (0,160,0), 2)
            if estimate['valid']:
                mx = int(width/2 + estimate['offset_px'])
                cv2.line(overlay, (mx,0), (mx,height-1), (255,100,0), 2)
                label = 'range=%.2fm offset=%+.1fpx' % (
                    estimate['range_m'], estimate['offset_px'])
            else:
                label = 'Two pillars not detected'
            cv2.putText(overlay, label, (10, max(20, height-18)),
                        cv2.FONT_HERSHEY_SIMPLEX, .65, (20,20,20), 2)
            image_msg = Image()
            image_msg.header = header
            image_msg.height, image_msg.width = height, width
            image_msg.encoding = 'bgr8'
            image_msg.is_bigendian = 0
            image_msg.step = width*3
            image_msg.data = overlay.tobytes()
            self.pub_image.publish(image_msg)
            self.pub_boxes.publish(String(data=json.dumps({
                'boxes': boxes, 'image_width': width, 'image_height': height,
                'stamp_sec': header.stamp.sec, 'stamp_nanosec': header.stamp.nanosec,
            }, ensure_ascii=False)))
            self.pub_estimate.publish(String(data=json.dumps(estimate)))
        except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error('Florence-2 inference failed: %s' % exc)
            self.pub_estimate.publish(String(data=json.dumps({
                'valid': False, 'range_m': None, 'offset_px': None,
                'offset_m': None, 'error': str(exc),
            })))
        finally:
            with self.lock:
                self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = Detector()
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
