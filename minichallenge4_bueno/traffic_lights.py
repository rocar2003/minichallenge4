import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data

class TrafficLights(Node):
    def __init__(self):
        super().__init__('traffic_lights')
        
        self.img = np.ndarray((720, 1280, 3))
        # Valores de los colores
        self.color_ranges = {
            "stop": (np.array([0, 100, 100]), np.array([10, 255, 255])),
            "slow": (np.array([20, 100, 100]), np.array([30, 255, 255])),
            "move": (np.array([40, 40, 40]), np.array([80, 255, 255]))
        }

        self.valid_img = False
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, '/video_source/raw', self.camera_callback, qos_profile_sensor_data)
        self.sub_vel = self.create_subscription(Twist, 'cmd_vel_aux', self.vel_cb, 10)
        self.pub = self.create_publisher(Image, '/img_processing/color', 10)
        self.pub_status = self.create_publisher(String, '/status', 10)
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)
        self.get_logger().info('CV Node started')

        self.status = String()

        self.vel = Twist()

    def count_color(self, red, yellow, green):
        red_pixels = cv2.countNonZero(red)
        yellow_pixels = cv2.countNonZero(yellow)
        green_pixels = cv2.countNonZero(green)

        if red_pixels > yellow_pixels and red_pixels > green_pixels:
            return "stop"
        elif yellow_pixels > red_pixels and yellow_pixels > green_pixels:
            return "slow"
        elif green_pixels > red_pixels and green_pixels > yellow_pixels:
            return "move"
        else:
            if self.status == "stop":
                return "stop"
            else:
                return "move"

    def process_color(self, mask, color):
        detected_output = cv2.bitwise_and(self.img, self.img, mask=mask)
        gray = cv2.cvtColor(detected_output, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray, 5)
        canny = cv2.Canny(blur, 75, 250)
        return canny

    def camera_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.valid_img = True
        except:
            self.get_logger().info('Failed to get an image')

    def vel_cb(self,msg):
        try:
            self.vel = msg
        except:
            self.get_logger().info('Failed to get velocity')


    def timer_callback(self):
        try:
            if self.valid_img:
                hsvFrame = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

                color_masks = {color: cv2.inRange(hsvFrame, lower, upper) for color, (lower, upper) in self.color_ranges.items()}
                rgb = self.count_color(color_masks["stop"], color_masks["slow"], color_masks["move"])

                if self.status.data == 'move':
                    self.pub_vel.publish(self.vel)

                elif self.status.data == 'slow':
                    self.vel.linear.x = self.vel.linear.x/2.0
                    self.vel.angular.z = self.vel.angular.z/2.0 
                    self.pub_vel.publish(self.vel)
                    self.vel.linear.x = self.vel.linear.x * 2.0
                    self.vel.angular.z = self.vel.angular.z * 2.0

                else: pass

                canny = self.process_color(color_masks.get(rgb, None), rgb)

                self.status.data = rgb
                self.pub_status.publish(self.status)
                self.pub.publish(self.bridge.cv2_to_imgmsg(canny))
                self.valid_img = False
        except:
            self.get_logger().info('Failed to process image')

def main(args=None):
    rclpy.init(args=args)
    cv_e = TrafficLights()
    rclpy.spin(cv_e)
    cv_e.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
