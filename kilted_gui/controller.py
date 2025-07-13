# kilted_gui/controller.py

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32MultiArray, Bool, String

class RobotController:
    def __init__(self, node):
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self.servo_pubs = {
            0: node.create_publisher(Float32, '/servo/s0/angle', 10),
            1: node.create_publisher(Float32, '/servo/s1/angle', 10)
        }
        self.led_pub = node.create_publisher(Int32MultiArray, '/led/set_color', 10)
        self.buzzer_pub = node.create_publisher(Bool, '/buzzer', 10)
        self.oled_pub = node.create_publisher(String, '/oled_text', 10)

        self.buzzer_on = False
        self.leds_on = False

    def move(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.move(0.0, 0.0)

    def set_servo(self, channel, angle):
        if channel in self.servo_pubs:
            self.servo_pubs[channel].publish(Float32(data=float(angle)))

    def set_led(self, index, r, g, b):
        msg = Int32MultiArray(data=[index, r, g, b])
        self.led_pub.publish(msg)

    def toggle_leds(self):
        self.leds_on = not self.leds_on
        if self.leds_on:
            self.set_led(0, 255, 0, 0)
            self.set_led(1, 0, 255, 0)
            self.set_led(2, 0, 0, 255)
            self.set_led(3, 255, 255, 0)
            self.set_led(4, 0, 255, 255)
            self.set_led(5, 255, 0, 255)
            self.set_led(6, 255, 255, 255)
            self.set_led(7, 128, 128, 128)
        else:
            for index in range(8):
                self.set_led(index, 0, 0, 0)

    def toggle_buzzer(self):
        self.buzzer_on = not self.buzzer_on
        self.buzzer_pub.publish(Bool(data=self.buzzer_on))

    def set_oled_text(self, text):
        self.oled_pub.publish(String(data=text))
