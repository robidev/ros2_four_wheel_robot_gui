# kilted_gui/sensor_reader.py

from std_msgs.msg import Float32, Int8MultiArray
from sensor_msgs.msg import Range

class SensorReader:
    def __init__(self, node):
        self.node = node
        self.values = {
            'ldr_left': 0.0,
            'ldr_right': 0.0,
            'battery': 0.0,
            'ultrasonic': 0.0,
            'line': [0, 0, 0]
        }
        self.callback = None

        node.create_subscription(Float32, '/sensors/ldr_left', self.ldr_left_cb, 10)
        node.create_subscription(Float32, '/sensors/ldr_right', self.ldr_right_cb, 10)
        node.create_subscription(Float32, '/sensors/battery_voltage', self.battery_cb, 10)
        node.create_subscription(Range, '/ultrasonic/distance', self.ultrasonic_cb, 10)
        node.create_subscription(Int8MultiArray, '/line_tracking', self.line_cb, 10)

    def set_update_callback(self, cb):
        self.callback = cb

    def ldr_left_cb(self, msg):
        self.values['ldr_left'] = msg.data
        self.update()

    def ldr_right_cb(self, msg):
        self.values['ldr_right'] = msg.data
        self.update()

    def battery_cb(self, msg):
        self.values['battery'] = msg.data
        self.update()

    def ultrasonic_cb(self, msg):
        self.values['ultrasonic'] = msg.range
        self.update()

    def line_cb(self, msg):
        self.values['line'] = msg.data
        self.update()

    def update(self):
        if self.callback:
            self.callback(self.values)
