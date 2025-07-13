import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout, QTextEdit, QSlider
from PyQt5.QtCore import Qt, QTimer
from rclpy.qos import QoSProfile
import rclpy

from .camera_widget import CameraWidget
from .sensor_reader import SensorReader
from .controller import RobotController

class KiltedControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Four Wheel Bot Control Panel')
        self.setGeometry(100, 100, 800, 600)

        # ROS2 Init
        rclpy.init()
        self.node = rclpy.create_node('kilted_gui_node')
        self.sensor_reader = SensorReader(self.node)
        self.controller = RobotController(self.node)

        # UI Elements
        self.camera = CameraWidget(self.node)
        self.oled_text = QTextEdit("Hello\nWorld!")
        self.servo_slider_0 = QSlider(Qt.Horizontal)
        self.servo_slider_1 = QSlider(Qt.Vertical)

        self.wheel_speed = QSlider(Qt.Horizontal)
        self.turn_speed = QSlider(Qt.Horizontal)
        
        self._wheel_speed = 1.0
        self._turn_speed = 1.0

        self.init_ui()

        # ROS Timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.ros_spin)
        self.timer.start(50)  # 20 Hz


    def init_ui(self):
        # Outer horizontal layout
        outer_layout = QHBoxLayout()

        # Main vertical layout for most widgets
        layout = QVBoxLayout()

        # Camera
        layout.addWidget(self.camera)

        # Sensor Labels
        self.sensor_labels = {
            'ldr_left': QLabel('LDR Left: '),
            'ldr_right': QLabel('LDR Right: '),
            'battery': QLabel('Battery: '),
            'ultrasonic': QLabel('Ultrasonic: '),
            'line': QLabel('Line Sensors: ')
        }
        for label in self.sensor_labels.values():
            layout.addWidget(label)
        self.sensor_reader.set_update_callback(self.update_sensor_labels)

        # Servo 0 Slider (horizontal)
        self.servo_slider_0.setRange(0, 180)
        self.servo_slider_0.setValue(90)
        self.servo_slider_0.setInvertedAppearance(True)
        self.servo_slider_0.valueChanged.connect(lambda v: self.controller.set_servo(0, v))
        layout.addWidget(QLabel("Servo 0"))
        layout.addWidget(self.servo_slider_0)

        # Buzzer and LED
        buzzer_btn = QPushButton("Toggle Buzzer")
        buzzer_btn.clicked.connect(self.controller.toggle_buzzer)
        layout.addWidget(buzzer_btn)

        led_btn = QPushButton("Toggle LEDs")
        led_btn.clicked.connect(self.controller.toggle_leds)
        layout.addWidget(led_btn)

        # OLED
        oled_button = QPushButton("Send to OLED")
        oled_button.clicked.connect(self.send_oled_text)
        layout.addWidget(self.oled_text)
        layout.addWidget(oled_button)

        # Wheel and Turn Speed
        self.wheel_speed.setRange(0, 100)
        self.wheel_speed.setValue(50)
        self.wheel_speed.valueChanged.connect(lambda v: self.set_wheel_speed(v))
        layout.addWidget(QLabel("Wheel Speed %"))
        layout.addWidget(self.wheel_speed)

        self.turn_speed.setRange(0, 100)
        self.turn_speed.setValue(50)
        self.turn_speed.valueChanged.connect(lambda v: self.set_turn_speed(v))
        layout.addWidget(QLabel("Turn Speed %"))
        layout.addWidget(self.turn_speed)

        # Control Buttons
        control_layout = QHBoxLayout()
        forward_btn = QPushButton("↑")
        back_btn = QPushButton("↓")
        left_btn = QPushButton("←")
        right_btn = QPushButton("→")
        stop_btn = QPushButton("Stop")

        forward_btn.clicked.connect(lambda: self.controller.move(self._wheel_speed, 0))
        back_btn.clicked.connect(lambda: self.controller.move(self._wheel_speed * -1.0, 0))
        left_btn.clicked.connect(lambda: self.controller.move(0.0, self._turn_speed))
        right_btn.clicked.connect(lambda: self.controller.move(0.0, self._turn_speed * -1.0))
        stop_btn.clicked.connect(lambda: self.controller.move(0, 0))

        control_layout.addWidget(left_btn)
        control_layout.addWidget(forward_btn)
        control_layout.addWidget(stop_btn)
        control_layout.addWidget(back_btn)
        control_layout.addWidget(right_btn)
        layout.addLayout(control_layout)

        # Add the main vertical layout to the outer horizontal layout
        outer_layout.addLayout(layout)

        # --- New Right Column with Servo 1 Slider ---
        right_layout = QVBoxLayout()
        self.servo_slider_1.setRange(0, 180)
        self.servo_slider_1.setValue(90)
        self.servo_slider_1.setInvertedAppearance(True)
        self.servo_slider_1.valueChanged.connect(lambda v: self.controller.set_servo(1, v))
        right_layout.addWidget(QLabel("Servo 1"))
        right_layout.addWidget(self.servo_slider_1)
        right_layout.addStretch()

        outer_layout.addLayout(right_layout)

        self.setLayout(outer_layout)


    def update_sensor_labels(self, values):
        for key, val in values.items():
            self.sensor_labels[key].setText(f"{key}: {val:.2f}" if isinstance(val, float) else f"{key}: {val}")

    def send_oled_text(self):
        self.controller.set_oled_text(self.oled_text.toPlainText())

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_W:
            self.controller.move(0.5, 0.0)
        elif key == Qt.Key_S:
            self.controller.move(-0.5, 0.0)
        elif key == Qt.Key_A:
            self.controller.move(0.0, 1.0)
        elif key == Qt.Key_D:
            self.controller.move(0.0, -1.0)
        elif key == Qt.Key_Space:
            self.controller.move(0, 0)

    def ros_spin(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)

    def closeEvent(self, event):
        self.controller.stop()
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

    def set_wheel_speed(self, speed):
        self._wheel_speed = speed * 0.02

    def set_turn_speed(self, speed):
        self._turn_speed = speed* 0.02


def main():
    app = QApplication(sys.argv)
    window = KiltedControlGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
