import rospy
from geometry_msgs.msg import Twist
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer

class TeleopController:
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.linear_vel = 1.0  # 기본 선속도 설정
        self.angular_vel = 0.2  # 기본 각속도 설정

    def move_forward_start(self):
        self.twist.linear.x = self.linear_vel

    def move_backward_start(self):
        self.twist.linear.x = -self.linear_vel

    def move_left_start(self):
        self.twist.angular.z = self.angular_vel

    def move_right_start(self):
        self.twist.angular.z = -self.angular_vel

    def move_stop(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0

    def publish_movement(self):
        self.pub.publish(self.twist)

class WindowClass(QMainWindow):
    def __init__(self):
        super().__init__()
        self.teleop_controller = TeleopController()  # TeleopController 초기화
        self.setup_ui()

    def setup_ui(self):
        self.setGeometry(100, 100, 300, 200)

        self.button_forward = QPushButton("Forward", self)
        self.button_forward.setGeometry(100, 30, 70, 30)
        self.button_forward.pressed.connect(self.teleop_controller.move_forward_start)

        self.button_backward = QPushButton("Backward", self)
        self.button_backward.setGeometry(100, 70, 70, 30)
        self.button_backward.pressed.connect(self.teleop_controller.move_backward_start)

        self.button_left = QPushButton("Left", self)
        self.button_left.setGeometry(30, 70, 50, 30)
        self.button_left.pressed.connect(self.teleop_controller.move_left_start)

        self.button_right = QPushButton("Right", self)
        self.button_right.setGeometry(190, 70, 50, 30)
        self.button_right.pressed.connect(self.teleop_controller.move_right_start)

        # 정지 버튼 추가
        self.button_stop = QPushButton("Stop", self)
        self.button_stop.setGeometry(100, 110, 70, 30)
        self.button_stop.clicked.connect(self.teleop_controller.move_stop)  # 정지 버튼 클릭 시 move_stop 메서드 호출

if __name__ == '__main__':
    rospy.init_node('turtlebot3_teleop')
    rospy.set_param("model", "burger")  # 모델 파라미터 설정

    app = QApplication(sys.argv)
    myWindow = WindowClass()
    myWindow.show()

    timer = QTimer()
    timer.timeout.connect(myWindow.teleop_controller.publish_movement)
    timer.start(10)  # 10ms마다 업데이트

    sys.exit(app.exec_())
