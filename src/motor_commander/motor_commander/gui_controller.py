#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QGridLayout, QGroupBox, QScrollArea,
                            QLabel, QLineEdit, QPushButton)
from PyQt5.QtCore import Qt
from quadruped.msg import MotorCmdMsg, MotorStateMsg

class MotorControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.node = Node('motor_control_gui')
        self.init_ui()
        self.setup_ros()
        self.show()

    def init_ui(self):
        self.setWindowTitle("beatbot_turtle电机控制终端")
        self.setGeometry(100, 100, 1400, 900)
        self.setStyleSheet("""
            QMainWindow {
                background: #F5F5F5;
            }
            QGroupBox {
                border: 2px solid #3A9CFF;
                border-radius: 8px;
                margin-top: 12px;
                padding: 10px;
                font: bold 12pt "微软雅黑";
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                color: #1A73E8;
            }
            QLabel {
                color: #444;
                font: 10pt "微软雅黑";
            }
            QLineEdit {
                border: 1px solid #CCC;
                border-radius: 3px;
                padding: 5px;
                font: 9pt "Consolas";
            }
            QPushButton {
                background: #3A9CFF;
                color: white;
                padding: 8px 15px;
                border-radius: 5px;
                font: bold 10pt "微软雅黑";
            }
            QPushButton:hover {
                background: #1E88E5;
            }
        """)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content_widget = QWidget()
        scroll.setWidget(content_widget)
        self.setCentralWidget(scroll)

        grid = QGridLayout(content_widget)
        grid.setSpacing(20)
        grid.setContentsMargins(20, 20, 20, 20)
        self.motor_widgets = []

        for i in range(16):
            group = QGroupBox(f"电机 #{i+1:02d}")
            layout = QVBoxLayout()
            
            fields = {
                '模式': QLineEdit('10', placeholderText='控制模式'),
                '目标位置': QLineEdit('0.0', placeholderText='rad'),
                '目标速度': QLineEdit('0.0', placeholderText='rad/s'),
                '目标力矩': QLineEdit('0.0', placeholderText='Nm'),
                '刚度系数': QLineEdit('80.0', placeholderText='Kp'),
                '阻尼系数': QLineEdit('3.0', placeholderText='Kd')
            }

            param_grid = QGridLayout()
            param_grid.addWidget(QLabel('模式'), 0, 0)
            param_grid.addWidget(fields['模式'], 0, 1)
            param_grid.addWidget(QLabel('目标位置'), 1, 0)
            param_grid.addWidget(fields['目标位置'], 1, 1)
            param_grid.addWidget(QLabel('目标速度'), 2, 0)
            param_grid.addWidget(fields['目标速度'], 2, 1)
            param_grid.addWidget(QLabel('目标力矩'), 0, 2)
            param_grid.addWidget(fields['目标力矩'], 0, 3)
            param_grid.addWidget(QLabel('刚度系数'), 1, 2)
            param_grid.addWidget(fields['刚度系数'], 1, 3)
            param_grid.addWidget(QLabel('阻尼系数'), 2, 2)
            param_grid.addWidget(fields['阻尼系数'], 2, 3)

            layout.addLayout(param_grid)
            
            status = QLabel("等待连接...")
            status.setAlignment(Qt.AlignCenter)
            status.setStyleSheet("""
                color: #666;
                font: 10pt "Consolas";
                background: #FAFAFA;
                padding: 8px;
                border-radius: 4px;
            """)

            btn = QPushButton("更新参数")
            btn.clicked.connect(lambda _, idx=i: self.send_command(idx))
            
            layout.addWidget(btn)
            layout.addWidget(status)
            group.setLayout(layout)
            
            grid.addWidget(group, i//4, i%4)
            self.motor_widgets.append({
                'fields': fields,
                'status': status
            })

    def setup_ros(self):
        self.publisher = self.node.create_publisher(MotorCmdMsg, 'motor_commands', 10)
        self.subscription = self.node.create_subscription(
            MotorStateMsg,
            'motor_states',
            self.state_callback,
            10
        )
        self.timer = self.node.create_timer(0.1, self.ros_spin)

    def ros_spin(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def send_command(self, motor_id):
        try:
            msg = MotorCmdMsg()
            msg.motor_index = list(range(16))
            msg.modes = [int(w['fields']['模式'].text()) for w in self.motor_widgets]
            msg.q_targets = [float(w['fields']['目标位置'].text()) for w in self.motor_widgets]
            msg.dq_targets = [float(w['fields']['目标速度'].text()) for w in self.motor_widgets]
            msg.tau_targets = [float(w['fields']['目标力矩'].text()) for w in self.motor_widgets]
            msg.kp_gains = [float(w['fields']['刚度系数'].text()) for w in self.motor_widgets]
            msg.kd_gains = [float(w['fields']['阻尼系数'].text()) for w in self.motor_widgets]
            
            self.publisher.publish(msg)
            self.motor_widgets[motor_id]['status'].setStyleSheet("color: #4CAF50;")
            self.motor_widgets[motor_id]['status'].setText("参数已更新 ✓")
        except Exception as e:
            self.motor_widgets[motor_id]['status'].setStyleSheet("color: #E53935;")
            self.motor_widgets[motor_id]['status'].setText(f"错误：{str(e)}")

    def state_callback(self, msg):
        for i in range(16):
            status = f"位置: {msg.q[i]:.2f}rad\n速度: {msg.dq[i]:.2f}rad/s\n力矩: {msg.tau_est[i]:.2f}Nm"
            self.motor_widgets[i]['status'].setText(status)
            self.motor_widgets[i]['status'].setStyleSheet("color: #2196F3;")

if __name__ == '__main__':
    rclpy.init()
    app = QApplication(sys.argv)
    window = MotorControlGUI()
    sys.exit(app.exec_())