#!/usr/bin/python3
import sys
import subprocess
import threading
import queue

import rclpy
from rclpy.node import Node

from rosbag2_interfaces.srv import SetRate, Pause, Resume
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget,
    QFileDialog, QLabel, QHBoxLayout, QMessageBox, QDoubleSpinBox, QPlainTextEdit
)
from PyQt5.QtCore import QTimer


class RosbagControlNode(Node):
    def __init__(self):
        super().__init__('rosbag_control_gui')

        self.declare_parameter('/topic_prefix', '/simulation')
        self.topic_prefix = self.get_parameter('/topic_prefix').get_parameter_value().string_value
        self.play_process = None
        self.current_rate = 1.0

        # 记录上次选择的bag目录（若为None表示从未选择过）
        self.last_bag_dir = None

        # 创建客户端，但不在此处阻塞等待服务可用
        self.pause_client = self.create_client(Pause, '/rosbag2_player/pause')
        self.resume_client = self.create_client(Resume, '/rosbag2_player/resume')
        self.set_rate_client = self.create_client(SetRate, '/rosbag2_player/set_rate')

        self.get_logger().info("RosbagControlNode initialized.")


class RosbagControlGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        
        self.is_paused = False  # 用于追踪是否已处于“暂停”状态

        # 队列：存放来自子进程 stdout/stderr 的“待显示”日志
        self.stdout_queue = queue.Queue()
        self.stderr_queue = queue.Queue()

        # 线程引用，用于在关闭窗口时安全退出
        self.stdout_thread = None
        self.stderr_thread = None

        # 初始化界面
        self.init_ui()

        # 定时器：每秒更新界面状态 & 读取队列日志
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.periodic_update)
        self.timer.start(500)

    def init_ui(self):
        self.setWindowTitle('ROS2 Bag Control Panel')
        self.setGeometry(300, 200, 500, 340)

        # -- 按钮与标签 --
        self.btn_select = QPushButton('Select Bag', self)
        self.btn_play_pause = QPushButton('Play/Pause', self)
        self.btn_increase = QPushButton('+ Speed', self)
        self.btn_decrease = QPushButton('- Speed', self)
        self.status_label = QLabel('Status: Stopped', self)
        self.rate_label = QLabel('Current Rate: 1.0x', self)

        # -- 速度调整“步进”设置 --
        self.step_label = QLabel('Speed Step:')
        self.speed_step_box = QDoubleSpinBox()
        self.speed_step_box.setRange(0.01, 10.0)
        self.speed_step_box.setSingleStep(0.05)
        self.speed_step_box.setValue(0.5)  # 默认步进 0.5

        # -- 日志输出窗口 --
        self.log_label = QLabel('Logs:')
        self.log_text = QPlainTextEdit()
        self.log_text.setReadOnly(True)

        # -- 布局 --
        main_layout = QVBoxLayout()

        # 第一行：文件选择按钮
        file_layout = QHBoxLayout()
        file_layout.addWidget(self.btn_select)
        main_layout.addLayout(file_layout)

        # 第二行：控制按钮
        control_layout = QHBoxLayout()
        control_layout.addWidget(self.btn_play_pause)
        control_layout.addWidget(self.btn_increase)
        control_layout.addWidget(self.btn_decrease)
        main_layout.addLayout(control_layout)

        # 第三行：速率显示 + 速率步进设置
        rate_layout = QHBoxLayout()
        rate_layout.addWidget(self.status_label)
        rate_layout.addWidget(self.rate_label)
        rate_layout.addWidget(self.step_label)
        rate_layout.addWidget(self.speed_step_box)
        main_layout.addLayout(rate_layout)

        # 第四行：日志输出
        log_layout = QVBoxLayout()
        log_layout.addWidget(self.log_label)
        log_layout.addWidget(self.log_text)
        main_layout.addLayout(log_layout)

        # 设置主窗口布局
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # -- 信号连接 --
        self.btn_select.clicked.connect(self.select_bag)
        self.btn_play_pause.clicked.connect(self.toggle_play_pause)
        self.btn_increase.clicked.connect(self.increase_speed)
        self.btn_decrease.clicked.connect(self.decrease_speed)

    def log_message(self, msg: str):
        """
        在日志窗口和控制台同时输出消息。
        """
        self.log_text.appendPlainText(msg)
        print(msg)  # 控制台也可同步输出

    def select_bag(self):
        """
        让用户选择bag文件夹，并调用 start_playback。
        """
        options = QFileDialog.Options()
        bag_dir = QFileDialog.getExistingDirectory(
            self, "Select Bag Directory", "", options=options
        )
        if bag_dir:
            self.start_playback(bag_dir)

    def start_playback(self, bag_dir):
        """
        启动对指定bag文件夹的播放，并开启后台线程读取子进程输出。
        """
        # 如果已经有进程在跑就不给重播
        if self.node.play_process and self.node.play_process.poll() is None:
            QMessageBox.warning(self, "Warning", "A bag is already playing!")
            return
        self.log_message(f"Starting playback: {bag_dir}")
        self.is_paused = False
        self.node.last_bag_dir = bag_dir
        if self.node.topic_prefix != "/simulation" or self.node.topic_prefix != "/hardware":
            self.node.topic_prefix = "/"+bag_dir.split('/')[-1].split('_')[-1]
        self.log_message(f"topci_prefix: {self.node.topic_prefix}")
        if self.node.topic_prefix == "/simulation":
            command = f"ros2 bag play {bag_dir} --clock --start-offset 0 --rate {self.node.current_rate} \
            --remap "+self.node.topic_prefix+"/odom:=/odom  "+ \
            self.node.topic_prefix+"/joint_states:=/joint_states"
        elif self.node.topic_prefix == "/hardware":
            command = f"ros2 bag play {bag_dir} --clock --start-offset 0 --rate {self.node.current_rate} \
            --remap "+self.node.topic_prefix+"/myodom:=/odom  "+ \
            self.node.topic_prefix+"/joint_states:=/joint_states"
        
        # 注意：text=True 以接收字符串，而不是字节流
        self.node.play_process = subprocess.Popen(
            command.split(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        self.status_label.setText(f'Playing: {bag_dir}')

        # 启动两个后台线程，分别监听 stdout 和 stderr
        self.stdout_thread = threading.Thread(
            target=self.read_stdout, daemon=True
        )
        self.stderr_thread = threading.Thread(
            target=self.read_stderr, daemon=True
        )
        self.stdout_thread.start()
        self.stderr_thread.start()

    def read_stdout(self):
        """
        后台线程函数：不断从子进程 stdout 中读取数据，放到 stdout_queue。
        """
        if not self.node.play_process or not self.node.play_process.stdout:
            return
        for line in self.node.play_process.stdout:
            self.stdout_queue.put(line)

    def read_stderr(self):
        """
        后台线程函数：不断从子进程 stderr 中读取数据，放到 stderr_queue。
        """
        if not self.node.play_process or not self.node.play_process.stderr:
            return
        for line in self.node.play_process.stderr:
            self.stderr_queue.put(line)

    def toggle_play_pause(self):
        """
        用户点击“Play/Pause”按钮时的入口。
        如果没有正在播放的子进程，且 last_bag_dir 不为空，则表示想重新播放。
        如果正在播放，则对其执行“暂停/继续”功能。
        """
        # 如果播放进程不存在，或者进程已经结束
        if self.node.play_process is None or self.node.play_process.poll() is not None:
            if self.node.last_bag_dir:
                self.log_message("No active playback. Trying to replay last bag.")
                self.start_playback(self.node.last_bag_dir)
            else:
                QMessageBox.information(self, "Info", "No bag selected yet.")
            return

        # 如果播放进程还在，则进行“暂停/继续”切换
        if not self.is_paused:
            self.call_pause_service()
        else:
            self.call_resume_service()

    def call_pause_service(self):
        """
        调用 Pause 服务。
        """
        if not self.node.pause_client.service_is_ready():
            self.log_message("Pause service not ready.")
            QMessageBox.warning(self, "Warning", "Pause service is not available yet!")
            return

        req_pause = Pause.Request()
        future = self.node.pause_client.call_async(req_pause)
        future.add_done_callback(self.pause_callback)

    def pause_callback(self, future):
        """
        暂停服务的回调。
        """
        try:
            future.result()
            self.is_paused = True
            self.status_label.setText('Status: Paused')
            self.log_message("Playback paused.")
        except Exception as e:
            error_msg = f"Pause failed: {str(e)}"
            self.log_message(error_msg)
            QMessageBox.critical(self, "Error", error_msg)

    def call_resume_service(self):
        """
        调用 Resume 服务。
        """
        if not self.node.resume_client.service_is_ready():
            self.log_message("Resume service not ready.")
            QMessageBox.warning(self, "Warning", "Resume service is not available yet!")
            return

        req_resume = Resume.Request()
        future = self.node.resume_client.call_async(req_resume)
        future.add_done_callback(self.resume_callback)

    def resume_callback(self, future):
        """
        继续服务的回调。
        """
        try:
            future.result()
            self.is_paused = False
            self.status_label.setText('Status: Playing')
            self.log_message("Playback resumed.")
        except Exception as e:
            error_msg = f"Resume failed: {str(e)}"
            self.log_message(error_msg)
            QMessageBox.critical(self, "Error", error_msg)

    def increase_speed(self):
        """
        调快播放速度。
        增加值由 speed_step_box 控件指定。
        """
        step = self.speed_step_box.value()
        self.adjust_rate(step)

    def decrease_speed(self):
        """
        调慢播放速度。
        减少值由 speed_step_box 控件指定。
        """
        step = self.speed_step_box.value()
        self.adjust_rate(-step)

    def adjust_rate(self, delta):
        """
        调整播放速率。
        """
        if not self.node.set_rate_client.service_is_ready():
            self.log_message("Set rate service not ready.")
            QMessageBox.warning(self, "Warning", "Set rate service is not available yet!")
            return

        new_rate = max(0.1, self.node.current_rate + delta)
        req = SetRate.Request()
        req.rate = new_rate

        future = self.node.set_rate_client.call_async(req)
        # 用lambda将 new_rate 传给回调
        future.add_done_callback(lambda f: self.rate_callback(f, new_rate))

    def rate_callback(self, future, new_rate):
        """
        调整播放速率的回调函数。
        """
        try:
            future.result()
            self.node.current_rate = new_rate
            msg = f"Current Rate changed to: {new_rate}x"
            self.rate_label.setText(msg)
            self.log_message(msg)
        except Exception as e:
            error_msg = f"Rate change failed: {str(e)}"
            self.log_message(error_msg)
            QMessageBox.critical(self, "Error", error_msg)

    def periodic_update(self):
        """
        定时任务：
        1. 检查播放进程状态（更新status）
        2. 从队列中取出子进程的 stdout/stderr 日志并显示
        """
        self.update_status()
        self.fetch_process_output()

    def fetch_process_output(self):
        """
        从队列中提取子进程输出的stdout/stderr日志并打印到log窗口。
        """
        # 取出并显示 stdout 中的内容
        while not self.stdout_queue.empty():
            line = self.stdout_queue.get_nowait()
            self.log_message(f"[stdout] {line.rstrip()}")

        # 取出并显示 stderr 中的内容
        while not self.stderr_queue.empty():
            line = self.stderr_queue.get_nowait()
            self.log_message(f"[stderr] {line.rstrip()}")

    def update_status(self):
        """
        定时检查播放进程是否还在运行。
        如果已经退出，则显示 Stopped，并检查返回码是否有错误。
        """
        if self.node.play_process and self.node.play_process.poll() is None:
            # 如果不在暂停状态，就显示“Playing”
            if not self.is_paused:
                self.status_label.setText('Status: Playing')
        else:
            if self.node.play_process and self.node.play_process.poll() is not None:
                # 进程已经退出
                retcode = self.node.play_process.returncode
                if retcode != 0:
                    self.log_message(f"Playback process ended with error code: {retcode}")
            self.status_label.setText('Status: Stopped')

    def closeEvent(self, event):
        """
        关闭窗口时终止播放进程并销毁ROS节点，结束日志线程。
        """
        if self.node.play_process and self.node.play_process.poll() is None:
            self.log_message("Terminating playback process.")
            self.node.play_process.terminate()
        self.node.destroy_node()
        self.log_message("Shutting down node and GUI.")
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    # 创建ROS节点
    ros_node = RosbagControlNode()

    # 创建GUI
    gui = RosbagControlGUI(ros_node)
    gui.show()

    # 启动Qt和ROS事件循环
    timer = QTimer()
    timer.start(100)  # 这里可调整回调频率
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()