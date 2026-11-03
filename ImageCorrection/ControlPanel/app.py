"""Windows PC controller. Opening the app never opens a serial port automatically."""
import argparse
import math
import os
import queue
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path

import serial
from serial.tools import list_ports
from PySide6.QtCore import Qt, QThread, QTimer, Signal, QCoreApplication, QEvent
from PySide6.QtGui import QFont, QFontDatabase
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QMainWindow, QPlainTextEdit, QPushButton, QSlider, QSpinBox,
    QVBoxLayout, QWidget, QTabWidget,
)
from protocol import (SET, ENABLE, DISABLE, STOP, QUERY, ERRORS, Status,
                      PAN_MIN_US, PAN_MAX_US, PAN_CENTER_US,
                      TILT_MIN_US, TILT_MAX_US, TILT_CENTER_US,
                      within_axis_limits, request, response, IMU_QUERY,
                      ImuStatus, ReplyStream)
from calibration import HomeCalibration

DEMO = '__DEMO__'

# ImageCorrection shares TX with a non-preemptible ~1 s J160 packet.
# Match the original ImageCorrection host budget; one request remains outstanding.
RESPONSE_TIMEOUT_S = 3.5

class DemoController:
    def __init__(self):
        self.enabled = False
        self.pan = self.pan_out = PAN_CENTER_US
        self.tilt = self.tilt_out = TILT_CENTER_US
        self.step = 5
        self.last_frame = time.monotonic()
        self.imu_start = self.last_frame
    def exchange(self, seq, command, pan, tilt, step):
        now = time.monotonic()
        frames = int((now - self.last_frame) / 0.02)
        self.last_frame += frames * 0.02
        if self.enabled:
            amount = self.step * frames
            self.pan_out += max(-amount, min(amount, self.pan - self.pan_out))
            self.tilt_out += max(-amount, min(amount, self.tilt - self.tilt_out))
        if command == IMU_QUERY:
            elapsed=now-self.imu_start
            # Explicit offline synthetic sensor; never inferred from PWM pulses.
            angles=(5*math.sin(elapsed),12*math.sin(elapsed/2),25*math.sin(elapsed/3))
            raw=tuple(round(value*32768/180) for value in angles)
            samples=int(elapsed*100)
            return ImuStatus(seq,0,0x43,*raw,0,samples,samples*33,0,1)
        if command == ENABLE:
            self.enabled = True
            self.pan, self.tilt = PAN_CENTER_US, TILT_CENTER_US
        elif command == DISABLE:
            self.enabled = False
            self.pan = self.pan_out = PAN_CENTER_US
            self.tilt = self.tilt_out = TILT_CENTER_US
        elif command == STOP:
            self.pan, self.tilt = self.pan_out, self.tilt_out
        elif command == SET:
            if not self.enabled:
                return Status(seq, 4, 0x40, self.pan, self.tilt, self.pan_out, self.tilt_out)
            self.pan, self.tilt, self.step = pan, tilt, step
        moving = self.enabled and (self.pan != self.pan_out or self.tilt != self.tilt_out)
        return Status(seq, 0, 0x40 | int(self.enabled) | (int(moving) << 1),
                      self.pan, self.tilt, self.pan_out, self.tilt_out)

class SerialWorker(QThread):
    ready = Signal(str)
    update = Signal(object, int)
    error = Signal(str)
    imu_update = Signal(object)
    imu_unavailable = Signal(str)
    def __init__(self, port):
        super().__init__()
        self.port_name = port
        self.control_queue = queue.Queue()
        self.lock = threading.Lock()
        self.latest = None
        self.seq = 0
        self.port = None
        self.demo = DemoController() if port == DEMO else None
        self.replies = ReplyStream()
    def post(self, command, pan=PAN_CENTER_US, tilt=TILT_CENTER_US, step=5):
        with self.lock:
            if command == SET:
                self.latest = (command, pan, tilt, step)
            else:
                self.latest = None
                self.control_queue.put((command, pan, tilt, step))
    def exchange(self, command, pan=PAN_CENTER_US, tilt=TILT_CENTER_US, step=5):
        self.seq = (self.seq + 1) & 255
        if self.demo:
            return self.demo.exchange(self.seq, command, pan, tilt, step)
        # Stop-and-wait: one outstanding command; serial work stays off the UI thread.
        self.port.write(request(self.seq, command, pan, tilt, step))
        deadline = time.monotonic() + RESPONSE_TIMEOUT_S
        received_bytes = 0
        saw_j160 = False
        rx_tail = b''
        while time.monotonic() < deadline:
            data = self.port.read(max(1, self.port.in_waiting))
            received_bytes += len(data)
            saw_j160 = saw_j160 or b'J160' in rx_tail + data
            rx_tail = (rx_tail + data)[-3:]
            for reply in self.replies.feed(data):
                if reply.seq==self.seq:
                    if command==IMU_QUERY and (isinstance(reply,ImuStatus) or reply.result==3): return reply
                    if command!=IMU_QUERY and isinstance(reply,Status): return reply
        detail = ('已收到 J160，但没有匹配的云台应答' if saw_j160 else
                  f'收到 {received_bytes} 字节，但没有匹配的云台应答' if received_bytes else
                  '未收到任何串口数据')
        raise TimeoutError(f'等待 ImageCorrection 应答 {RESPONSE_TIMEOUT_S:g} 秒超时：{detail}。'
                           '确认选择正确 COM 口、板上为 ImageCorrection，串口为 115200 / 8N1；'
                           '电脑 TX 应接 FPGA B24，电脑 RX 应接 C24，并共地。')
    def run(self):
        verified = False
        try:
            if not self.demo:
                self.port = serial.Serial(self.port_name, 115200, timeout=0.02, write_timeout=0.3)
                self.port.reset_input_buffer()
            initial = self.exchange(QUERY)
            if initial.result:
                raise RuntimeError(ERRORS.get(initial.result, 'FPGA 返回错误'))
            verified = True
            self.ready.emit('离线演示' if self.demo else self.port_name)
            self.update.emit(initial, QUERY)
            next_query = time.monotonic() + 0.25
            next_imu = time.monotonic()
            imu_supported = True
            while not self.isInterruptionRequested():
                try:
                    item = self.control_queue.get_nowait()
                except queue.Empty:
                    with self.lock:
                        item, self.latest = self.latest, None
                if item is None:
                    now=time.monotonic()
                    if imu_supported and now>=next_imu:
                        item = (IMU_QUERY,PAN_CENTER_US,TILT_CENTER_US,5)
                    elif now>=next_query:
                        item = (QUERY,PAN_CENTER_US,TILT_CENTER_US,5)
                    else:
                        self.msleep(5)
                        continue
                command, pan, tilt, step = item
                status = self.exchange(command, pan, tilt, step)
                if command==IMU_QUERY:
                    if isinstance(status,ImuStatus) and not status.result:
                        self.imu_update.emit(status)
                    else:
                        imu_supported=False
                        self.imu_unavailable.emit('当前 FPGA 程序不支持角度查询，请下载传感器版 bit 文件。')
                    next_imu=time.monotonic()+0.05
                    continue
                self.update.emit(status, command)
                if status.result:
                    self.error.emit(ERRORS.get(status.result, 'FPGA 返回错误'))
                next_query = time.monotonic() + 0.25
        except Exception as exc:
            self.error.emit(str(exc))
        finally:
            # A deliberate disconnect freezes the ramp and continues holding PWM.
            # Only send to a device whose protocol handshake succeeded.
            if verified:
                try: self.exchange(STOP)
                except Exception: pass
            if self.port:
                self.port.close()

class AxisControl(QGroupBox):
    changed = Signal()
    def __init__(self, title, pin_text, low, high, center):
        super().__init__(title)
        layout = QVBoxLayout(self)
        self.setToolTip(pin_text)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(low, high)
        self.slider.setValue(center)
        self.spin = QSpinBox()
        self.spin.setRange(low, high)
        self.spin.setSuffix(' μs')
        self.spin.setValue(center)
        row = QHBoxLayout()
        row.addWidget(self.slider, 1)
        row.addWidget(self.spin)
        layout.addLayout(row)
        # These controls are placed on the settings page by the main window.
        self.readout = QLabel('输出：—', self)
        self.reverse = QCheckBox('方向反转', self)
        self.home_readout = QLabel(f'回中：{center} μs', self)
        self.capture_button = QPushButton('记录回中', self)
        self.capture_button.setToolTip('等待移动完成后，记录本轴的控制脉宽；此操作不会移动舵机。')
        self.slider.valueChanged.connect(self.spin.setValue)
        self.spin.valueChanged.connect(self.slider.setValue)
        self.spin.valueChanged.connect(lambda _: self.changed.emit())
    def set_range(self, low, high):
        self.slider.setRange(low, high)
        self.spin.setRange(low, high)
    def nudge(self, delta):
        self.spin.setValue(self.spin.value() + (-delta if self.reverse.isChecked() else delta))

class Window(QMainWindow):
    def __init__(self, calibration_path=None):
        super().__init__()
        self.setWindowTitle('云台控制')
        self.resize(760, 520)
        self.calibration_path = (Path(__file__).resolve().parent / 'home_calibration.json'
                                 if calibration_path is None else
                                 Path(calibration_path) if calibration_path is not False else None)
        self.home = HomeCalibration()
        self.demo_home_modified = False
        self.pending_control = None
        self.worker = None
        self.connected = self.enabled = self.dirty = self.syncing = False
        self.last_status = None
        self.last_status_time = None
        self.last_imu=None
        self.last_imu_time=None
        self.imu_rate=None
        self.imu_rate_reference=None
        self.imu_note='未连接'
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(10, 10, 10, 6)
        connection = QHBoxLayout()
        self.ports = QComboBox()
        self.refresh_button = QPushButton('刷新')
        self.connect_button = QPushButton('连接')
        connection.addWidget(QLabel('串口'))
        connection.addWidget(self.ports, 1)
        self.ports.setToolTip('电脑与开发板：115200 / 8N1')
        connection.addWidget(self.refresh_button)
        connection.addWidget(self.connect_button)
        layout.addLayout(connection)
        self.badge = QLabel('未连接')
        self.statusBar().addPermanentWidget(self.badge)
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)
        control_page, settings_page, log_page = QWidget(), QWidget(), QWidget()
        self.tabs.addTab(control_page, '控制')
        self.tabs.addTab(settings_page, '设置')
        self.tabs.addTab(log_page, '日志')
        control_layout = QVBoxLayout(control_page)
        control_layout.setSpacing(12)
        imu_box=QGroupBox('姿态角')
        imu_layout=QGridLayout(imu_box)
        self.angle_labels=[]
        for column,name in enumerate(('横滚 X','俯仰 Y','航向 Z')):
            imu_layout.addWidget(QLabel(name),0,column)
            label=QLabel('—')
            label.setObjectName('angle')
            imu_layout.addWidget(label,1,column)
            self.angle_labels.append(label)
        self.imu_badge=QLabel('未连接')
        self.imu_details=QLabel('未连接')
        self.imu_details.setWordWrap(True)
        imu_layout.addWidget(self.imu_badge,2,0,1,3)
        control_layout.addWidget(imu_box)
        axes = QHBoxLayout()
        self.pan = AxisControl('水平', '下方舵机：J11 第 1 脚',
                               PAN_MIN_US, PAN_MAX_US, PAN_CENTER_US)
        self.tilt = AxisControl('俯仰', '上方舵机：J11 第 2 脚',
                                TILT_MIN_US, TILT_MAX_US, TILT_CENTER_US)
        axes.addWidget(self.pan)
        axes.addWidget(self.tilt)
        control_layout.addLayout(axes)
        actions = QHBoxLayout()
        self.enable_button = QPushButton('启用舵机')
        self.enable_button.setToolTip('启用时从 ImageCorrection 固件当前输出位置继续运行。')
        self.center_button = QPushButton('回中')
        self.stop_button = QPushButton('停止')
        self.stop_button.setToolTip('停止移动，保持当前位置。')
        self.disable_button = QPushButton('停用舵机')
        self.disable_button.setToolTip('关闭 PWM，舵机可能失去保持力。')
        for button in (self.enable_button, self.center_button, self.stop_button, self.disable_button):
            actions.addWidget(button)
        self.direction_box = QGroupBox('微调')
        grid = QGridLayout(self.direction_box)
        self.nudge_step = QSpinBox()
        self.nudge_step.setRange(1, PAN_MAX_US-PAN_MIN_US)
        self.nudge_step.setValue(10)
        self.nudge_step.setSuffix(' μs')
        self.nudge_step.setToolTip('每次点击方向按钮调整的脉宽。')
        self.nudge_buttons = {}
        for text, row, col, axis, delta in (
            ('←', 0, 1, self.pan, -1), ('→', 0, 2, self.pan, 1),
            ('↑', 0, 4, self.tilt, 1), ('↓', 0, 5, self.tilt, -1),
        ):
            button = QPushButton(text)
            button.clicked.connect(lambda checked=False, a=axis, d=delta: a.nudge(d*self.nudge_step.value()))
            self.nudge_buttons[text] = button
            grid.addWidget(button, row, col)
        grid.addWidget(QLabel('水平'), 0, 0)
        grid.addWidget(QLabel('俯仰'), 0, 3)
        grid.addWidget(QLabel('步长'), 0, 6)
        grid.addWidget(self.nudge_step, 0, 7)
        self.direction_box.setToolTip('方向反转可在设置中修改。ImageCorrection 固件速度范围为 50–1000 μs/秒。')
        control_layout.addWidget(self.direction_box)
        self.speed = QComboBox()
        for step in (1, 2, 5, 10, 20):
            self.speed.addItem(f'{step*50} μs/秒', step)
        self.speed.setCurrentIndex(self.speed.findData(10))
        self.speed.setToolTip('控制脉宽的变化速度；实际转速受舵机限制。')
        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel('移动速度'))
        speed_row.addWidget(self.speed)
        speed_row.addStretch()
        control_layout.addLayout(speed_row)
        control_layout.addLayout(actions)
        control_layout.addStretch()
        settings_layout = QVBoxLayout(settings_page)
        settings = QGroupBox('舵机设置')
        settings_grid = QGridLayout(settings)
        self.pan_low, self.pan_high = QSpinBox(), QSpinBox()
        self.tilt_low, self.tilt_high = QSpinBox(), QSpinBox()
        for low, high, minimum, maximum, center in (
                (self.pan_low,self.pan_high,PAN_MIN_US,PAN_MAX_US,PAN_CENTER_US),
                (self.tilt_low,self.tilt_high,TILT_MIN_US,TILT_MAX_US,TILT_CENTER_US)):
            low.setRange(minimum,center-1)
            high.setRange(center+1,maximum)
            low.setValue(minimum)
            high.setValue(maximum)
            for spin in (low,high):
                spin.setSuffix(' μs')
                spin.setSingleStep(50)
                spin.setToolTip('本轴范围已按实物记录；可关闭 PWM 后缩小调试范围。')
        settings_grid.addWidget(QLabel('水平范围'), 0, 0)
        settings_grid.addWidget(self.pan_low, 0, 1)
        settings_grid.addWidget(self.pan_high, 0, 2)
        settings_grid.addWidget(QLabel('俯仰范围'), 1, 0)
        settings_grid.addWidget(self.tilt_low, 1, 1)
        settings_grid.addWidget(self.tilt_high, 1, 2)
        for row, name, axis in ((2, '水平', self.pan), (5, '俯仰', self.tilt)):
            settings_grid.addWidget(QLabel(name), row, 0)
            settings_grid.addWidget(axis.reverse, row, 1, 1, 2)
            settings_grid.addWidget(axis.home_readout, row+1, 0, 1, 2)
            settings_grid.addWidget(axis.capture_button, row+1, 2)
            settings_grid.addWidget(axis.readout, row+2, 0, 1, 3)
        settings_layout.addWidget(settings)
        sensor_settings = QGroupBox('传感器状态')
        sensor_layout = QVBoxLayout(sensor_settings)
        sensor_layout.addWidget(self.imu_details)
        sensor_layout.addWidget(QLabel('TX：J11.3　供电：3.3V　GND：J11.30'))
        sensor_note = QLabel('角度随模块安装方向变化；六轴模块的航向角会漂移。')
        sensor_note.setWordWrap(True)
        sensor_layout.addWidget(sensor_note)
        settings_layout.addWidget(sensor_settings)
        settings_layout.addStretch()
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(100)
        log_layout = QVBoxLayout(log_page)
        log_layout.addWidget(self.log)
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.toggle_connection)
        self.enable_button.clicked.connect(lambda: self.control(ENABLE))
        self.disable_button.clicked.connect(lambda: self.control(DISABLE))
        self.stop_button.clicked.connect(lambda: self.control(STOP))
        self.center_button.clicked.connect(self.center)
        self.pan.changed.connect(self.mark_dirty)
        self.tilt.changed.connect(self.mark_dirty)
        self.pan.capture_button.clicked.connect(lambda: self.capture_home('pan'))
        self.tilt.capture_button.clicked.connect(lambda: self.capture_home('tilt'))
        for spin in (self.pan_low,self.pan_high,self.tilt_low,self.tilt_high):
            spin.valueChanged.connect(self.change_range)
        self.speed.currentIndexChanged.connect(self.mark_dirty)
        self.load_home()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.send_latest)
        self.timer.timeout.connect(self.refresh_imu)
        self.timer.start(100)
        self.refresh_ports()
        self.update_controls()
        self.message('请选择串口。')
    def message(self, text):
        self.log.appendPlainText(time.strftime('%H:%M:%S') + '  ' + text)
        self.statusBar().showMessage(text, 6000)
    def on_imu(self,status):
        now=time.monotonic()
        if self.imu_rate_reference is None:
            self.imu_rate_reference=(now,status.sample_count)
        else:
            previous_time,previous_count=self.imu_rate_reference
            if now-previous_time>=1:
                self.imu_rate=((status.sample_count-previous_count)&0xffffffff)/(now-previous_time)
                self.imu_rate_reference=(now,status.sample_count)
        self.last_imu=status
        self.last_imu_time=now
        self.refresh_imu()
    def on_imu_unavailable(self,text):
        self.last_imu=None
        self.imu_note=text
        self.message(text)
        self.refresh_imu()
    def refresh_imu(self):
        status=self.last_imu
        elapsed=time.monotonic()-self.last_imu_time if self.last_imu_time is not None else float('inf')
        effective_age=status.age_ms+elapsed*1000 if status else float('inf')
        live=self.connected and status is not None and not status.result and status.fresh and effective_age<500
        for label,value in zip(self.angle_labels,status.angles if live else (None,)*3):
            label.setText(f'{value:+.2f}°' if value is not None else '—')
        if not self.connected:
            self.imu_badge.setText('未连接')
            return
        if status is None:
            self.imu_badge.setText(self.imu_note)
            return
        state='在线' if live else '数据超时' if status.has_sample else '未收到有效角度'
        self.imu_badge.setText(f'离线模拟数据 · {state}' if self.worker and self.worker.demo else state)
        baud=str(status.baud) if status.baud else '未识别'
        age=f'{round(effective_age)} ms' if status.has_sample else '—'
        rate=f'{self.imu_rate:.1f} Hz' if live and self.imu_rate is not None else '—'
        self.imu_details.setText(f'模块波特率 {baud} · 数据年龄 {age} · 收到角度 {rate} · '
                                 f'有效角度帧 {status.sample_count} · RX 字节 {status.byte_count} · 校验失败 {status.bad_count}')
    def load_home(self):
        if self.calibration_path and self.calibration_path.exists():
            try:
                self.home = HomeCalibration.load(self.calibration_path)
                self.syncing = True
                for name in ('pan_low','pan_high','tilt_low','tilt_high'):
                    getattr(self,name).setValue(getattr(self.home,name+'_us'))
                self.syncing = False
                self.message('已加载回中设置。')
            except (OSError, ValueError, TypeError) as exc:
                self.message(f'回中配置未加载，使用水平 1500 / 俯仰 1500 μs：{exc}')
        self.refresh_home_labels()
    def refresh_home_labels(self):
        for axis, value, recorded in ((self.pan, self.home.pan_us, self.home.pan_recorded),
                                     (self.tilt, self.home.tilt_us, self.home.tilt_recorded)):
            axis.home_readout.setText(f'回中值：{value} μs（{"已记录" if recorded else "未校准"}）')
    def can_capture_home(self):
        status = self.last_status
        return bool(self.worker and self.connected and self.enabled and status and self.status_within_limits() and
                    self.last_status_time is not None and time.monotonic() - self.last_status_time <= 0.75 and
                    not status.result and not status.moving and
                    not self.dirty and self.pending_control is None and
                    status.pan_current == status.pan_target == self.pan.spin.value() and
                    status.tilt_current == status.tilt_target == self.tilt.spin.value())
    def capture_home(self, axis):
        if not self.can_capture_home():
            self.message('等待移动完成后再记录回中。')
            return
        value = getattr(self.last_status, f'{axis}_current')
        candidate = replace(self.home, **{f'{axis}_us': value, f'{axis}_recorded': True},
                            pan_low_us=self.pan_low.value(),pan_high_us=self.pan_high.value(),
                            tilt_low_us=self.tilt_low.value(),tilt_high_us=self.tilt_high.value())
        try:
            candidate.validate()
            if self.worker.demo is None and self.calibration_path:
                candidate.save(self.calibration_path)
        except (OSError, ValueError) as exc:
            self.message(f'回中值未保存：{exc}')
            return
        self.home = candidate
        if self.worker.demo is not None: self.demo_home_modified = True
        self.refresh_home_labels()
        name = '水平' if axis == 'pan' else '俯仰'
        suffix = '离线' if self.worker.demo is not None or not self.calibration_path else '已保存'
        self.message(f'{name}回中：{value} μs（{suffix}）。')
        self.update_controls()
    def refresh_ports(self):
        previous = self.ports.currentData()
        self.ports.clear()
        self.ports.addItem('离线演示', DEMO)
        for port in list_ports.comports():
            self.ports.addItem(f'{port.device} · {port.description}', port.device)
        index = self.ports.findData(previous)
        if index >= 0: self.ports.setCurrentIndex(index)
    def toggle_connection(self):
        if self.worker:
            self.disconnect()
            return
        self.dirty = False
        self.last_status = None
        self.last_status_time = None
        self.pending_control = None
        self.last_imu=None
        self.last_imu_time=None
        self.imu_rate=None
        self.imu_rate_reference=None
        self.imu_note='等待数据…'
        if self.ports.currentData() != DEMO and self.demo_home_modified:
            self.home = HomeCalibration()
            self.load_home()
            self.demo_home_modified = False
        self.worker = SerialWorker(self.ports.currentData())
        self.worker.ready.connect(self.on_ready)
        self.worker.update.connect(self.on_status)
        self.worker.error.connect(self.message)
        self.worker.imu_update.connect(self.on_imu)
        self.worker.imu_unavailable.connect(self.on_imu_unavailable)
        self.worker.finished.connect(self.on_finished)
        self.connect_button.setText('断开')
        self.ports.setEnabled(False)
        self.refresh_button.setEnabled(False)
        self.badge.setText('正在连接…')
        self.worker.start()
    def on_ready(self, source):
        self.connected = True
        self.message(f'{source} 已连接。')
        self.update_controls()
    def on_status(self, status, command):
        if command == self.pending_control: self.pending_control = None
        first_status = self.last_status is None
        newly_timed_out = status.timed_out and not (self.last_status and self.last_status.timed_out)
        self.last_status = status
        self.last_status_time = time.monotonic()
        self.enabled = status.enabled
        state = '移动中' if status.moving else '已启用'
        if not status.enabled: state = '舵机已停用'
        if status.timed_out: state = '通信超时，移动已停止'
        self.badge.setText(state)
        self.pan.readout.setText(f'目标 {status.pan_target} μs　输出 {status.pan_current} μs')
        self.tilt.readout.setText(f'目标 {status.tilt_target} μs　输出 {status.tilt_current} μs')
        if not self.status_within_limits() and first_status:
            self.message('板端控制值超出已记录范围；请关闭 PWM 并下载双轴校准版后再移动。')
        if (first_status or command in (ENABLE, DISABLE, STOP) or newly_timed_out) and self.status_within_limits():
            self.syncing = True
            # Preserve both board targets before an explicit one-axis command.
            # Widen display limits to already-active valid pulses; send no command.
            if first_status:
                for name in ('pan','tilt'):
                    low,high = getattr(self,name+'_low'),getattr(self,name+'_high')
                    low.setValue(min(low.value(),getattr(status,name+'_target'),getattr(status,name+'_current')))
                    high.setValue(max(high.value(),getattr(status,name+'_target'),getattr(status,name+'_current')))
            self.pan.spin.setValue(status.pan_target)
            self.tilt.spin.setValue(status.tilt_target)
            self.syncing = False
            self.dirty = False
        self.update_controls()
    def on_finished(self):
        if self.worker and self.worker.isRunning(): return
        self.worker = None
        self.connected = self.enabled = self.dirty = False
        self.pending_control = None
        self.last_status_time = None
        self.connect_button.setText('连接')
        self.ports.setEnabled(True)
        self.refresh_button.setEnabled(True)
        self.badge.setText('未连接')
        self.refresh_imu()
        self.update_controls()
    def disconnect(self):
        if self.worker:
            self.worker.requestInterruption()
            self.worker.wait(1500)
            if self.worker.isRunning():
                self.message('正在等待串口退出…')
                return
            self.on_finished()
    def update_controls(self):
        self.enable_button.setEnabled(self.connected and not self.enabled)
        self.disable_button.setEnabled(self.connected and self.enabled)
        position_ready = self.connected and self.enabled and self.status_within_limits()
        for control in (self.pan, self.tilt, self.center_button, self.direction_box):
            control.setEnabled(position_ready)
        self.stop_button.setEnabled(self.connected and self.enabled)
        capture_ready = self.can_capture_home()
        self.pan.capture_button.setEnabled(capture_ready)
        self.tilt.capture_button.setEnabled(capture_ready)
        self.center_button.setEnabled(position_ready and self.home_in_range())
        self.center_button.setToolTip('调试范围须包含两个轴的回中值；此按钮按当前速度移动到回中位置。')
        for spin in (self.pan_low,self.pan_high,self.tilt_low,self.tilt_high):
            spin.setEnabled(not self.enabled)
    def status_within_limits(self):
        status = self.last_status
        return bool(status and within_axis_limits(status.pan_target,status.tilt_target) and
                    within_axis_limits(status.pan_current,status.tilt_current))
    def home_in_range(self):
        return (self.pan_low.value() <= self.home.pan_us <= self.pan_high.value() and
                self.tilt_low.value() <= self.home.tilt_us <= self.tilt_high.value())
    def control(self, command):
        self.dirty = False
        if self.worker and self.connected:
            self.pending_control = command
            self.worker.post(command)
            self.update_controls()
    def center(self):
        if not (self.connected and self.enabled and self.status_within_limits()): return
        if not self.home_in_range():
            self.message('调试范围不包含已记录的回中值，请先关闭 PWM 并调整范围。')
            return
        self.pan.spin.setValue(self.home.pan_us)
        self.tilt.spin.setValue(self.home.tilt_us)
        self.mark_dirty()
    def mark_dirty(self, *args):
        if not self.syncing:
            self.dirty = True
            self.update_controls()
    def change_range(self, *args):
        self.pan.set_range(self.pan_low.value(), self.pan_high.value())
        self.tilt.set_range(self.tilt_low.value(), self.tilt_high.value())
        self.update_controls()
    def send_latest(self):
        if self.dirty and self.worker and self.connected and self.enabled and self.status_within_limits():
            self.dirty = False
            self.worker.post(SET, self.pan.spin.value(), self.tilt.spin.value(), self.speed.currentData())
        self.update_controls()
    def closeEvent(self, event):
        self.disconnect()
        if self.worker and self.worker.isRunning(): event.ignore()
        else:
            self.timer.stop()
            event.accept()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--preview', help='Render an offline demo PNG and run UI smoke checks')
    parser.add_argument('--calibration-file', type=Path, help='Override the project-local home calibration path')
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    # The Qt offscreen renderer has no system font database on Windows.
    # Load installed fonts explicitly for readable Chinese and identical previews.
    font_directory = Path(os.environ.get('WINDIR', 'C:/Windows')) / 'Fonts'
    for filename in ('msyh.ttc', 'msyhbd.ttc', 'segoeui.ttf'):
        font_path = font_directory / filename
        if font_path.exists(): QFontDatabase.addApplicationFont(str(font_path))
    app.setFont(QFont('Microsoft YaHei UI', 9))
    app.setStyle('Fusion')
    app.styleHints().setColorScheme(Qt.ColorScheme.Light)
    app.processEvents()
    app.setPalette(app.style().standardPalette())
    app.setStyleSheet('''
        QLabel#angle { font-size: 22px; }
    ''')
    window = Window(calibration_path=False if args.preview else args.calibration_file)
    window.show()
    result = [0]
    if args.preview:
        # Force the offline backend even if physical COM ports exist.
        window.ports.setCurrentIndex(window.ports.findData(DEMO))
        window.connect_button.click()
        QTimer.singleShot(180, window.enable_button.click)
        QTimer.singleShot(320, lambda: (window.pan.spin.setValue(1520), window.tilt.spin.setValue(1530)))
        QTimer.singleShot(720, window.pan.capture_button.click)
        QTimer.singleShot(720, window.tilt.capture_button.click)
        QTimer.singleShot(900, lambda: (window.pan.spin.setValue(1510), window.tilt.spin.setValue(1540)))
        QTimer.singleShot(1250, window.center_button.click)
        def finish_preview():
            status = window.last_status
            if not (window.connected and status and status.enabled and
                    status.pan_target == 1520 and status.tilt_target == 1530 and
                    window.pan.slider.value() == 1520 and window.tilt.slider.value() == 1530 and
                    window.home.pan_us == 1520 and window.home.tilt_us == 1530 and
                    window.home.pan_recorded and window.home.tilt_recorded and
                    status.pan_current == 1520 and status.tilt_current == 1530 and
                    window.last_imu and window.last_imu.fresh and
                    '离线模拟数据' in window.imu_badge.text() and
                    all(label.text()!='—' for label in window.angle_labels)):
                print(f'FAIL GUI smoke test connected={window.connected} status={status} home={window.home} imu={window.last_imu} badge={window.imu_badge.text()} log={window.log.toPlainText()}', flush=True)
                result[0] = 1
            else:
                path = Path(args.preview).resolve()
                path.parent.mkdir(parents=True, exist_ok=True)
                if not window.grab().save(str(path)):
                    result[0] = 1
                else: print('PASS GUI offline IMU telemetry and existing calibrated servo controls; PNG render', flush=True)
            window.close()
        QTimer.singleShot(1800, finish_preview)
    app.exec()
    # Destroy child widgets while QApplication still exists on Windows.
    window.deleteLater()
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    return result[0]

if __name__ == '__main__':
    sys.exit(main())
