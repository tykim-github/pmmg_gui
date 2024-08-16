import sys
import numpy as np
import serial
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLineEdit, QLabel, QHBoxLayout, QFileDialog
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from screeninfo import get_monitors

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import serial.tools.list_ports
import numpy.linalg as LA
from scipy.signal import butter, filtfilt


class FileHandler:
    def __init__(self, filename_prefix):
        self.filename_prefix = filename_prefix
        self.session_index = 1
        self.file = None

    def open_new_file(self, header_data):
        if self.file is not None:
            return # 이미 파일이 열려 있다면 새로 열지 않음
        filename = f"{self.filename_prefix}_{str(self.session_index).zfill(2)}.txt"
        self.file = open(filename, "w")
        # 헤더 데이터를 키-값 쌍으로 저장
        for key, value in header_data.items():
            value_str = ','.join(map(str, value)) if isinstance(value, (list, np.ndarray)) else str(value)
            self.file.write(f"{key}={value_str}\n")
        self.file.write("\n")  # 헤더와 본문을 구분하는 빈 줄 추가
        self.session_index += 1

    def write_line(self, line):
        if self.file:
            self.file.write(line + "\n")

    def close_file(self):
        if self.file:
            self.file.close()
            self.file = None

class SerialReader(QThread):
    data_processed = pyqtSignal(np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray)  # 시간, 쿼터니언, 압력 데이터를 전달할 시그널
    line_read = pyqtSignal(str)  # 읽은 줄을 전달할 시그널
    state_changed = pyqtSignal(str)
    initial_angles_calculated = pyqtSignal(float, float)  # 초기 각도를 전달할 시그널

    def __init__(self, filename, parent=None):
        super().__init__(parent)
        self.file_handler = FileHandler(filename)
        self.filename = filename
        self.running = False
        self.collecting = False
        self.data_buffer = []
        self.initial_knee_angle = None
        self.initial_ankle_angle = None
        self.initial_quaternions = None
        self.q_ti = None
        self.q_si = None
        self.q_fi = None


    def run(self):
        self.running = True
        for port in serial.tools.list_ports.comports():
            if "wch.cn" in port.manufacturer:
                self.com_port = port.device
        
        ser = serial.Serial(self.com_port, 115200, timeout=1)
        file = None

        try:
            while self.running:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    state, _ = line.split(',', 1)
                    if state != "0":
                        self.state_changed.emit(state)  # 상태 변경 시그널 발생
                    if state == "102":
                        self.data_buffer = []  # 초기화
                        self.collecting = True
                    elif state == "103" and self.collecting:
                        self.calculate_initial_state()
                        self.collecting = False
                    elif state == "104":
                        self.data_buffer = []  # 초기화
                        header_data = {
                            'q_ti': self.q_ti,
                            'q_si': self.q_si,
                            'q_fi': self.q_fi,
                            'initial_knee_angle': self.initial_knee_angle,
                            'initial_ankle_angle': self.initial_ankle_angle
                        }
                        self.file_handler.open_new_file(header_data)  # 새 파일 생성
                        self.collecting = True
                    elif state == "105" and self.collecting:
                        self.process_data()
                        self.collecting = False
                        self.file_handler.close_file()  # 데이터 수집 완료 후 파일 닫기
                    elif self.collecting:
                        self.data_buffer.append(line)
                        if state not in {"102", "103", "104", "105"}:
                            self.file_handler.write_line(line)  # 데이터를 파일에 계속 기록
        finally:
            self.file_handler.close_file()  # 프로그램 종료 시 열려있는 파일 닫기
            ser.close()

    def calculate_initial_state(self):
        if not self.data_buffer:
            return
        data = np.array([list(map(float, x.split(','))) for x in self.data_buffer])
        Quaternions = data[:, 2:14]

        # Normalize quaternions and calculate mean
        Quaternions /= LA.norm(Quaternions, axis=1)[:, np.newaxis]
        avg_quaternions = np.mean(Quaternions, axis=0)

        # IMU1, IMU2, IMU3의 평균 쿼터니언
        self.q_ti = avg_quaternions[:4]
        self.q_si = avg_quaternions[4:8]
        self.q_fi = avg_quaternions[8:12]

        # 무릎과 발목의 초기 각도 계산
        self.initial_knee_angle = Quaternion.calculate_angle(self.q_ti, self.q_si)
        self.initial_ankle_angle = Quaternion.calculate_angle(self.q_si, self.q_fi)

        self.initial_quaternions = (self.q_ti, self.q_si, self.q_fi)

        header_data = {
            'q_ti': self.q_ti,
            'q_si': self.q_si,
            'q_fi': self.q_fi,
            'initial_knee_angle': self.initial_knee_angle,
            'initial_ankle_angle': self.initial_ankle_angle
            # 필요시 추가적인 데이터를 여기에 포함
        }
        self.file_handler.open_new_file(header_data)

        # 초기 각도를 로그로 출력 또는 시그널로 전달
        print(f"Initial Knee Angle: {self.initial_knee_angle} degrees")
        print(f"Initial Ankle Angle: {self.initial_ankle_angle} degrees")
        self.initial_angles_calculated.emit(self.initial_knee_angle, self.initial_ankle_angle)

    def process_data(self):
        if not self.data_buffer:
            return
        data = np.array([list(map(float, x.split(','))) for x in self.data_buffer])
        Time = data[:, 1]
        Quaternions = data[:, 2:14]
        q_thigh = data[:, 2:6]
        q_shank = data[:, 6:10]
        q_foot  = data[:, 10:14]
        Pressure = data[:, 14]

        q_k = np.array([Quaternion.mult(Quaternion.mult(q_shank[i], Quaternion.conj(self.q_si)),
                       Quaternion.mult(self.q_ti, Quaternion.conj(q_thigh[i])))
                    for i in range(len(data))])

        q_a = np.array([Quaternion.mult(Quaternion.mult(q_foot[i], Quaternion.conj(self.q_fi)),
                       Quaternion.mult(self.q_si, Quaternion.conj(q_shank[i])))
                    for i in range(len(data))])
        
        knee_angle = np.degrees([Quaternion.angle(q) for q in q_k])
        ankle_angle = np.degrees([Quaternion.angle(q) for q in q_a])

        self.data_buffer = []
        self.data_processed.emit(Time, Quaternions, Pressure, knee_angle, ankle_angle)

    def stop(self):
        self.running = False

class Quaternion:
    @staticmethod
    def mult(q1, q2):
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        return np.array([w, x, y, z])

    @staticmethod
    def conj(q):
        w, x, y, z = q
        return np.array([w, -x, -y, -z])

    @staticmethod
    def angle(q):
        w = q[0]
        return 2 * np.arccos(w)

    @staticmethod
    def calculate_angle(q1, q2):
        q1 = np.asarray(q1)
        q2 = np.asarray(q2)
        
        dot_product = np.dot(q1, q2)
        dot_product = dot_product / (np.linalg.norm(q1) * np.linalg.norm(q2))
        angle = 2 * np.arccos(np.clip(np.abs(dot_product), -1.0, 1.0))
        angle_degrees = np.degrees(angle)
        
        return angle_degrees

class SerialDataSaver(QWidget):
    def __init__(self):
        super().__init__()
        self.thread = None
        self.init_ui()

    def init_ui(self):
        monitor = get_monitors()[0]
        screen_width = monitor.width
        screen_height = monitor.height

        screen = QApplication.primaryScreen()
        qt_dpi = screen.physicalDotsPerInch()
        diagonal_in_inches = (monitor.width_mm**2 + monitor.height_mm**2)**0.5 / 25.4

        base_font_size_inch = diagonal_in_inches * 0.009
        base_font_size_px = int(base_font_size_inch * qt_dpi)

        plt.rcParams['figure.dpi'] = qt_dpi

        scale_factor = 96 / qt_dpi
        plt.rcParams['font.size'] = base_font_size_px * scale_factor

        self.setGeometry(int(screen_width * 0.15), int(screen_height * 0.1), 
                         int(screen_width * 0.7), int(screen_height * 0.8))

        layout = QVBoxLayout()
        base_font_size = int(min(screen_width, screen_height) * 0.015)
        self.fig = plt.figure(figsize=(15, 15))
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)

        self.axes = [self.fig.add_subplot(3, 1, i+1) for i in range(3)]

        self.status_label = QLabel("Status: None")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet(f"font-size: {int(base_font_size_px*1.2)}px; font-weight: bold;")
        layout.addWidget(self.status_label)

        self.initial_angles_label = QLabel("Initial Angles: Not calculated")
        self.initial_angles_label.setAlignment(Qt.AlignCenter)
        self.initial_angles_label.setStyleSheet(f"font-size: {base_font_size_px}px; font-weight: bold;")
        layout.addWidget(self.initial_angles_label)

        instruction_label = QLabel("Enter the output trial name:")
        instruction_label.setStyleSheet(f"font-size: {base_font_size_px}px;")
        layout.addWidget(instruction_label)

        self.filename_input = QLineEdit(self)
        self.filename_input.setStyleSheet(f"font-size: {base_font_size_px}px;")
        layout.addWidget(self.filename_input)

        buttons_layout = QHBoxLayout()
        self.start_btn = QPushButton('START', self)
        self.start_btn.setStyleSheet(f"font-size: {int(base_font_size_px*1.4)}px;")
        self.start_btn.clicked.connect(self.start_reading)
        buttons_layout.addWidget(self.start_btn)

        end_btn = QPushButton('END', self)
        end_btn.setStyleSheet(f"font-size: {int(base_font_size_px*1.4)}px;")
        end_btn.clicked.connect(self.close_app)
        buttons_layout.addWidget(end_btn)

        load_btn = QPushButton('LOAD DATA', self)
        load_btn.setStyleSheet(f"font-size: {int(base_font_size_px*1.4)}px;")
        load_btn.clicked.connect(self.load_data)
        buttons_layout.addWidget(load_btn)

        layout.addLayout(buttons_layout)
        self.setLayout(layout)
        self.setWindowTitle('Spasticity Measurement Software')
        self.show()

    def start_reading(self):
        filename = self.filename_input.text()
        if not filename:
            return

        self.start_btn.setEnabled(False)
        self.filename_input.setEnabled(False)

        self.thread = SerialReader(filename, self)
        self.thread.data_processed.connect(self.plot_data)
        self.thread.line_read.connect(self.handle_line_read)
        self.thread.state_changed.connect(self.update_status_label)
        self.thread.initial_angles_calculated.connect(self.display_initial_angles)
        self.thread.start()

    def load_data(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self, "Load Data File", "", "Text Files (*.txt);;All Files (*)", options=options)
        if file_name:
            self._process_loaded_data(file_name)

    def _process_loaded_data(self, file_name):
        header_data = {}
        with open(file_name, 'r') as file:
            for line in file:
                line = line.strip()
                if not line:
                    break
                key, value_str = line.split('=')
                values = list(map(float, value_str.split(',')))
                header_data[key] = np.array(values) if len(values) > 1 else values[0]
            
            q_ti = header_data.get('q_ti')
            q_si = header_data.get('q_si')
            q_fi = header_data.get('q_fi')

            data = np.loadtxt(file, delimiter=',')
            Time = data[:, 1]
            Quaternions = data[:, 2:14]
            Pressure = data[:, 14]

            q_thigh = data[:, 2:6]
            q_shank = data[:, 6:10]
            q_foot = data[:, 10:14]

            q_k = Quaternion.mult(Quaternion.mult(q_shank, Quaternion.conj(q_si)),
                                  Quaternion.mult(q_ti, Quaternion.conj(q_thigh)))
            q_a = Quaternion.mult(Quaternion.mult(q_foot, Quaternion.conj(q_fi)),
                                  Quaternion.mult(q_si, Quaternion.conj(q_shank)))

            knee_angle = np.degrees([Quaternion.angle(q) for q in q_k])
            ankle_angle = np.degrees([Quaternion.angle(q) for q in q_a])

            self.plot_data(Time, Quaternions, Pressure, knee_angle, ankle_angle)

    def plot_data(self, Time, Quaternions, Pressure, knee_angle, ankle_angle):
        Time_sec = Time / 1000.0

        self._clear_axes()

        self._plot_joint_angles(Time_sec, knee_angle, ankle_angle)
        self._plot_angular_velocity(Time_sec, knee_angle, ankle_angle)
        self._plot_pressure_data(Time_sec, Pressure)

        self.canvas.draw()

    def _clear_axes(self):
        for ax in self.axes:
            ax.clear()

    def _plot_joint_angles(self, Time_sec, knee_angle, ankle_angle):
        self.axes[0].plot(Time_sec, knee_angle, color='darkred', label='Knee joint')
        self.axes[0].plot(Time_sec, ankle_angle, color='darkblue', label='Ankle joint')
        self.axes[0].set_xlabel("Time (sec)")
        self.axes[0].set_ylabel("Angle (deg)")
        self.axes[0].legend()
        self.axes[0].grid(True)

    def _plot_angular_velocity(self, Time_sec, knee_angle, ankle_angle):
        dt = np.diff(Time_sec)
        knee_velocity = np.diff(knee_angle) / dt
        ankle_velocity = np.diff(ankle_angle) / dt

        knee_velocity_filtered = self._apply_lowpass_filter(knee_velocity, 50, 1 / np.mean(dt))
        ankle_velocity_filtered = self._apply_lowpass_filter(ankle_velocity, 50, 1 / np.mean(dt))

        self.axes[1].plot(Time_sec[:-1], knee_velocity_filtered, color='darkred', label='Knee joint velocity (filtered)')
        self.axes[1].plot(Time_sec[:-1], ankle_velocity_filtered, color='darkblue', label='Ankle joint velocity (filtered)')
        self.axes[1].set_xlabel("Time (sec)")
        self.axes[1].set_ylabel("Angular Velocity (deg/sec)")
        self.axes[1].legend()
        self.axes[1].grid(True)

    def _apply_lowpass_filter(self, data, cutoff_freq, fs, order=2):
        nyquist_freq = 0.5 * fs
        normal_cutoff = cutoff_freq / nyquist_freq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return filtfilt(b, a, data)

    def _plot_pressure_data(self, Time_sec, Pressure):
        self.axes[2].plot(Time_sec, Pressure, color='black', label='pMMG')
        self.axes[2].set_xlabel("Time (sec)")
        self.axes[2].set_ylabel("Pressure (kPa)")
        self.axes[2].legend()
        self.axes[2].grid(True)

    def update_status_label(self, status_code):
        status_mapping = {
            "101": "Standby",
            "102": "Leg zeroing started",
            "103": "Leg zeroing stopped",
            "104": "Reading started",
            "105": "Reading stopped",
            "106": "Magnetometer calibrating",
            "201": "ERROR - Low voltage",
            "202": "ERROR - Sensor malfunction"
        }
        status_message = status_mapping.get(status_code, "ERROR - ???")
        self.status_label.setText(f"Status: {status_message}")

    def handle_line_read(self, line):
        pass

    def display_initial_angles(self, knee_angle, ankle_angle):
        self.initial_angles_label.setText(f"Initial Knee Angle: {knee_angle:.2f} degrees, Initial Ankle Angle: {ankle_angle:.2f} degrees")

    def close_app(self):
        if self.thread and self.thread.isRunning():
            self.thread.stop()
            self.thread.wait()
        self.start_btn.setEnabled(True)
        self.filename_input.setEnabled(True)