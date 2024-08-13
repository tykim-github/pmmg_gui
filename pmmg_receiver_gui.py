import sys
import numpy as np
import serial
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLineEdit, QLabel, QHBoxLayout
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from screeninfo import get_monitors

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import serial.tools.list_ports
import numpy.linalg as LA

class SerialReader(QThread):
    data_processed = pyqtSignal(np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray)  # 시간, 쿼터니언, 압력 데이터를 전달할 시그널
    line_read = pyqtSignal(str)  # 읽은 줄을 전달할 시그널
    state_changed = pyqtSignal(str)
    initial_angles_calculated = pyqtSignal(float, float)  # 초기 각도를 전달할 시그널

    def __init__(self, filename, parent=None):
        super().__init__(parent)
        self.filename = filename
        self.running = False
        self.collecting = False
        self.data_buffer = []
        self.session_index = 1
        self.initial_knee_angle = None
        self.initial_ankle_angle = None
        self.initial_quaternions = None

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
                        if file:
                            file.close()  # 현재 파일이 열려있다면 닫기
                        filename = f"{self.filename}_{str(self.session_index).zfill(2)}.txt"
                        file = open(filename, "w")  # 새 파일 생성
                        self.session_index += 1
                        self.collecting = True
                    elif state == "105" and self.collecting:
                        self.process_data()
                        self.collecting = False
                        if file:
                            file.close()  # 데이터 수집 완료 후 파일 닫기
                            file = None
                    elif self.collecting:
                        self.data_buffer.append(line)
                        if state != "102" and state != "103" and state != "104" and state != "105":
                            if file:
                                file.write(line + "\n")  # 데이터를 파일에 계속 기록
        finally:
            if file:
                file.close()  # 프로그램 종료시 열려있는 파일 닫기
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
        self.initial_knee_angle = self.calculate_angle(self.q_ti, self.q_si)
        self.initial_ankle_angle = self.calculate_angle(self.q_si, self.q_fi)

        self.initial_quaternions = (self.q_ti, self.q_si, self.q_fi)

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

        q_k = np.array([self.q_mult(self.q_mult(q_shank[i], self.q_conj(self.q_si)),
                       self.q_mult(self.q_ti, self.q_conj(q_thigh[i])))
                    for i in range(len(data))])

        q_a = np.array([self.q_mult(self.q_mult(q_foot[i], self.q_conj(self.q_fi)),
                       self.q_mult(self.q_si, self.q_conj(q_shank[i])))
                    for i in range(len(data))])
        
        knee_angle = np.degrees([self.q_angle(q) for q in q_k])
        ankle_angle = np.degrees([self.q_angle(q) for q in q_a])
        
        # 쿼터니안 사이의 각도를 계산하여 저장
        # knee_angle = np.array([self.calculate_angle(q1, q2) for q1, q2 in zip(Quaternions[:, :4], Quaternions[:, 4:8])])
        # ankle_angle = np.array([self.calculate_angle(q1, q2) for q1, q2 in zip(Quaternions[:, 4:8], Quaternions[:, 8:12])])

        # 초기 각도를 빼줌
        # if self.initial_knee_angle is not None:
        #     knee_angle -= self.initial_knee_angle
        # if self.initial_ankle_angle is not None:
        #     ankle_angle -= self.initial_ankle_angle

        self.data_buffer = []
        self.data_processed.emit(Time, Quaternions, Pressure, knee_angle, ankle_angle)

    # 쿼터니언 곱셈 함수
    def q_mult(self, q1, q2):
        q1 = q1 / np.linalg.norm(q1)  # 쿼터니언 정규화
        q2 = q2 / np.linalg.norm(q2)  # 쿼터니언 정규화
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        return np.array([w, x, y, z])

    # 쿼터니언 켤레 함수
    def q_conj(self, q):
        w, x, y, z = q
        return np.array([w, -x, -y, -z])

    # 회전 각도를 계산하는 함수
    def q_angle(self, q):
        w = q[0]
        return 2 * np.arccos(w)

    def calculate_angle(self, q1, q2):
        q1 = np.asarray(q1)
        q2 = np.asarray(q2)
        
        dot_product = np.dot(q1, q2)
        dot_product = dot_product / (np.linalg.norm(q1) * np.linalg.norm(q2))
        angle = 2 * np.arccos(np.clip(np.abs(dot_product), -1.0, 1.0))
        angle_degrees = np.degrees(angle)
        
        return angle_degrees

    def stop(self):
        self.running = False

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

        # 모니터의 물리적 크기 (인치 단위)
        diagonal_in_inches = (monitor.width_mm**2 + monitor.height_mm**2)**0.5 / 25.4

        # 모니터의 가로세로 비율에 따라 기본 폰트 크기를 계산
        base_font_size_inch = diagonal_in_inches * 0.01  # 대략적인 폰트 크기 비율
        base_font_size_px = int(base_font_size_inch * qt_dpi)  # DPI를 곱하여 픽셀 단위로 변환

        # Matplotlib 설정
        plt.rcParams['figure.dpi'] = qt_dpi  # Matplotlib의 DPI를 PyQt의 DPI에 맞춥니다.
        
        # Adjust the scale factor inversely for Matplotlib fonts based on DPI
        scale_factor = 96 / qt_dpi
        plt.rcParams['font.size'] = base_font_size_px * scale_factor  # 계산된 픽셀 크기를 직접 설정
        plt.rcParams['axes.labelsize'] = base_font_size_px * scale_factor
        plt.rcParams['xtick.labelsize'] = int(base_font_size_px * 0.8 * scale_factor)
        plt.rcParams['ytick.labelsize'] = int(base_font_size_px * 0.8 * scale_factor)
        plt.rcParams['legend.fontsize'] = int(base_font_size_px * 0.8 * scale_factor)

        self.setGeometry(int(screen_width * 0.15), int(screen_height * 0.1), int(screen_width * 0.7), int(screen_height * 0.8))

        layout = QVBoxLayout()
        base_font_size = int(min(screen_width, screen_height) * 0.015)  # 해상도의 1.5%를 폰트 크기로 설정
        self.fig = plt.figure(figsize=(15, 15))  # Figure 생성
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)

        # Grid definition
        self.axes = [self.fig.add_subplot(3, 3, i) for i in range(1, 4)]  # 첫 번째 행 (3개의 그래프)
        self.axes.append(self.fig.add_subplot(3, 1, 2))  # 두 번째 행
        self.axes.append(self.fig.add_subplot(3, 1, 3))  # 세 번째 행

        self.status_label = QLabel("Status: None")  # 초기 상태 메시지
        self.status_label.setAlignment(Qt.AlignCenter)  # 텍스트 가운데 정렬
        self.status_label.setStyleSheet(f"font-size: {int(base_font_size_px*1.2)}px; font-weight: bold;")  # 글씨 키우고 bold 처리
        layout.addWidget(self.status_label)  

        self.initial_angles_label = QLabel("Initial Angles: Not calculated")
        self.initial_angles_label.setAlignment(Qt.AlignCenter)
        self.initial_angles_label.setStyleSheet(f"font-size: {base_font_size_px}px; font-weight: bold;")
        layout.addWidget(self.initial_angles_label)

        instruction_label = QLabel("Enter the output trial name:")
        instruction_label.setStyleSheet(f"font-size: {base_font_size_px}px;")
        layout.addWidget(instruction_label)

        self.filename_input = QLineEdit(self)
        layout.addWidget(self.filename_input)

        buttons_layout = QHBoxLayout()
        self.start_btn = QPushButton('START', self)
        self.start_btn.setStyleSheet(f"font-size: {base_font_size_px}px;")
        self.start_btn.clicked.connect(self.start_reading)
        buttons_layout.addWidget(self.start_btn)

        end_btn = QPushButton('END', self)
        end_btn.setStyleSheet(f"font-size: {base_font_size_px}px;")
        end_btn.clicked.connect(self.close_app)
        buttons_layout.addWidget(end_btn)

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

    def close_app(self):
        if self.thread and self.thread.isRunning():
            self.thread.stop()
            self.thread.wait()
        self.start_btn.setEnabled(True)
        self.filename_input.setEnabled(True)

    def plot_data(self, Time, Quaternions, Pressure, knee_angle, ankle_angle):
        # 모든 그래프를 초기화
        for ax in self.axes:
            ax.clear()

        Time_sec = Time / 1000.0

        # 색상 설정
        colors = ['black', 'red', 'green', 'blue']
        labelcomponent = ['w', 'x', 'y', 'z']

        # 첫 번째 행: 각 IMU의 쿼터니언 데이터 플로팅
        for i in range(3):  # 각 IMU에 대하여
            for j in range(4):  # 각 쿼터니언 컴포넌트(w, x, y, z)
                self.axes[i].plot(Time_sec, Quaternions[:, i*4+j], color=colors[j], label=labelcomponent[j])
            self.axes[i].set_xlabel("Time (sec)")
            self.axes[i].set_ylabel("Quaternion")
            self.axes[i].legend()
            self.axes[i].grid(True)

        # 두 번째 행: 쿼터니안 사이의 각도 플로팅
        self.axes[3].plot(Time_sec, knee_angle, color='darkred', label='Knee joint')
        self.axes[3].plot(Time_sec, ankle_angle, color='darkblue', label='Ankle joint')
        self.axes[3].set_xlabel("Time (sec)")
        self.axes[3].set_ylabel("Angle (deg)")
        self.axes[3].legend()
        self.axes[3].grid(True)

        # 세 번째 행: 압력 데이터를 짙은 빨강색으로 플로팅
        self.axes[4].plot(Time_sec, Pressure, color='black', label='pMMG')
        self.axes[4].set_xlabel("Time (sec)")
        self.axes[4].set_ylabel("Pressure (kPa)")
        self.axes[4].legend()
        self.axes[4].grid(True)

        # 캔버스에 변경사항 반영하여 다시 그리기
        self.canvas.draw()

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

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SerialDataSaver()
    sys.exit(app.exec_())
