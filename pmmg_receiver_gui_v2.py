import os
import sys
import traceback
import numpy as np
import serial
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QPushButton, 
                             QLineEdit, QLabel, QHBoxLayout, QFileDialog, 
                             QCheckBox, QFormLayout, QMessageBox)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QPixmap
from screeninfo import get_monitors

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

import serial.tools.list_ports
from numpy.linalg import norm
from scipy.signal import butter, filtfilt
import csv

# to make this exe
# pyinstaller --onefile --noconsole --icon=legmus.ico --add-data="joint_angle_definition.png;." pmmg_receiver_gui_v2.py

PROGRAM_VERSION = "1.02.2"
# 1.01   Initial release
# 1.02   After 2 child patients
# 1.02.2 Minor bug change

class DataProcessor:
    def __init__(self, initial_knee_angle=None, initial_ankle_angle=None):
        self.initial_knee_angle = initial_knee_angle
        self.initial_ankle_angle = initial_ankle_angle
        self.q_ti = None
        self.q_si = None
        self.q_fi = None
        self.data_buffer = []

        # Attributes to store processed data
        self.Time = None
        self.knee_angle = None
        self.ankle_angle = None
        self.Pressure = None

    def initialize_from_header(self, header_data):
        """헤더 데이터에서 초기 각도를 설정합니다."""
        self.initial_knee_angle = header_data.get('initial_knee_angle')
        self.initial_ankle_angle = header_data.get('initial_ankle_angle')
        self.q_ti = header_data.get('q_ti')
        self.q_si = header_data.get('q_si')
        self.q_fi = header_data.get('q_fi')

    def process_data(self, data):
        """데이터를 처리하여 결과를 반환합니다."""
        self.Time = data[:, 1]  # Time을 객체 속성으로 저장
        Quaternions = data[:, 2:14]
        self.Pressure = data[:, 14]  # Pressure도 객체 속성으로 저장
        q_thigh = data[:, 2:6]
        q_shank = data[:, 6:10]
        q_foot = data[:, 10:14]

        q_k = np.array([Quaternion.mult(Quaternion.mult(q_shank[i], Quaternion.conj(self.q_si)),
                        Quaternion.mult(self.q_ti, Quaternion.conj(q_thigh[i])))
                        for i in range(len(data))])

        q_a = np.array([Quaternion.mult(Quaternion.mult(q_foot[i], Quaternion.conj(self.q_fi)),
                        Quaternion.mult(self.q_si, Quaternion.conj(q_shank[i])))
                        for i in range(len(data))])

        self.knee_angle = np.degrees([Quaternion.angle(q) for q in q_k]) + self.initial_knee_angle  # knee_angle 속성으로 저장
        self.ankle_angle = np.degrees([Quaternion.angle(q) for q in q_a]) + self.initial_ankle_angle  # ankle_angle 속성으로 저장

        return self.Time, Quaternions, self.Pressure, self.knee_angle, self.ankle_angle

    def calculate_initial_state(self):
        """초기 상태를 계산하여 쿼터니언을 설정합니다."""
        if not self.data_buffer:
            return
        data = np.array([list(map(float, x.split(','))) for x in self.data_buffer])
        Quaternions = data[:, 2:14]

        Quaternions /= norm(Quaternions, axis=1)[:, np.newaxis]
        avg_quaternions = np.mean(Quaternions, axis=0)

        self.q_ti = avg_quaternions[:4]
        self.q_si = avg_quaternions[4:8]
        self.q_fi = avg_quaternions[8:12]

        self.initial_quaternions = (self.q_ti, self.q_si, self.q_fi)

class FileHandler:
    def __init__(self, filename_prefix):
        self.filename_prefix = filename_prefix
        self.session_index = 1
        self.file = None

    def open_new_file(self, header_data):
        """Open a new file with the specified header data."""
        if self.file is not None:
            return  # Avoid opening a new file if one is already open
        filename = f"{self.filename_prefix}_{str(self.session_index).zfill(2)}.txt"
        self.file = open(filename, "w")
        # Save header data as key-value pairs
        for key, value in header_data.items():
            value_str = ','.join(map(str, value)) if isinstance(value, (list, np.ndarray)) else str(value)
            self.file.write(f"{key}={value_str}\n")
        self.file.write("\n")  # Add a blank line to separate the header from the body
        self.session_index += 1

    def write_line(self, line):
        """Write a single line to the file."""
        if self.file:
            self.file.write(line + "\n")

    def close_file(self):
        """Close the currently open file."""
        if self.file:
            self.file.close()
            self.file = None


class SerialReader(QThread):
    data_processed = pyqtSignal(np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray)
    line_read = pyqtSignal(str)
    state_changed = pyqtSignal(str)
    initial_angles_calculated = pyqtSignal(float, float)

    def __init__(self, filename, header_info, processor, parent=None):
        super().__init__(parent)
        self.file_handler = FileHandler(filename)
        self.header_info = header_info
        self.processor = processor
        self.filename = filename
        self.running = False
        self.collecting = False
        self.data_buffer = []

    def run(self):
        """Main loop for reading serial data."""
        self.running = True
        self.com_port = None

        for port in serial.tools.list_ports.comports():
            if "wch.cn" in port.manufacturer:
                self.com_port = port.device
                break  # USB 포트를 찾으면 루프를 종료합니다.

        if self.com_port is None:
            self.state_changed.emit("SerialFail")
            return  # 메서드를 종료하여 더 이상 코드를 실행하지 않도록 합니다.

        try:
            ser = serial.Serial(self.com_port, 115200, timeout=1)
        except serial.SerialException as e:
            self.state_changed.emit("SerialFail")
            return  # 메서드를 종료하여 더 이상 코드를 실행하지 않도록 합니다.

        try:
            while self.running:
                try:
                    line = ser.readline().decode('utf-8').strip()
                except (serial.SerialException, UnicodeDecodeError):
                    continue  # Skip invalid lines

                if line:
                    state, _ = line.split(',', 1)
                    if state != "0":
                        self.state_changed.emit(state)
                    if state == "102":
                        self.processor.data_buffer = []  # Reset buffer
                        self.collecting = True
                    elif state == "103" and self.collecting:
                        self.processor.calculate_initial_state()
                        self.collecting = False
                    elif state == "104":
                        self.processor.data_buffer = []  # Reset buffer
                        header_data = {
                            'q_ti': self.processor.q_ti,
                            'q_si': self.processor.q_si,
                            'q_fi': self.processor.q_fi,
                            'initial_knee_angle': self.processor.initial_knee_angle,
                            'initial_ankle_angle': self.processor.initial_ankle_angle,
                            **self.header_info  # Add patient info to header
                        }
                        self.file_handler.open_new_file(header_data)
                        self.collecting = True
                    elif state == "105" and self.collecting:
                        data = np.array([list(map(float, x.split(','))) for x in self.processor.data_buffer])
                        Time, Quaternions, self.processor.Pressure, knee_angle, ankle_angle = self.processor.process_data(data)
                        self.data_processed.emit(Time, Quaternions, self.processor.Pressure, knee_angle, ankle_angle)
                        self.collecting = False
                        self.file_handler.close_file()
                    elif self.collecting:
                        self.processor.data_buffer.append(line)
                        if state not in {"102", "103", "104", "105"}:
                            self.file_handler.write_line(line)
        finally:
            self.file_handler.close_file()
            if ser.is_open:
                ser.close()

    def stop(self):
        """Stop the serial reading thread."""
        self.running = False


def lowpass_filter(data, cutoff_freq, fs, order=5):
    nyquist_freq = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist_freq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

class Quaternion:
    @staticmethod
    def mult(q1, q2):
        """Multiply two quaternions."""
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
        """Return the conjugate of a quaternion."""
        w, x, y, z = q
        return np.array([w, -x, -y, -z])

    @staticmethod
    def angle(q):
        """Return the angle of a quaternion."""
        w = q[0]
        return 2 * np.arccos(w)



class SerialDataSaver(QWidget):
    def __init__(self):
        super().__init__()
        self.thread = None
        self.processor = DataProcessor()
        self.init_ui()

    def init_ui(self):
        try:
            """Initialize the user interface."""
            monitor = get_monitors()[0]
            screen_width = monitor.width
            screen_height = monitor.height
            
            screen = QApplication.primaryScreen()
            qt_dpi = screen.physicalDotsPerInch()

            diagonal_in_inches = (monitor.width_mm**2 + monitor.height_mm**2)**0.5 / 25.4
            base_font_size_inch = diagonal_in_inches * 0.007
            base_font_size_px = int(base_font_size_inch * qt_dpi)

            plt.rcParams['figure.dpi'] = qt_dpi
            scale_factor = 96 / qt_dpi
            plt.rcParams['font.size'] = base_font_size_px * scale_factor
            plt.rcParams['axes.labelsize'] = base_font_size_px * 0.6 * scale_factor
            plt.rcParams['xtick.labelsize'] = int(base_font_size_px * 0.6 * scale_factor)
            plt.rcParams['ytick.labelsize'] = int(base_font_size_px * 0.6 * scale_factor)
            plt.rcParams['legend.fontsize'] = int(base_font_size_px * 0.6 * scale_factor)

            self.setGeometry(int(screen_width * 0.15), int(screen_height * 0.1), int(screen_width * 0.7), int(screen_height * 0.8))

            layout = QHBoxLayout()  # Main layout for the entire window

            # Left side for plots
            plot_layout = QVBoxLayout()
            self.fig = plt.figure(figsize=(15, 15))
            self.canvas = FigureCanvas(self.fig)
            plot_layout.addWidget(self.canvas)

            # Add the navigation toolbar
            self.toolbar = NavigationToolbar(self.canvas, self)
            plot_layout.addWidget(self.toolbar)  # Add toolbar below the plot

            self.axes = [self.fig.add_subplot(3, 1, i+1) for i in range(3)]

            # Add version label below the plot
            version_label = QLabel(f"Spasticity Measurement Software v{PROGRAM_VERSION}")
            version_label.setAlignment(Qt.AlignCenter)
            version_label.setStyleSheet(f"font-size: {int(base_font_size_px*0.7)}px; color: gray;")
            plot_layout.addWidget(version_label)  # Add the version label to the plot layout

            layout.addLayout(plot_layout, 2)  # Allocate 60% to 70% of width for plots

            # Right side for user inputs and controls
            control_layout = QVBoxLayout()

            # Status label
            self.status_label = QLabel("Status: None")
            self.status_label.setAlignment(Qt.AlignCenter)
            self.status_label.setStyleSheet(f"font-size: {int(base_font_size_px*1.4)}px; font-weight: bold;")
            control_layout.addWidget(self.status_label)  

            # Add image between status label and input fields
            self.image_label = QLabel(self)
            if hasattr(sys, '_MEIPASS'):
                image_path = os.path.join(sys._MEIPASS, "joint_angle_definition.png")
            else:
                image_path = "joint_angle_definition.png"

            pixmap = QPixmap(image_path)
            max_image_width = int(screen_width * 0.3)
            max_image_height = int(screen_height * 0.3)
            scaled_pixmap = pixmap.scaled(max_image_width, max_image_height, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.image_label.setPixmap(scaled_pixmap)
            self.image_label.setAlignment(Qt.AlignCenter)
            control_layout.addWidget(self.image_label)

            # Patient information inputs
            patient_info_layout = QFormLayout()
            self.filename_input = QLineEdit(self)
            self.filename_input.setStyleSheet(f"font-size: {base_font_size_px}px;")
            patient_info_layout.addRow("Trial Name:", self.filename_input)

            self.patient_shank_diameter_input = QLineEdit(self)
            self.patient_shank_diameter_input.setStyleSheet(f"font-size: {base_font_size_px}px;")
            patient_info_layout.addRow("Shank Diameter (mm):", self.patient_shank_diameter_input)

            self.patient_band_elongation_input = QLineEdit(self)
            self.patient_band_elongation_input.setStyleSheet(f"font-size: {base_font_size_px}px;")
            patient_info_layout.addRow("Elongated Band Length (mm):", self.patient_band_elongation_input)

            self.initial_knee_angle_input = QLineEdit(self)
            self.initial_knee_angle_input.setStyleSheet(f"font-size: {base_font_size_px}px;")
            patient_info_layout.addRow("Initial Knee Angle (deg):", self.initial_knee_angle_input)

            self.initial_ankle_angle_input = QLineEdit(self)
            self.initial_ankle_angle_input.setStyleSheet(f"font-size: {base_font_size_px}px;")
            patient_info_layout.addRow("Initial Ankle Angle (deg):", self.initial_ankle_angle_input)

            self.recorder_name_input = QLineEdit(self)
            self.recorder_name_input.setStyleSheet(f"font-size: {base_font_size_px}px;")
            patient_info_layout.addRow("Recorder Name:", self.recorder_name_input)

            control_layout.addLayout(patient_info_layout)

            # Control buttons
            buttons_layout = QVBoxLayout()

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

            self.save_plot_btn = QPushButton('SAVE PLOT', self)
            self.save_plot_btn.setStyleSheet(f"font-size: {int(base_font_size_px*1.4)}px;")
            self.save_plot_btn.clicked.connect(self.save_plot)
            buttons_layout.addWidget(self.save_plot_btn)

            self.export_csv_btn = QPushButton('EXPORT CSV', self)
            self.export_csv_btn.setStyleSheet(f"font-size: {int(base_font_size_px*1.4)}px;")
            self.export_csv_btn.clicked.connect(self.export_csv)
            buttons_layout.addWidget(self.export_csv_btn)

            control_layout.addLayout(buttons_layout)
            layout.addLayout(control_layout, 1)

            self.setLayout(layout)
            self.setWindowTitle('Spasticity Measurement Software')
            self.show()
        except Exception as e:
            self.handle_exception(e)
        
    def handle_exception(self, e):
        """오류 발생 시 호출되어 오류 내용을 로그 파일에 기록하고 메시지 박스로 표시합니다."""
        with open("error_log.txt", "a") as f:
            traceback.print_exc(file=f)

        # 오류 메시지를 사용자에게 표시
        error_message = f"An error occurred: {str(e)}"
        QMessageBox.critical(self, "Error", error_message)

    def start_reading(self):
        try:
            """Start the serial reading thread."""
            self.file_name = self.filename_input.text()
            shank_diameter = self.patient_shank_diameter_input.text()
            band_elongation = self.patient_band_elongation_input.text()
            initial_knee_angle = float(self.initial_knee_angle_input.text())
            initial_ankle_angle = float(self.initial_ankle_angle_input.text())
            recorder_name = self.recorder_name_input.text()

            if not self.file_name:
                return

            self.start_btn.setEnabled(False)
            self.filename_input.setEnabled(False)

            header_info = {
                "Shank Diameter (mm)": shank_diameter,
                "Elongated Band Length(mm)": band_elongation,
                "Initial Knee Angle (deg)": initial_knee_angle,
                "Initial Ankle Angle (deg)": initial_ankle_angle,
                "Recorder Name": recorder_name
            }

            self.processor = DataProcessor(initial_knee_angle, initial_ankle_angle)

            self.thread = SerialReader(self.file_name, header_info, self.processor, self)
            self.thread.data_processed.connect(self.plot_data)
            self.thread.line_read.connect(self.handle_line_read)
            self.thread.state_changed.connect(self.update_status_label)
            self.thread.initial_angles_calculated.connect(self.display_initial_angles)
            self.thread.finished.connect(self.on_thread_finished)
            self.thread.start()

        except Exception as e:
            self.handle_exception(e)


    def load_data(self):
        try:
            options = QFileDialog.Options()
            self.file_name, _ = QFileDialog.getOpenFileName(self, "Load Data File", "", "Text Files (*.txt);;All Files (*)", options=options)
            if self.file_name:
                header_data = {}
                
                # 첫 번째로 파일을 열어 헤더 정보를 읽습니다.
                with open(self.file_name, 'r') as file:
                    for line in file:
                        line = line.strip()
                        if not line:
                            break  # End of header
                        key, value_str = line.split('=')
                        
                        # Try to convert the value to float(s), but if it fails, keep it as a string
                        try:
                            values = list(map(float, value_str.split(',')))
                            header_data[key] = np.array(values) if len(values) > 1 else values[0]
                        except ValueError:
                            # If the value cannot be converted to float, store it as a string
                            header_data[key] = value_str

                # DataProcessor 인스턴스를 초기화하고 헤더 정보를 로드합니다.
                self.processor.initialize_from_header(header_data)

                # 두 번째로 파일을 열어 데이터를 읽습니다.
                data = np.loadtxt(self.file_name, delimiter=',', skiprows=len(header_data) + 1)  # 헤더 줄을 건너뛰도록 설정

                # 데이터를 처리하고 플롯을 그립니다.
                Time, Quaternions, Pressure, knee_angle, ankle_angle = self.processor.process_data(data)
                self.plot_data(Time, Quaternions, Pressure, knee_angle, ankle_angle)
        except Exception as e:
            self.handle_exception(e)

    def plot_data(self, Time, Quaternions, Pressure, knee_angle, ankle_angle):
        """Plot the processed data."""
        for ax in self.axes:
            ax.clear()

        Time_sec = Time / 1000.0

        self.axes[0].plot(Time_sec, knee_angle, color='darkred', label='Knee joint')
        self.axes[0].plot(Time_sec, ankle_angle, color='darkblue', label='Ankle joint')
        self.axes[0].set_xlabel("Time (sec)")
        self.axes[0].set_ylabel("Angle (deg)")
        self.axes[0].legend()
        self.axes[0].grid(True)

        dt = 0.005
        knee_velocity = np.diff(knee_angle) / dt
        ankle_velocity = np.diff(ankle_angle) / dt

        fs = 1 / dt
        knee_velocity_filtered = lowpass_filter(knee_velocity, 13, fs)
        ankle_velocity_filtered = lowpass_filter(ankle_velocity, 13, fs)

        self.axes[1].plot(Time_sec[:-1], knee_velocity_filtered, color='darkred', label='Knee joint velocity (filtered)')
        self.axes[1].plot(Time_sec[:-1], ankle_velocity_filtered, color='darkblue', label='Ankle joint velocity (filtered)')
        self.axes[1].set_xlabel("Time (sec)")
        self.axes[1].set_ylabel("Angular Velocity (deg/sec)")
        self.axes[1].legend()
        self.axes[1].grid(True)

        self.axes[2].plot(Time_sec, self.processor.Pressure, color='black', label='pMMG')
        self.axes[2].set_xlabel("Time (sec)")
        self.axes[2].set_ylabel("Pressure (kPa)")
        self.axes[2].legend()
        self.axes[2].grid(True)

        self.canvas.draw()

    def save_plot(self):
        try:
            if hasattr(self, 'file_name'):
                file_name = os.path.splitext(self.file_name)[0] + '.png'
                self.fig.savefig(file_name)

        except Exception as e:
            self.handle_exception(e)

    def export_csv(self):
        try:
            if hasattr(self, 'file_name'):
                file_name = os.path.splitext(self.file_name)[0] + '.csv'

                Time_sec = self.processor.Time / 1000.0
                dt = 0.005
                knee_velocity = np.diff(self.processor.knee_angle) / dt
                ankle_velocity = np.diff(self.processor.ankle_angle) / dt

                fs = 1 / dt
                knee_velocity_filtered = lowpass_filter(knee_velocity, 13, fs)
                ankle_velocity_filtered = lowpass_filter(ankle_velocity, 13, fs)

                data_to_export = {
                    'Time_sec': Time_sec[:-1],
                    'knee_angle': self.processor.knee_angle[:-1],
                    'ankle_angle': self.processor.ankle_angle[:-1],
                    'knee_velocity_filtered': knee_velocity_filtered,
                    'ankle_velocity_filtered': ankle_velocity_filtered,
                    'Pressure': self.processor.Pressure[:-1]
                }

                with open(file_name, 'w', newline='') as csvfile:
                    writer = csv.DictWriter(csvfile, fieldnames=data_to_export.keys())
                    writer.writeheader()
                    for i in range(len(Time_sec) - 1):
                        row = {key: value[i] for key, value in data_to_export.items()}
                        writer.writerow(row)

        except Exception as e:
            self.handle_exception(e)

    def update_status_label(self, status_code):
        """Update the status label based on the status code."""
        status_mapping = {
            "101": "Standby",
            "102": "Leg zeroing started",
            "103": "Leg zeroing stopped",
            "104": "Reading started",
            "105": "Reading stopped",
            "106": "Magnetometer calibrating",
            "201": "ERROR - Low voltage",
            "202": "ERROR - pMMG malfunction",
            "203": "ERROR - IMUs malfunction",
            "SerialFail": "ERROR - USB포트 연결없음",
        }
        status_message = status_mapping.get(status_code, "ERROR - ???")
        self.status_label.setText(f"Status: {status_message}")

    def on_thread_finished(self):
        """Reset the UI after the thread finishes."""
        self.start_btn.setEnabled(True)
        self.filename_input.setEnabled(True)

    def handle_line_read(self, line):
        """Handle a line read from the serial port."""
        pass

    def display_initial_angles(self, knee_angle, ankle_angle):
        """Display the calculated initial angles."""
        self.initial_angles_label.setText(f"Initial Knee Angle: {knee_angle:.2f} degrees, Initial Ankle Angle: {ankle_angle:.2f} degrees")

    def close_app(self):
        try:
            """Close the application and stop the thread."""
            if self.thread and self.thread.isRunning():
                self.thread.stop()
                self.thread.wait()
            self.start_btn.setEnabled(True)
            self.filename_input.setEnabled(True)

        except Exception as e:
            self.handle_exception(e)


if __name__ == '__main__':
    try:
        app = QApplication(sys.argv)
        window = SerialDataSaver()
        sys.exit(app.exec_())

    except Exception as e:
        # 마지막 예외 처리
        with open("error_log.txt", "a") as f:
            traceback.print_exc(file=f)

        error_message = f"An unexpected error occurred: {str(e)}"
        QMessageBox.critical(None, "Critical Error", error_message)
        sys.exit(1)
