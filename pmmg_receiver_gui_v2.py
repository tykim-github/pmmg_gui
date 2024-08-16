import sys
import numpy as np
import serial
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLineEdit, QLabel, QHBoxLayout, QFileDialog, QCheckBox, QFormLayout
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QPixmap
from screeninfo import get_monitors

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

import serial.tools.list_ports
import numpy.linalg as LA
from scipy.signal import butter, filtfilt

# to make this exe
# pyinstaller --onefile --noconsole --icon=legmus.ico pmmg_receiver_gui_v2.py

class DataProcessor:
    def __init__(self, initial_knee_angle=None, initial_ankle_angle=None):
        self.initial_knee_angle = initial_knee_angle
        self.initial_ankle_angle = initial_ankle_angle
        self.q_ti = None
        self.q_si = None
        self.q_fi = None
        self.data_buffer = []

    def initialize_from_header(self, header_data):
        """헤더 데이터에서 초기 각도를 설정합니다."""
        self.initial_knee_angle = header_data.get('initial_knee_angle')
        self.initial_ankle_angle = header_data.get('initial_ankle_angle')
        self.q_ti = header_data.get('q_ti')
        self.q_si = header_data.get('q_si')
        self.q_fi = header_data.get('q_fi')

    def process_data(self, data):
        """데이터를 처리하여 결과를 반환합니다."""
        Time = data[:, 1]
        Quaternions = data[:, 2:14]
        Pressure = data[:, 14]
        q_thigh = data[:, 2:6]
        q_shank = data[:, 6:10]
        q_foot = data[:, 10:14]

        q_k = np.array([Quaternion.mult(Quaternion.mult(q_shank[i], Quaternion.conj(self.q_si)),
                        Quaternion.mult(self.q_ti, Quaternion.conj(q_thigh[i])))
                        for i in range(len(data))])

        q_a = np.array([Quaternion.mult(Quaternion.mult(q_foot[i], Quaternion.conj(self.q_fi)),
                        Quaternion.mult(self.q_si, Quaternion.conj(q_shank[i])))
                        for i in range(len(data))])

        knee_angle = np.degrees([Quaternion.angle(q) for q in q_k]) + self.initial_knee_angle
        ankle_angle = np.degrees([Quaternion.angle(q) for q in q_a]) + self.initial_ankle_angle

        return Time, Quaternions, Pressure, knee_angle, ankle_angle

    def calculate_initial_state(self):
        """초기 상태를 계산하여 쿼터니언을 설정합니다."""
        if not self.data_buffer:
            return
        data = np.array([list(map(float, x.split(','))) for x in self.data_buffer])
        Quaternions = data[:, 2:14]

        Quaternions /= LA.norm(Quaternions, axis=1)[:, np.newaxis]
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
        for port in serial.tools.list_ports.comports():
            if "wch.cn" in port.manufacturer:
                self.com_port = port.device
        
        try:
            ser = serial.Serial(self.com_port, 115200, timeout=1)
        except serial.SerialException as e:
            self.state_changed.emit("ERROR - Serial Port Failure")
            return

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
                        Time, Quaternions, Pressure, knee_angle, ankle_angle = self.processor.process_data(data)
                        self.data_processed.emit(Time, Quaternions, Pressure, knee_angle, ankle_angle)
                        self.collecting = False
                        self.file_handler.close_file()
                    elif self.collecting:
                        self.processor.data_buffer.append(line)
                        if state not in {"102", "103", "104", "105"}:
                            self.file_handler.write_line(line)
        finally:
            self.file_handler.close_file()
            ser.close()

    def stop(self):
        """Stop the serial reading thread."""
        self.running = False


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
        pixmap = QPixmap("joint_angle_definition.png")  # 이미지 파일 경로
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

        self.patient_leg_thickness_input = QLineEdit(self)
        self.patient_leg_thickness_input.setStyleSheet(f"font-size: {base_font_size_px}px;")
        patient_info_layout.addRow("Leg Thickness (cm):", self.patient_leg_thickness_input)

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

        control_layout.addLayout(buttons_layout)
        layout.addLayout(control_layout, 1)  # Allocate 30% to 40% of width for controls

        self.setLayout(layout)
        self.setWindowTitle('Spasticity Measurement Software')
        self.show()

    def start_reading(self):
        """Start the serial reading thread."""
        filename = self.filename_input.text()
        leg_thickness = self.patient_leg_thickness_input.text()

        # GUI에서 입력된 초기 각도 값을 가져옵니다.
        initial_knee_angle = float(self.initial_knee_angle_input.text())
        initial_ankle_angle = float(self.initial_ankle_angle_input.text())

        recorder_name = self.recorder_name_input.text()

        if not filename:
            return

        self.start_btn.setEnabled(False)
        self.filename_input.setEnabled(False)

        # header_info에 초기 각도 값을 저장합니다.
        header_info = {
            "Leg Thickness (cm)": leg_thickness,
            "Initial Knee Angle (deg)": initial_knee_angle,
            "Initial Ankle Angle (deg)": initial_ankle_angle,
            "Recorder Name": recorder_name
        }

        self.processor = DataProcessor(initial_knee_angle, initial_ankle_angle)

        # 쓰레드 시작
        self.thread = SerialReader(filename, header_info, self.processor, self)
        self.thread.data_processed.connect(self.plot_data)
        self.thread.line_read.connect(self.handle_line_read)
        self.thread.state_changed.connect(self.update_status_label)
        self.thread.initial_angles_calculated.connect(self.display_initial_angles)
        self.thread.start()

    def load_data(self):
        """Load data from a file."""
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self, "Load Data File", "", "Text Files (*.txt);;All Files (*)", options=options)
        if file_name:
            header_data = {}
            
            # 첫 번째로 파일을 열어 헤더 정보를 읽습니다.
            with open(file_name, 'r') as file:
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
            data = np.loadtxt(file_name, delimiter=',', skiprows=len(header_data) + 1)  # 헤더 줄을 건너뛰도록 설정

            # 데이터를 처리하고 플롯을 그립니다.
            Time, Quaternions, Pressure, knee_angle, ankle_angle = self.processor.process_data(data)
            self.plot_data(Time, Quaternions, Pressure, knee_angle, ankle_angle)

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

        def lowpass_filter(data, cutoff_freq, fs, order=5):
            nyquist_freq = 0.5 * fs
            normal_cutoff = cutoff_freq / nyquist_freq
            b, a = butter(order, normal_cutoff, btype='low', analog=False)
            return filtfilt(b, a, data)

        fs = 1 / dt
        knee_velocity_filtered = lowpass_filter(knee_velocity, 13, fs)
        ankle_velocity_filtered = lowpass_filter(ankle_velocity, 13, fs)

        self.axes[1].plot(Time_sec[:-1], knee_velocity_filtered, color='darkred', label='Knee joint velocity (filtered)')
        self.axes[1].plot(Time_sec[:-1], ankle_velocity_filtered, color='darkblue', label='Ankle joint velocity (filtered)')
        self.axes[1].set_xlabel("Time (sec)")
        self.axes[1].set_ylabel("Angular Velocity (deg/sec)")
        self.axes[1].legend()
        self.axes[1].grid(True)

        self.axes[2].plot(Time_sec, Pressure, color='black', label='pMMG')
        self.axes[2].set_xlabel("Time (sec)")
        self.axes[2].set_ylabel("Pressure (kPa)")
        self.axes[2].legend()
        self.axes[2].grid(True)

        self.canvas.draw()

    def save_plot(self):
        """Save the current plot to a file."""
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getSaveFileName(self, "Save Plot", "", "PNG Files (*.png);;All Files (*)", options=options)
        if file_name:
            self.fig.savefig(file_name)

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
            "203": "ERROR - IMUs malfunction"
        }
        status_message = status_mapping.get(status_code, "ERROR - ???")
        self.status_label.setText(f"Status: {status_message}")

    def handle_line_read(self, line):
        """Handle a line read from the serial port."""
        pass

    def display_initial_angles(self, knee_angle, ankle_angle):
        """Display the calculated initial angles."""
        self.initial_angles_label.setText(f"Initial Knee Angle: {knee_angle:.2f} degrees, Initial Ankle Angle: {ankle_angle:.2f} degrees")

    def close_app(self):
        """Close the application and stop the thread."""
        if self.thread and self.thread.isRunning():
            self.thread.stop()
            self.thread.wait()
        self.start_btn.setEnabled(True)
        self.filename_input.setEnabled(True)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SerialDataSaver()
    sys.exit(app.exec_())
