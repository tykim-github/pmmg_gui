# flash_esp32.py

import sys
import serial.tools.list_ports
import subprocess
import os
import tkinter as tk
from tkinter import filedialog, messagebox

def find_esp32_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if hasattr(port, 'manufacturer') and "wch.cn" in port.manufacturer.lower():
            return port.device
    return None

def flash_firmware(port, firmware_path):
    esptool_path = os.path.join(os.getcwd(), 'esptool.exe')
    
    cmd = [
        esptool_path,
        '--chip', 'esp32',
        '--port', port,
        '--baud', '921600',
        '--before', 'default_reset',
        '--after', 'hard_reset',
        'write_flash',
        '0x0', firmware_path
    ]
    
    subprocess.run(cmd)


def main():
    # GUI 초기화
    root = tk.Tk()
    root.withdraw()

    # 통합 바이너리 파일 선택
    messagebox.showinfo("안내", "업로드할 통합 펌웨어(.bin) 파일을 선택해주세요.")
    firmware_path = filedialog.askopenfilename(filetypes=[("Binary files", "*.bin")])
    if not firmware_path:
        messagebox.showerror("오류", "펌웨어 파일을 선택하지 않았습니다.")
        sys.exit(1)
    
    # ESP32 포트 찾기
    port = find_esp32_port()
    if port is None:
        messagebox.showerror("오류", "ESP32를 찾을 수 없습니다. USB 케이블을 확인해주세요.")
        sys.exit(1)
    else:
        messagebox.showinfo("안내", f"ESP32가 {port} 포트에 연결되었습니다.\n펌웨어 업로드를 시작합니다.")
        flash_firmware(port, firmware_path)
        messagebox.showinfo("완료", "펌웨어 업로드가 완료되었습니다.")


if __name__ == '__main__':
    main()
