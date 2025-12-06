import time
import serial
import cv2
import numpy as np
from serial.tools import list_ports

# 如果你确定自己知道端口，可以在这里填，比如 "COM3"
# 如果填了但这个口不存在，会自动改用自动搜索结果
PREFERRED_PORT = "COM3"
BAUD = 2000000  # 要和 Arduino 里的 SERIAL_BAUD 一致
NO_FRAME_TIMEOUT = 3.0  # 超过这么多秒没有拿到有效帧就重连串口


def auto_find_port(preferred: str | None = None) -> str | None:
    """自动寻找一个适合 XIAO 的串口。"""
    ports = list(list_ports.comports())
    if not ports:
        print("没有找到任何串口设备。")
        return None

    # 如果指定了 preferred，并且真的存在，就用它
    if preferred:
        for p in ports:
            if p.device == preferred:
                print(f"使用指定串口: {preferred}")
                return preferred
        print(f"警告：指定的串口 {preferred} 没找到，尝试自动搜索...")

    # 优先：描述里带 XIAO 的
    for p in ports:
        desc = (p.description or "").upper()
        if "XIAO" in desc:
            print(f"自动检测到 XIAO 串口: {p.device} ({p.description})")
            return p.device

    # 其次：常见的 USB 串口名字
    for p in ports:
        desc = (p.description or "").upper()
        if "USB-SERIAL" in desc or "USB SERIAL" in desc:
            print(f"自动选择 USB 串口: {p.device} ({p.description})")
            return p.device

    # 实在找不到，就用第一个
    p = ports[0]
    print(f"没有找到明显的 XIAO 串口，使用第一个: {p.device} ({p.description})")
    return p.device


def read_frame(ser: serial.Serial):
    """
    从串口读取一帧：
    1. 读一行形如 'FRAME <len>\\n'
    2. 再读 <len> 字节的 JPEG 数据
    """
    # 读 header 行
    line = ser.readline().decode(errors="ignore").strip()
    if not line.startswith("FRAME "):
        return None

    # 解析长度
    try:
        length = int(line.split(" ")[1])
    except (IndexError, ValueError):
        return None

    # 读取 JPEG 数据
    data = b""
    while len(data) < length:
        chunk = ser.read(length - len(data))
        if not chunk:
            break
        data += chunk

    if len(data) != length:
        return None

    # 用 OpenCV 解码 JPEG → 图像
    img_array = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    return img


def open_serial():
    """打开串口封装，带自动找端口。"""
    port = auto_find_port(PREFERRED_PORT)
    if port is None:
        return None

    print(f"Opening {port} at {BAUD} baud...")
    ser = serial.Serial(port, BAUD, timeout=1)
    # 清空一下缓冲区，避免一开始读到半截数据
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def main():
    ser = open_serial()
    if ser is None:
        print("没有可用串口，退出。")
        return

    last_ok_time = time.time()

    try:
        while True:
            frame = read_frame(ser)
            now = time.time()

            if frame is not None:
                # 成功拿到一帧
                last_ok_time = now
                cv2.imshow("XIAO ESP32S3 Camera", frame)
                # 按 q 退出
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            else:
                # 没拿到帧，检查是否超时
                if now - last_ok_time > NO_FRAME_TIMEOUT:
                    print("长时间没有收到有效帧，重置串口连接...")
                    try:
                        ser.close()
                    except Exception:
                        pass
                    time.sleep(1.0)
                    ser = open_serial()
                    if ser is None:
                        print("重连失败，没有可用串口，退出。")
                        break
                    last_ok_time = time.time()

    except KeyboardInterrupt:
        print("用户中断，退出。")

    finally:
        try:
            if ser and ser.is_open:
                ser.close()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
