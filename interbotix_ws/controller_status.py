#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import os
import time

class ControllerStatus(Node):
    def __init__(self):
        super().__init__('controller_status')
        self.subscription = self.create_subscription(
            Joy,
            '/wx200/commands/joy_raw',
            self.joy_callback,
            10)
        
        self.latest_joy = None
        
        # 创建一个定时器来定期更新显示
        self.timer = self.create_timer(0.1, self.update_display)
        
        print("控制器状态监控程序已启动!")

    def joy_callback(self, msg):
        self.latest_joy = msg

    def update_display(self):
        if self.latest_joy is None:
            return
        
        # 清屏
        os.system('clear')
        
        print("=" * 60)
        print("Xbox One 控制器实时状态监控")
        print("=" * 60)
        print(f"时间: {time.strftime('%H:%M:%S')}")
        print()
        
        # 显示按钮状态
        print("🔘 按钮状态:")
        button_names = [
            "A", "B", "X", "Y", "LB", "RB", "Back/View", "Start/Menu",
            "Xbox/Guide", "LS按下", "RS按下", "未知11", "未知12", "未知13", "未知14", "未知15"
        ]
        
        for i, button in enumerate(self.latest_joy.buttons):
            name = button_names[i] if i < len(button_names) else f"按钮{i}"
            status = "🟢 按下" if button else "⚫ 未按"
            print(f"  {i:2d}: {name:12s} - {status}")
        
        print()
        
        # 显示轴状态
        print("🎮 轴状态:")
        axis_names = [
            "左摇杆X", "左摇杆Y", "左扳机", "右摇杆X", "右摇杆Y", "右扳机",
            "方向键X", "方向键Y", "轴8", "轴9", "轴10", "轴11", "轴12", "轴13", "轴14", "轴15"
        ]
        
        for i, axis in enumerate(self.latest_joy.axes):
            name = axis_names[i] if i < len(axis_names) else f"轴{i}"
            # 使用颜色标识非零值
            if abs(axis) > 0.1:
                status = f"🔴 {axis:7.3f}"
            else:
                status = f"⚫ {axis:7.3f}"
            print(f"  {i:2d}: {name:12s} - {status}")
        
        print()
        print("按 Ctrl+C 退出程序")
        print("=" * 60)

def main(args=None):
    rclpy.init(args=args)
    
    controller_status = ControllerStatus()
    
    try:
        rclpy.spin(controller_status)
    except KeyboardInterrupt:
        print("\n程序已停止")
    
    controller_status.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
