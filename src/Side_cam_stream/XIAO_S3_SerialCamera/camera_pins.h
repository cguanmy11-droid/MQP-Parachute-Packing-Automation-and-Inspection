// camera_pins.h —— Seeed Studio XIAO ESP32S3 Sense 摄像头引脚定义

#ifndef CAMERA_PINS_H
#define CAMERA_PINS_H

// XIAO ESP32S3 Sense 上摄像头的引脚分配（OV2640 / OV5640 通用）

// PWDN / RESET 通常不用，设为 -1
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1

// 时钟 & SCCB(I2C)
#define XCLK_GPIO_NUM     10    // XMCLK
#define SIOD_GPIO_NUM     40    // SDA
#define SIOC_GPIO_NUM     39    // SCL

// 数据线（DVP）
#define Y9_GPIO_NUM       48    // D7
#define Y8_GPIO_NUM       11    // D6
#define Y7_GPIO_NUM       12    // D5
#define Y6_GPIO_NUM       14    // D4
#define Y5_GPIO_NUM       16    // D3
#define Y4_GPIO_NUM       18    // D2
#define Y3_GPIO_NUM       17    // D1
#define Y2_GPIO_NUM       15    // D0

// 同步信号
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

#endif // CAMERA_PINS_H
