#include "esp_camera.h"

// ====== XIAO ESP32S3 Sense 摄像头引脚定义（Seeed 官方映射）======

// PWDN / RESET 一般不用
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

// ====== 串口 & 图像参数 ======
// 串口波特率：尽量保持和 Python 端一致
#define SERIAL_BAUD   2000000          // 2 Mbps
// 分辨率：800 x 600
#define FRAME_SIZE    FRAMESIZE_SVGA   // 800x600
// JPEG 质量：数值越小画质越好、数据越大
#define JPEG_QUALITY  18               // 如果太卡可以改成 18~20

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    ; // 等待串口就绪
  }

  Serial.println();
  Serial.println("=== XIAO ESP32S3 Serial Camera ===");
  Serial.println("Checking PSRAM...");

  bool has_psram = psramFound();
  Serial.print("psramFound() = ");
  Serial.println(has_psram ? "YES" : "NO");

  Serial.println("Initializing camera...");

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  // 正确映射 D0~D7
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;

  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAME_SIZE;
  config.jpeg_quality = JPEG_QUALITY;

  // 默认先用 1 帧缓冲 + DRAM，保证稳定
  config.fb_count     = 1;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_DRAM;

  // 如果有 PSRAM，就用 PSRAM + 双缓冲，800x600 会更稳
  if (has_psram) {
    Serial.println("Using PSRAM for frame buffer");
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count    = 2;
    config.grab_mode   = CAMERA_GRAB_LATEST;
  } else {
    Serial.println("PSRAM not found, may be unstable at SVGA.");
  }

  // 初始化摄像头
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed, error 0x%x\r\n", err);
    while (true) {
      delay(2000);
    }
  }

  Serial.println("Camera ready.");
  Serial.println("Host should read 'FRAME <len>' then <len> bytes of JPEG.");
}

void loop() {
  // 获取一帧
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Failed to get frame");
    delay(100);
    return;
  }

  // 帧头：FRAME <长度>\n
  Serial.print("FRAME ");
  Serial.println(fb->len);
  Serial.write(fb->buf, fb->len);

  esp_camera_fb_return(fb);

  // 分辨率变大后，适当加大 delay，避免串口来不及发完
  delay(120);  // 约 8 fps 左右，根据实际情况可以调大/调小
}
