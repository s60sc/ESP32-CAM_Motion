// Motion detection test - looks for diffs between camera images.

#define CAMERA_MODEL_AI_THINKER

#include "esp_camera.h"
#include "camera_pins.h"

#define THIS_FRAME FRAMESIZE_SVGA // JPEG frames size to be retrieved
#define LAMP_PIN 4

bool debug = true;
uint8_t fsizePtr = THIS_FRAME; // framesize selection
uint8_t lightLevel; // Current ambient light level 
uint8_t nightSwitch = 10; // white level % for night/day switching
uint8_t motionVal = 10; // motion sensitivity setting
bool haveMotion = false;

struct frameStruct {
  const char* frameSizeStr;
  const uint16_t frameWidth;
  const uint16_t frameHeight;
  const uint16_t defaultFPS;
  const uint8_t scaleFactor;
  const uint8_t sampleRate;
};
// sample factors, 5th arg is scaling factor (1..N), 6th arg is sample factor (0..3)
// indexed by frame size - needs to be consistent with sensor.h enum
extern const frameStruct frameData[] = {
  {"QQVGA", 160, 120, 25, 2, 1},
  {"n/a", 0, 0, 0, 0, 1}, 
  {"n/a", 0, 0, 0, 0, 1}, 
  {"HQVGA", 240, 176, 25, 3, 1}, 
  {"QVGA", 320, 240, 25, 3, 1}, 
  {"CIF", 400, 296, 25, 3, 1},
  {"VGA", 640, 480, 15, 3, 2}, 
  {"SVGA", 800, 600, 10, 3, 2}, 
  {"XGA", 1024, 768, 5, 3, 3}, 
  {"SXGA", 1280, 1024, 3, 3, 4}, 
  {"UXGA", 1600, 1200, 2, 6, 5}  
};

bool checkMotion(camera_fb_t* fb, bool motionStatus);
bool isNight(uint8_t nightSwitch);

void setup() {
  Serial.begin(115200);
  setup_camera();
}

void loop() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) {
    haveMotion = checkMotion(fb, haveMotion);
    controlLamp(isNight(nightSwitch)); // switch on lamp if dark
    esp_camera_fb_return(fb); 
  } else Serial.println("Failed to get frame");
  delay(100);
}

void controlLamp(bool lampVal) {
  pinMode(LAMP_PIN, OUTPUT);
  digitalWrite(LAMP_PIN, lampVal);
  Serial.printf("Turn lamp %s\n", lampVal ? "On" : "Off");
}

void setup_camera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_JPEG;  
    config.frame_size = THIS_FRAME;
    config.jpeg_quality = 10;
    config.fb_count = 8;

    // camera init
    esp_err_t err = ESP_FAIL;
    uint8_t retries = 2;
    while (retries && err != ESP_OK) {
      err = esp_camera_init(&config);
      if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        digitalWrite(PWDN_GPIO_NUM, 1);
        delay(100);
        digitalWrite(PWDN_GPIO_NUM, 0); // power cycle the camera (OV2640)
        retries--;
      }
    } 
    if (err != ESP_OK) ESP.restart();
}
