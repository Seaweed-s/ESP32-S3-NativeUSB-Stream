/////////////////////////////////////////////////////////////////
/*
  Wired video streaming over serial for an ESP32CAM.
  Seaweed#4353
*/
/////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "esp_camera.h"
#include <base64.h>
#define FREENOVE_S3
#include "camera_pins2.h"
#include "USB.h"
#include "ws2812.h"
//#include "Freenove_WS2812_Lib_for_ESP32.h"

#define BAUD_RATE 2000000

/* BAUD RATES:
460800 - Success
640000 - Success
860000 - Success
921600  - Untested
1200000 - Success
1500000 - Success
2000000 - Success
3000000 - Success - Doesn't seem to work with two ESP32s connected to a USB hub
*/

#define LED_BUILTIN  2

//#define WS2812_PIN  48

USBCDC SerialUSB;

//Freenove_ESP32_WS2812 rgbLed = Freenove_ESP32_WS2812(1, WS2812_PIN, 1, TYPE_GRB);

camera_fb_t* fb;

void cameraInit(){
  camera_config_t config;
  sensor_t *camera_sensor;
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
  config.xclk_freq_hz = 15000000; // 25000000 seems to be the highest stable frequency I can achieve
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_240X240;
  config.jpeg_quality = 7;
  config.fb_count = 1;

  if(psramFound()){
    //config.jpeg_quality = 10;
    //config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    // Limit the frame size when PSRAM is not available
    //config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);

  camera_sensor = esp_camera_sensor_get();
  // fixes corrupted jpegs, https://github.com/espressif/esp32-camera/issues/203
  camera_sensor->set_reg(camera_sensor, 0xff, 0xff, 0x00);         // banksel
  camera_sensor->set_reg(camera_sensor, 0xd3, 0xff, 5);            // clock
  camera_sensor->set_brightness(camera_sensor, 2);                 // -2 to 2   I see no difference between numbers..
  camera_sensor->set_contrast(camera_sensor, 2);                   // -2 to 2
  camera_sensor->set_saturation(camera_sensor, -2);                // -2 to 2
  camera_sensor->set_whitebal(camera_sensor, 1);                   // 0 = disable , 1 = enable
  camera_sensor->set_awb_gain(camera_sensor, 1);                   // 0 = disable , 1 = enable
  camera_sensor->set_wb_mode(camera_sensor, 0);                    // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  camera_sensor->set_exposure_ctrl(camera_sensor, 1);              // 0 = disable , 1 = enable
  camera_sensor->set_aec2(camera_sensor, 0);                       // 0 = disable , 1 = enable
  camera_sensor->set_gain_ctrl(camera_sensor, 0);                  // 0 = disable , 1 = enable
  camera_sensor->set_agc_gain(camera_sensor, 2);                   // 0 to 30  brightness of sorts? higher = brighter with more lag
  camera_sensor->set_gainceiling(camera_sensor, (gainceiling_t)6); // 0 to 6
  camera_sensor->set_bpc(camera_sensor, 1);                        // 0 = disable , 1 = enable
  camera_sensor->set_wpc(camera_sensor, 1);                        // 0 = disable , 1 = enable
  camera_sensor->set_raw_gma(camera_sensor, 1);                    // 0 = disable , 1 = enable (makes much lighter and noisy)
  camera_sensor->set_lenc(camera_sensor, 0);                       // 0 = disable , 1 = enable                // 0 = disable , 1 = enable
  camera_sensor->set_dcw(camera_sensor, 0);                        // 0 = disable , 1 = enable
  camera_sensor->set_colorbar(camera_sensor, 0);                   // 0 = disable , 1 = enable
  camera_sensor->set_special_effect(camera_sensor, 2);             // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)

  // Below is commented out to save processing time...
  //sensor_t * s = esp_camera_sensor_get();
  //s->set_hmirror(s, 1);   
  //s->set_vflip(s, 1);   
}


void grabImage(){
  fb = esp_camera_fb_get();
  if(!fb || fb->format != PIXFORMAT_JPEG){

  }else{

    //Serial.write((unsigned char *)fb->buf, fb->len);
    Serial.write(fb->buf, fb->len);
    USBSerial.write(fb->buf, fb->len);
    
  }
  esp_camera_fb_return(fb);
}

void setup()
{
  //ws2812Init();
  Serial.begin(BAUD_RATE);
  SerialUSB.begin(2000000);
  //while(!SerialUSB);
  pinMode(LED_BUILTIN, OUTPUT);

  ws2812Init();

  cameraInit();
}

/*
void loop() {
  digitalWrite(48, LOW);
  grabImage();
}

/*
#define LED_BUILTIN  2
// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(BAUD_RATE);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

*/

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
  ws2812SetColor(0);
  //ws2812SetColor(0);
  Serial.println("Serial Working");
  SerialUSB.println("USB Working");

}

