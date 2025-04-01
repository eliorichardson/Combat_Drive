#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <ESP32Servo.h> // ESP32Servo library installed by Library Manager
#include "ESC.h" // RC_ESP library installed by Library Manager
#include <FastLED.h> // FastLED library installed by Library Manager

#define DriveL D5
#define DriveR D6
#define Weapon D1

#define NUM_LEDS 4
#define DATA_PIN D10
bool controllerConnected = false;

CRGB leds[NUM_LEDS];

XboxSeriesXControllerESP32_asukiaaa::Core xboxController("EC:83:50:05:71:92");

ESC Lmotor (DriveL, 1000, 2000, 500);
ESC Rmotor (DriveR, 1000, 2000, 500);

void setupPWM() {
  //FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS); 

  Serial.println("Initializing PWM...");

  const int pwmFrequency = 5000;  // 5 kHz
  const int pwmResolution = 8;    // 8-bit resolution

  ledcSetup(2, pwmFrequency, pwmResolution);  ledcAttachPin(Weapon, 2);

  Serial.println("PWM Initialized Successfully");
}

void LED_Task(void* pvParameters) {
  while (true) {
    if (controllerConnected) {
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::OrangeRed;
      }
      FastLED.show();
      vTaskDelay(100 / portTICK_PERIOD_MS);

      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Black;
      }
      FastLED.show();
      vTaskDelay(100 / portTICK_PERIOD_MS);

    } else {

      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Red;
      }
      FastLED.show();
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Black;
      }
      FastLED.show();
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setupPWM();
  delay(100);
  pinMode(DriveL, OUTPUT);
  pinMode(DriveR, OUTPUT);

  Lmotor.arm();
  Rmotor.arm();

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);

  xboxController.begin();
  Serial.println("Controller initialized");

  xTaskCreatePinnedToCore(LED_Task, "LED_Task", 4096, NULL, 1, NULL, 0); // Create LED task on core 0
}

void loop() {
  xboxController.onLoop();
  vTaskDelay(10 / portTICK_PERIOD_MS);

  if (xboxController.isConnected()) {
    controllerConnected = true;
    int rx_axis = (((int)xboxController.xboxNotif.joyRHori) - 32768) * 0.3; // Left stick horizontal
    int ly_axis = (-((int)xboxController.xboxNotif.joyLVert ) + 32768); // Left stick vertical
    float rt_axis = ((float)xboxController.xboxNotif.trigRT / 8); // Left trigger

    rx_axis = abs(rx_axis) < 2000 ? 0 : rx_axis;
    ly_axis = abs(ly_axis) < 2000 ? 0 : ly_axis;

    int Lmotor_trimmed = constrain(constrain(ly_axis + rx_axis, -32768, 32768) > 0 ? 
                         constrain(ly_axis + rx_axis, -32768, 32768) * 1 :                   //Left forward
                         constrain(ly_axis + rx_axis, -32768, 32768) * 1, -32768, 32768);  //Left backward

    int Rmotor_trimmed = constrain(constrain(ly_axis - rx_axis, -32768, 32768) > 0 ? 
                         constrain(ly_axis - rx_axis, -32768, 32768) * 1 :                 //Right forward
                         constrain(ly_axis - rx_axis, -32768, 32768) * 1, -32768, 32768);    //Right backward

    int LPWM = map(Lmotor_trimmed, -32768, 32768, 1900, 1100);
    int RPWM = map(Rmotor_trimmed, -32768, 32768, 1900, 1100);

    Lmotor.speed(LPWM);
    Rmotor.speed(RPWM);

    Serial.print("Left Stick: ");
    Serial.print(ly_axis);
    Serial.print(" Right Stick: ");
    Serial.print(rx_axis);
    Serial.print("Left PWM: ");
    Serial.print(LPWM);
    Serial.print(" Right PWM: ");
    Serial.println(RPWM);

    ledcWrite(2, constrain(rt_axis + 127, 0, 255));

  } else {
    controllerConnected = false;
    Lmotor.speed(1500);
    Rmotor.speed(1500);
    ledcWrite(2, 0);
  }
}