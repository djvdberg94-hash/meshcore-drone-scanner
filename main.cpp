#include <Arduino.h>
#include <Mesh.h>
#include "DroneScanner.h"
#include <SPIFFS.h>
#include <helpers/ArduinoSerialInterface.h>

ArduinoSerialInterface serial_interface;
StdRNG fast_rng;
SimpleMeshTables tables;
DroneScanner the_mesh(radio_driver, fast_rng, rtc_clock, tables);

// Display state - updated by DroneScanner
char disp_line1[22] = "DroneScanner";
char disp_line2[22] = "Starting...";
char disp_line3[22] = "";

void set_display(const char* l1, const char* l2 = "", const char* l3 = "") {
  strncpy(disp_line1, l1, sizeof(disp_line1) - 1);
  strncpy(disp_line2, l2, sizeof(disp_line2) - 1);
  strncpy(disp_line3, l3, sizeof(disp_line3) - 1);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  board.begin();

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.setCursor(0, 0); display.print("DroneScanner");
    display.setCursor(0, 12); display.print("Starting...");
    display.endFrame();
  }
#endif

  if (!radio_init()) {
    Serial.println("Radio init FAILED!");
#ifdef DISPLAY_CLASS
    display.startFrame();
    display.setCursor(0, 0); display.print("Radio FAIL!");
    display.endFrame();
#endif
    while (1);
  }

  fast_rng.begin(radio_driver.getRngSeed());
  SPIFFS.begin(true);
  the_mesh.begin(&SPIFFS);

  set_display("DroneScanner", "Ready for", "takeoff!");

  board.onBootComplete();
}

static unsigned long last_display = 0;
static uint8_t tick = 0;

void loop() {
  the_mesh.loop();
  rtc_clock.tick();

#ifdef DISPLAY_CLASS
  if (millis() - last_display > 500) {
    last_display = millis();
    const char* spinner = "|/-\\";
    char hb[2] = {spinner[tick++ % 4], 0};

    display.startFrame();
    // Line 1: title + spinner
    char title[22];
    snprintf(title, sizeof(title), "Drone %s", hb);
    display.setCursor(0, 0); display.print(title);
    // Line 2
    display.setCursor(0, 12); display.print(disp_line2);
    // Line 3
    display.setCursor(0, 24); display.print(disp_line3);
    display.endFrame();
  }
#endif
}
