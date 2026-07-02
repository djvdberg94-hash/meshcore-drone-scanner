#include <Arduino.h>
#include <Mesh.h>
#include "DroneScanner.h"

#if defined(NRF52_PLATFORM)
  #include <InternalFileSystem.h>
  #if defined(QSPIFLASH)
    #include <CustomLFS_QSPIFlash.h>
  #endif
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#include <helpers/ArduinoSerialInterface.h>
ArduinoSerialInterface serial_interface;

StdRNG fast_rng;
SimpleMeshTables tables;
DroneScanner the_mesh(radio_driver, fast_rng, rtc_clock, tables);

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
    while (1);
  }

  fast_rng.begin(radio_driver.getRngSeed());

#if defined(NRF52_PLATFORM)
  InternalFS.begin();
  #if defined(QSPIFLASH)
    QSPIFlash.begin();
    the_mesh.begin(&QSPIFlash);
  #else
    the_mesh.begin(&InternalFS);
  #endif
#elif defined(ESP32)
  SPIFFS.begin(true);
  the_mesh.begin(&SPIFFS);
#endif

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
    char title[22];
    snprintf(title, sizeof(title), "Drone %s", hb);
    display.setCursor(0, 0); display.print(title);
    display.setCursor(0, 12); display.print(disp_line2);
    display.setCursor(0, 24); display.print(disp_line3);
    display.endFrame();
  }
#endif
}
