/**
 * @file main.cpp
 * @brief Emulates an Xbox 360 controller using an ESP32, BLE, and USB HID.
 *
 * This program connects to an Xbox Series X controller via BLE using the XboxSeriesXControllerESP32_asukiaaa library,
 * reads its input state, and sends corresponding Xbox 360 HID reports over USB to a nintendo switch.
 * It also uses an onboard RGB LED (via Adafruit_NeoPixel) to indicate connection status:
 *   - Green: Controller connected and USB HID ready
 *   - Red:   Controller not connected or USB HID not ready
 *
 * Key Components:
 * - BLE initialization and Xbox controller connection
 * - Mapping Xbox Series X controller input to Xbox 360 HID report format
 * - USB HID report sending using TinyUSB
 * - RGB LED status indication
 *
 * Libraries Used:
 * - Arduino core for ESP32
 * - XboxSeriesXControllerESP32_asukiaaa (for BLE Xbox controller support)
 * - NimBLEDevice (for BLE stack)
 * - Adafruit_NeoPixel (for RGB LED control)
 * - TinyUSB (for USB HID functionality)
 *
 * @author
 * @date
 */
#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <Adafruit_NeoPixel.h>
#include <NimBLEDevice.h>
#include "usb_descriptors.h"
#include <USB.h>

#define LED_PIN 48 
#define LED_COUNT 1 

Adafruit_NeoPixel rgb(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define REPORT_ID_XBOX360 0x01
#define REPORT_SIZE_XBOX360 20

XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

void sendXbox360Report(const XboxControllerNotificationParser &notif)
{

  uint8_t report[REPORT_SIZE_XBOX360] = {0};

  uint16_t buttons = 0;
  if (notif.btnA)
    buttons |= (1 << 0);
  if (notif.btnB)
    buttons |= (1 << 1);
  if (notif.btnX)
    buttons |= (1 << 3);
  if (notif.btnY)
    buttons |= (1 << 2);
  if (notif.btnLB)
    buttons |= (1 << 4);
  if (notif.btnRB)
    buttons |= (1 << 5);
  // if (notif.btnLS)     buttons |= (1 << 6);
  // if (notif.btnRS)     buttons |= (1 << 7);
  if (notif.trigLT > 0)
    buttons |= (1 << 6); // LS
  if (notif.trigRT > 0)
    buttons |= (1 << 7); // RS

  if (notif.btnStart)
    buttons |= (1 << 9);
  if (notif.btnSelect)
    buttons |= (1 << 8);
  if (notif.btnShare)
    buttons |= (1 << 10);
  if (notif.btnXbox)
    buttons |= (1 << 11);

  report[0] = buttons & 0xFF;
  report[1] = (buttons >> 8) & 0xFF;

  uint8_t hat = 8;
  if (notif.btnDirUp && notif.btnDirRight)
    hat = 1;
  else if (notif.btnDirRight && notif.btnDirDown)
    hat = 3;
  else if (notif.btnDirDown && notif.btnDirLeft)
    hat = 5;
  else if (notif.btnDirLeft && notif.btnDirUp)
    hat = 7;
  else if (notif.btnDirUp)
    hat = 0;
  else if (notif.btnDirRight)
    hat = 2;
  else if (notif.btnDirDown)
    hat = 4;
  else if (notif.btnDirLeft)
    hat = 6;
  report[2] = hat;

  report[3] = map(notif.trigLT, 0, 0x3FF, 0, 255);
  report[4] = map(notif.trigRT, 0, 0x3FF, 0, 255);

  int16_t lx = map(notif.joyLHori, 0, 0xFFFF, -32767, 32767);
  int16_t ly = map(notif.joyLVert, 0, 0xFFFF, -32767, 32767);
  int16_t rx = map(notif.joyRHori, 0, 0xFFFF, -32767, 32767);
  int16_t ry = map(notif.joyRVert, 0, 0xFFFF, -32767, 32767);

  report[5] = (uint8_t)(lx & 0xFF);
  report[6] = (uint8_t)((lx >> 8) & 0xFF);
  report[7] = (uint8_t)(ly & 0xFF);
  report[8] = (uint8_t)((ly >> 8) & 0xFF);
  report[9] = (uint8_t)(rx & 0xFF);
  report[10] = (uint8_t)((rx >> 8) & 0xFF);
  report[11] = (uint8_t)(ry & 0xFF);
  report[12] = (uint8_t)((ry >> 8) & 0xFF);

  tud_hid_report(REPORT_ID_XBOX360, &report[0], REPORT_SIZE_XBOX360);
}

// Main entry point for the program
void setup()
{
  USB.begin(); // Initialize USB HID
  NimBLEDevice::init("Xbox BLE"); // Initialize BLE stack with device name
  xboxController.begin(); // Start Xbox controller BLE connection

  rgb.begin();            // Initialize RGB LED
  rgb.setBrightness(100); // Set LED brightness
  rgb.show();             // Update LED to show initial state
}

void loop()
{
  xboxController.onLoop(); // Handle BLE events and update controller state

  // If controller is connected and USB HID is ready
  if (xboxController.isConnected() && tud_hid_ready())
  {
    rgb.setPixelColor(0, rgb.Color(0, 255, 0)); // Set LED to green (connected)
    rgb.show();
    sendXbox360Report(xboxController.xboxNotif); // Send HID report based on controller input
  }
  else
  {
    rgb.setPixelColor(0, rgb.Color(255, 0, 0)); // Set LED to red (disconnected)
    rgb.show();
  }
  delay(10); // Small delay to avoid busy loop
}