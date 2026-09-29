/*
 * config.h - single source of truth for every GPIO pin on the car.
 *
 * NFR-02 requires one shared config.h for pin assignments so nobody
 * accidentally reuses a pin someone else already claimed. If you need a
 * new pin for something, add it here (with a comment) and check it
 * against every other #define in this file first - don't hardcode a
 * GPIO number in your own .c file.
 *
 * These pins reflect the Week 6 pin layout (Section 2) RECONCILED
 * against docs/wiring-connection-sketch.md (that file was the actual
 * as-built wiring; a few pins in the original Section 2 table didn't
 * match it and have been corrected here - see the design review
 * assessment doc for the full list of what changed and why).
 */

#ifndef CONFIG_H
#define CONFIG_H

// ---- Encoders (Dave / Motor task) --------------------------------------
// A/B quadrature per the wiring sketch. If the team ends up building
// single-pin tick counting instead, update these and motor.c together.
#define PIN_LEFT_ENC_A   0
#define PIN_LEFT_ENC_B   1
#define PIN_RIGHT_ENC_A  2
#define PIN_RIGHT_ENC_B  3

// ---- IMU / LSM303DLHC (Sook Yee / IMU-Terrain task) --------------------
#define PIN_IMU_SDA      4
#define PIN_IMU_SCL      5

// ---- HC-SR04 ultrasonic (Felix / Obstacle Scan task) -------------------
// ECHO is 5V and MUST go through the 1k/2k divider documented in the
// wiring sketch before reaching this pin - never wire ECHO directly.
#define PIN_ULTRA_TRIG   6
#define PIN_ULTRA_ECHO   7

// ---- Motors (Dave / Motor task) ----------------------------------------
// FWD line HIGH/PWM = forward spin, BWD line HIGH/PWM = reverse spin.
// Never drive FWD and BWD high at the same time on the same motor.
#define PIN_LEFT_MOTOR_FWD   8
#define PIN_LEFT_MOTOR_BWD   9
#define PIN_RIGHT_MOTOR_FWD  10
#define PIN_RIGHT_MOTOR_BWD  11

// ---- Scanning servo (Felix / Obstacle Scan task) -----------------------
#define PIN_SERVO_PWM    12

// ---- Free / spare GPIOs ------------------------------------------------
// Safe to use for a new sensor or status LED. GP16/17/19 are held back
// on purpose - see reserved block below. GP23/24/25/29 are NOT listed
// anywhere in this file because the Pico W uses them internally for the
// wireless chip; do not repurpose them.
#define PIN_FREE_1       13
#define PIN_FREE_2       14
#define PIN_FREE_3       15
#define PIN_FREE_4       22

// ---- Reserved: IR DOUT contingency (Vania / Line Tracking task) -------
// Not wired in the baseline design (we read IR via AOUT/ADC instead).
// Only wire these up if the ADC approach fails bench testing.
#define PIN_IR_LEFT_DOUT_RESERVED    16
#define PIN_IR_CENTER_DOUT_RESERVED  17
#define PIN_IR_RIGHT_DOUT_RESERVED   19

// ---- Onboard NeoPixel status LED (shared / integration) ----------------
// This is a WS2812, NOT a plain GPIO-high LED - drive it via ws2812.pio,
// not gpio_put().
#define PIN_NEOPIXEL_STATUS  18

// ---- Buttons (shared / integration) ------------------------------------
// Wired to GND, active-low, use internal pull-ups.
#define PIN_BTN_START    20
#define PIN_BTN_2        21

// ---- IR line sensors (Vania / Line Tracking + Barcode tasks) ----------
// AOUT (analog), read via ADC - NOT the digital DO output. Grove 5's
// GP26 pin must stay UNCONNECTED (it's the same electrical node as
// IR_LEFT below); see the wiring sketch's "Hard rules" section.
#define PIN_IR_LEFT_AOUT    26  // ADC0
#define PIN_IR_CENTER_AOUT  27  // ADC1
#define PIN_IR_RIGHT_AOUT   28  // ADC2

#endif // CONFIG_H
