/*
 * obstacle_scan.h - Ultrasonic Scanning, Servo, Obstacle Avoidance
 * (owner: Felix). Covers FR-07, FR-08, FR-09, FR-10, NFR-07, NFR-08, NFR-09.
 */
#ifndef OBSTACLE_SCAN_H
#define OBSTACLE_SCAN_H

void obstacle_scan_init(void);

// TODO(Felix): sweep the servo (PIN_SERVO_PWM) across the five angles
// (-60, -30, 0, +30, +60), read PIN_ULTRA_TRIG/PIN_ULTRA_ECHO at each
// angle, and report an obstacle alert (width + clearance) to
// mission_state.c when something is within 30cm.
void obstacle_scan_update(void);

#endif // OBSTACLE_SCAN_H
