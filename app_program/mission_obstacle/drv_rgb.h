/**
 * @file drv_rgb.h
 *      Status LEDs: the Robo Pico's NeoPixel RGB LEDs on GP18 (PIN_LED_STATUS),
 *      driven by a PIO1 state machine.  Owned by mission_task.
 */
#ifndef DRV_RGB_H
#define DRV_RGB_H

#define RGB_LED_COUNT           2       /* NeoPixels chained on GP18 */

/* Colours, 0xRRGGBB, kept dim: full brightness is blinding.  About a third
   of the first values (2026-11-23: still too bright on the bench). */
#define RGB_OFF                 0x000000u
#define RGB_BLUE                0x000008u       /* IDLE */
#define RGB_GREEN               0x000800u       /* running */
#define RGB_YELLOW              0x060500u       /* obstacle: sweeping or going round */
#define RGB_PURPLE              0x050008u       /* barcode: reading or turning */
#define RGB_CYAN                0x000606u       /* on a hump */
#define RGB_RED                 0x0a0000u       /* FAULT_RECOVERY */
#define RGB_WHITE               0x040404u       /* INIT: waiting for WiFi */

/* Claim a PIO1 state machine and hand GP18 to it.  E_LIMIT if none is
   free (the LEDs then stay dark; nothing else is affected). */
IMPORT ER   rgb_init(void);
/* Show one colour on every LED (sent only when it changes) */
IMPORT void rgb_set(UW colour);

#endif /* DRV_RGB_H */

/*** end of file ***/
