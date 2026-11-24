#ifndef DASH_LOGIC_H
#define DASH_LOGIC_H

#define DASH_WIFI_CONNECTING	0
#define DASH_MQTT_CONNECTING	1
#define DASH_ONLINE		2
#define DASH_HEARTBEAT		3
#define DASH_RECONNECTING	4

#define DASH_HEARTBEAT_AFTER_MS	1500
#define DASH_HEARTBEAT_EVERY_MS	1000
#define DASH_RECONNECT_MS	2000

#define DASH_JSON_MAX		512

typedef struct {
	UB	state;
	INT	t_ms;
	INT	since_telem_ms;
	INT	since_hb_ms;
} DASH;

typedef struct {
	BOOL	wifi_up;
	UB	mqtt_event;
	BOOL	got_telem;
	INT	dt_ms;
} DASH_IN;

typedef struct {
	BOOL	publish_telem;
	BOOL	publish_heartbeat;
} DASH_OUT;

IMPORT void dash_init(DASH *d);
IMPORT void dash_step(DASH *d, const DASH_IN *in, DASH_OUT *out);

IMPORT INT  dash_format_telem(const MSG_TELEM *t, char *buf, INT size);

#define DASH_CMD_BYTES		1	/* MQTT command payload: 1 byte, bit 0 = START */
IMPORT UB   dash_parse_cmd(const char *data, INT len);

/* Junction mapping on csc11/dev/car/map (published retained, so the car gets
   it again after a reboot): 3 bytes, protocol_bits order, 5 bits each for
   the U-turn, left, right and straight-ahead letters (0..25 = 'A'..'Z',
   DASH_CHAR_NONE = none), then 4 bits that must be 0.  Returns TRUE and fills letters[]
   ('A'..'Z' or 0) only if the frame is exactly that and no letter is given
   to two turns. */
#define DASH_MAP_BYTES		3	/* 4 x 5 + 4 = 24 bits */
#define DASH_MAP_SPARE		15	/* the 4 spare bits */
IMPORT BOOL dash_parse_map(const char *data, INT len, UB letters[MQTTCMD_TURNS]);

/* ---- Binary frames (TEAM.md section 7) --------------------------------
   Packed with protocol_bits: each field takes the bits its maximum needs,
   fields in the order listed, starting at bit 0 of byte 0 (least
   significant bit first).  Signed fields are sent with an offset. */
#define DASH_TELEM_BYTES	11	/* 81 bits of fields, 7 spare */
#define DASH_DEBUG_BYTES	17	/* 135 bits of fields, 1 spare */
#define DASH_HB_BYTES		1	/* system state only */
#define DASH_CHAR_NONE		31	/* character code: A..Z = 0..25 */

/* Debug frame: sampled every dashboard tick (100 ms) on its own topic */
typedef struct {
	INT	dark_c;		/* line sensor darkness, centre, 0..1000 */
	INT	dark_r;		/* right */
	INT	err;		/* line position error, -500..+500 */
	INT	steer;		/* steering sent, -1000..+1000 */
	UB	mode;		/* line recovery mode, 0..3 */
	UW	bc_edges;	/* barcode edges seen (sent modulo 256) */
	UW	bc_ok;		/* barcodes decoded (modulo 256) */
	UW	bc_fail;	/* barcodes failed (modulo 256) */
	UB	bc_last;	/* last decoded character, 0 = none */
	INT	ultra_mm;	/* last ultrasonic reading, 0 = no echo */
	UB	motor_state;	/* MOT_xxx */
	UB	bc_status;	/* last decode result, 0 = OK (bc_decode_status_t) */
	UW	bc_count;	/* edges in the last capture (sent up to 63) */
	UB	scan_state;	/* SCAN_xxx (task_scan.c), 0..6 */
	UB	scan_side;	/* 0 none yet, 1 left, 2 right, 3 no way round */
	UB	bc_exit;	/* why the last barcode pass ended, BCX_xxx 0..7 */
	INT	width_mm;	/* measured obstacle width, mm (0 = none) */
	INT	half_l_mm;	/* its left half, mm (the right half is the rest) */
	UB	byp_result;	/* last bypass return: 0 none, 1 on the line, 2 no line */
} DASH_DEBUG;

IMPORT UB   dash_char_code(UB ch);
IMPORT INT  dash_pack_telem(const MSG_TELEM *t, UB seq, UB stored_char,
			    UB *buf, INT size);
IMPORT INT  dash_pack_debug(const DASH_DEBUG *g, UB seq, UB *buf, INT size);

/* IMU frame (csc11/dev/car/imu), every DASH_IMU_EVERY dashboard ticks:
   the IMU subsystem's live values for the dashboard's IMU table. */
#define DASH_IMU_BYTES		8	/* 58 bits of fields, 6 spare */
#define DASH_IMU_EVERY		2	/* every 200 ms */
typedef struct {
	UB	state;		/* IMU_xxx, 0..5 */
	UB	calibrated;	/* mounting tilt measured */
	INT	lean_um;	/* lean height now (roll x track), sent in 0.1 mm, +-102.4 mm */
	INT	last_mm;	/* last hump, 0..127 */
	INT	max_mm;		/* tallest of the run, 0..127 */
	UW	humps;		/* humps completed (modulo 256) */
	INT	rise;		/* lift detector, mg x samples, -4096..4095 */
} DASH_IMU;
IMPORT INT  dash_pack_imu(const DASH_IMU *m, UB seq, UB *buf, INT size);

/* Barcode trace frame (csc11/dev/car/barcode), sent once per capture:
   seq, decode status, edge count n, then the n-1 element widths (bar,
   space, bar, ...) as 16-bit little-endian values in 0.1 ms. */
#define DASH_BC_TRACE_EDGES	40
#define DASH_BC_TRACE_MAX	(3 + 2 * (DASH_BC_TRACE_EDGES - 1))
IMPORT INT  dash_pack_bc_trace(UB seq, UB status, const UW *edge_us, INT count,
			       UB *buf, INT size);

#endif
