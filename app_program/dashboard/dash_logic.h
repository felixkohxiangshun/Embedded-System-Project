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
IMPORT INT  dash_format_heartbeat(UW uptime_ms, UB sys_state, char *buf, INT size);

#define DASH_CMD_BYTES		1	/* MQTT command payload: 1 byte, bit 0 = START */
IMPORT UB   dash_parse_cmd(const char *data, INT len);

/* ---- Binary frames (TEAM.md section 7) --------------------------------
   Packed with protocol_bits: each field takes the bits its maximum needs,
   fields in the order listed, starting at bit 0 of byte 0 (least
   significant bit first).  Signed fields are sent with an offset. */
#define DASH_TELEM_BYTES	11	/* 81 bits of fields, 7 spare */
#define DASH_DEBUG_BYTES	24	/* 189 bits of fields, 3 spare */
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
	INT	scan_mm[5];	/* last sweep, far left .. far right, 0 = no echo */
	UB	scan_side;	/* 0 none yet, 1 left, 2 right, 3 no way round */
	UB	bc_exit;	/* why the last barcode pass ended, BCX_xxx 0..7 */
	INT	echo_rise_us;	/* ultrasonic: trigger to ECHO high, us (0 = never) */
	INT	echo_edges;	/* ultrasonic: ECHO edges in the last ping */
	UB	byp_result;	/* last bypass return: 0 none, 1 on the line, 2 no line */
} DASH_DEBUG;

IMPORT UB   dash_char_code(UB ch);
IMPORT INT  dash_pack_telem(const MSG_TELEM *t, UB seq, UB stored_char,
			    UB *buf, INT size);
IMPORT INT  dash_pack_debug(const DASH_DEBUG *g, UB seq, UB *buf, INT size);

/* Barcode trace frame (csc11/dev/car/barcode), sent once per capture:
   seq, decode status, edge count n, then the n-1 element widths (bar,
   space, bar, ...) as 16-bit little-endian values in 0.1 ms. */
#define DASH_BC_TRACE_EDGES	40
#define DASH_BC_TRACE_MAX	(3 + 2 * (DASH_BC_TRACE_EDGES - 1))
IMPORT INT  dash_pack_bc_trace(UB seq, UB status, const UW *edge_us, INT count,
			       UB *buf, INT size);

#endif
