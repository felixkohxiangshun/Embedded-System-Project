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

IMPORT UB   dash_parse_cmd(const char *text, INT len);

/* ---- Binary frames (TEAM.md section 7) --------------------------------
   Packed with protocol_bits: each field takes the bits its maximum needs,
   fields in the order listed, starting at bit 0 of byte 0 (least
   significant bit first).  Signed fields are sent with an offset. */
#define DASH_TELEM_BYTES	11	/* 81 bits of fields, 7 spare */
#define DASH_DEBUG_BYTES	12	/* 95 bits of fields, 1 spare */
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
} DASH_DEBUG;

IMPORT UB   dash_char_code(UB ch);
IMPORT INT  dash_pack_telem(const MSG_TELEM *t, UB seq, UB stored_char,
			    UB *buf, INT size);
IMPORT INT  dash_pack_debug(const DASH_DEBUG *g, UB seq, UB *buf, INT size);

#endif
