/**
 * @file task_dashboard.c
 * @brief dashboard_task: publishes the frames and receives the commands.
 */

#include "dashboard.h"
#include "../line_barcode/line_barcode.h"
#include "../mission_obstacle/mission_obstacle.h"
#include "../imu_terrain/imu_terrain.h"
#include "../line_barcode/ir_driver.h"

/* 1 publishes a debug frame (line sensors, steering, barcode counters,
   ultrasonic) every DASH_TICK_MS while online.  It is extra to the graded
   telemetry and goes on its own topic. */
/* 1: line-sensor calibration.  darkC / darkR on the debug topic carry the
   RAW filtered counts of the centre and right sensors (10-bit, 0..1023,
   LB_ADC_BITS) instead of the calibrated darkness, so the white and
   black values can be read off the dashboard.  Restore 0. */
#ifndef DASH_RAW_IR
#define DASH_RAW_IR             0
#endif

#ifndef DASH_DEBUG_FRAMES
#define DASH_DEBUG_FRAMES       1
#endif
#if TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif

EXPORT volatile UB s4_state;
EXPORT volatile UW s4_frames;
EXPORT volatile UW s4_console_lines;

#define DASH_TICK_MS    100

#if APP_TEST && TM_WIFI_CYW43 && TM_NET_LWIP

LOCAL void report_wifi_test_status(void)
{
    T_CYW43_UTK_STATUS st;
    UW ip;
    cyw43_utk_get_status(&st);
    if (st.link_status == 3 && st.ip_addr != 0) {
        ip = st.ip_addr;
        tm_printf((UB*)"[wifi-test] UP ip=%d.%d.%d.%d\n",
            (INT)(ip & 0xff), (INT)((ip >> 8) & 0xff),
            (INT)((ip >> 16) & 0xff), (INT)((ip >> 24) & 0xff));
    } else {
        tm_printf((UB*)"[wifi-test] NOT_UP link=%d ready=%d\n",
            (INT)st.link_status, (INT)st.ready);
    }
}
#endif

LOCAL BOOL wifi_is_up(void)
{
#if TM_WIFI_CYW43
    T_CYW43_UTK_STATUS st;

    cyw43_utk_get_status(&st);
    return (st.ready && st.link_status > 0 && st.ip_addr != 0) ? TRUE : FALSE;
#else
    return FALSE;
#endif
}

LOCAL void send_out(UB kind, const UB *data, INT len)
{
    static MQTT_MSG m;
    INT i;

    m.kind = kind;
    m.len = (UH)len;
    for (i = 0; i < len; i++) {
        m.data[i] = (char)data[i];
    }
    MBF_SEND(mbf_mqtt_out, m);
}

#if DASH_DEBUG_FRAMES
/* Sample the values most useful for debugging on the car.  These are
   another subsystem's counters, read without a lock: a debug view only. */
LOCAL void sample_debug(DASH_DEBUG *g)
{
#if DASH_RAW_IR
    g->dark_c = (INT)ir_driver_get_dark(IR_CHANNEL_CENTRE);     /* already 10-bit */
    g->dark_r = (INT)ir_driver_get_dark(IR_CHANNEL_RIGHT);
#else
    g->dark_c = (INT)g_lb_stats.line_dark_centre;
    g->dark_r = (INT)g_lb_stats.line_dark_right;
#endif
    g->err = (INT)g_lb_stats.line_err;
    g->steer = (INT)g_lb_stats.line_steer;
    g->mode = (UB)g_lb_stats.line_mode;
    g->bc_edges = g_lb_stats.bc_edges;
    g->bc_ok = g_lb_stats.bc_decoded;
    g->bc_fail = g_lb_stats.bc_failed;
    g->bc_last = (UB)g_lb_stats.bc_last;
    g->ultra_mm = s5_scan_last_mm;
    g->scan_state = s5_scan_state;
    g->scan_side = s5_scan_side;
    g->bc_exit = s5_bc_exit;
    g->byp_result = s5_byp_result;
    g->width_mm = s5_scan_width_mm;
    g->half_l_mm = s5_scan_half_mm[0];
    g->bc_status = (UB)g_lb_stats.bc_status;
    g->bc_count = g_lb_stats.bc_count;
    g->motor_state = MOT_STOPPED;
    if (state_lock(TMO_POL) == E_OK) {
        g->motor_state = app_state.motor_state;
        state_unlock();
    }
}
#endif

#if DASH_DEBUG_FRAMES
/* The IMU subsystem's live values (read without a lock: a display only) */
LOCAL void sample_imu(DASH_IMU *m)
{
    m->state = s3_state;
    m->calibrated = s3_calibrated ? 1 : 0;
    m->lean_um = s3_h_um;
    m->last_mm = s3_last_mm;
    m->max_mm = s3_max_mm;
    m->humps = s3_humps;
    m->rise = s3_rise;
}
#endif

EXPORT void dashboard_task(INT stacd, void *exinf)
{
    static MQTT_MSG in;
    static char     json[DASH_JSON_MAX];
    UB              frame[DASH_DEBUG_BYTES];
    UB              telem_seq = 0;
#if DASH_DEBUG_FRAMES
    UB              debug_seq = 0;
    DASH_DEBUG      g;
    UB              bc_frame[DASH_BC_TRACE_MAX];
    UW              bc_edges[DASH_BC_TRACE_EDGES];
    UW              bc_seq_sent = 0;
    UB              imu_seq = 0;
    DASH_IMU        imu;
#endif
    UB              stored = 0;         /* Mission's letter for the next junction */
    DASH            d;
    DASH_IN         din;
    DASH_OUT        dout;
    MSG_TELEM       t, latest;
    MSG_MQTTCMD     c;
    INT             len;
    UB              cmd;
#if APP_TEST && TM_WIFI_CYW43 && TM_NET_LWIP
    INT             wifi_report_ms = 0;
#endif

    (void)stacd; (void)exinf;
    dash_init(&d);
    latest.sys_state = SYS_INIT;

    while (1) {
        tk_dly_tsk(DASH_TICK_MS);
#if APP_TEST && TM_WIFI_CYW43 && TM_NET_LWIP
        wifi_report_ms += DASH_TICK_MS;
        if (wifi_report_ms >= 5000) {
            wifi_report_ms = 0;
            report_wifi_test_status();
        }
#endif

        din.dt_ms = DASH_TICK_MS;
        din.wifi_up = wifi_is_up();
        din.mqtt_event = 0;
        din.got_telem = FALSE;

        while (MBF_RECV(mbf_mqtt_in, in, TMO_POL) > 0) {
            if (in.kind == MQTTM_CONNECTED || in.kind == MQTTM_DISCONNECTED) {
                din.mqtt_event = in.kind;
            } else if (in.kind == MQTTM_COMMAND) {
                cmd = dash_parse_cmd(in.data, in.len);
                if (cmd) {
                    c.cmd = cmd;
                    MBF_SEND(mbf_mqttcmd, c);
                }
            } else if (in.kind == MQTTM_MAP) {
                if (dash_parse_map(in.data, in.len, c.letters)) {
                    c.cmd = MQTTCMD_MAP;
                    MBF_SEND(mbf_mqttcmd, c);
                    tm_printf((UB*)"[map] U-turn %c, left %c, right %c, straight %c\n",
                          c.letters[0] ? c.letters[0] : '-',
                          c.letters[1] ? c.letters[1] : '-',
                          c.letters[2] ? c.letters[2] : '-',
                          c.letters[3] ? c.letters[3] : '-');
                }
            }
        }

        while (MBF_RECV(mbf_telem, t, TMO_POL) > 0) {
            latest = t;
            din.got_telem = TRUE;
        }

        dash_step(&d, &din, &dout);
        s4_state = d.state;
        if (state_lock(TMO_POL) == E_OK) {
            app_state.net_online = (d.state == DASH_ONLINE || d.state == DASH_HEARTBEAT) ? 1 : 0;
            stored = app_state.stored_char;
            state_unlock();
        }

        if (din.got_telem && dout.publish_telem) {
            /* online: the 11-byte binary frame (TEAM.md 7) */
            len = dash_pack_telem(&latest, telem_seq++, dash_char_code(stored),
                          frame, (INT)sizeof(frame));
            if (len > 0) {
                s4_frames++;
                send_out(MQTTM_PUBLISH, frame, len);
            }
        } else if (din.got_telem) {
            /* not online yet: show the frame on the console */
            len = dash_format_telem(&latest, json, (INT)sizeof(json));
            if (len > 0) {
                s4_frames++;
                tm_printf((UB*)"[telemetry] %s\n", json);
                s4_console_lines++;
            }
        }
        if (dout.publish_heartbeat) {
            frame[0] = latest.sys_state;
            send_out(MQTTM_HEARTBEAT, frame, DASH_HB_BYTES);
        }
#if DASH_DEBUG_FRAMES
        if (d.state == DASH_ONLINE || d.state == DASH_HEARTBEAT) {
            sample_debug(&g);
            len = dash_pack_debug(&g, debug_seq++, frame, (INT)sizeof(frame));
            if (len > 0) {
                send_out(MQTTM_DEBUG, frame, len);
            }
            if ((debug_seq % DASH_IMU_EVERY) == 0) {
                sample_imu(&imu);
                len = dash_pack_imu(&imu, imu_seq++, frame, (INT)sizeof(frame));
                if (len > 0) {
                    send_out(MQTTM_IMU, frame, len);
                }
            }
            /* A capture finished: send its element widths once. */
            if (g_lb_bc_trace.seq != bc_seq_sent) {
                INT n = (INT)g_lb_bc_trace.count, k;

                bc_seq_sent = g_lb_bc_trace.seq;
                if (n > DASH_BC_TRACE_EDGES) {
                    n = DASH_BC_TRACE_EDGES;
                }
                for (k = 0; k < n; k++) {
                    bc_edges[k] = g_lb_bc_trace.edge_us[k];
                }
                len = dash_pack_bc_trace((UB)bc_seq_sent, (UB)g_lb_bc_trace.status,
                             bc_edges, n, bc_frame, (INT)sizeof(bc_frame));
                if (len > 0) {
                    send_out(MQTTM_BARCODE, bc_frame, len);
                }
            }
        }
#endif
    }
}

/*** end of file ***/
