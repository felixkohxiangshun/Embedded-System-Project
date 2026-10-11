/*
 *	task_motor.c
 *	Motor control -- the Motor task: PID speed control of both wheels and the
 *	Motor statechart (design review 6.3).
 *
 *	States: STOPPED, DRIVING, TURNING, BYPASS, ENCODER_FAULT
 *
 *	Inputs   mbf_nav   steering and mode commands from Mission (IF-08)
 *	         flg_evt   START / STOP / RESUME from Mission (IF-10)
 *	         enc_cnt   quadrature counts updated by the GPIO ISR
 *	Outputs  PWM duty on both wheels, and speed/distance/state written to
 *	         app_state under mtx_state for Telemetry and Mission.
 *
 *	The task is woken every 10 ms by motor_cyclic_handler (NFR-14).  It
 *	never blocks on anything else: IPC is polled with TMO_POL.
 */

#include "motor_control.h"

EXPORT volatile UW s1_ticks;
EXPORT volatile UB s1_state;

/* A bypass is a short list of segments: pivot turns and straight runs */
#define SEG_TURN	1	/* val = degrees, + right */
#define SEG_FWD		2	/* val = mm */
#define BYPASS_LEG_MM	150	/* extra distance beside the obstacle [CAL] */
#define BYPASS_MARGIN_MM 60	/* sideways clearance added to half width [CAL] */
#define MAX_SEGS	8

typedef struct { UB kind; INT val; } SEG;

LOCAL SEG   segs[MAX_SEGS];
LOCAL INT   seg_n, seg_i;
LOCAL INT   seg_goal;		/* counts the current segment must travel */
LOCAL INT   seg_base_l, seg_base_r;

LOCAL PID   pid[2];
LOCAL UB    state = MOT_STOPPED;
LOCAL INT   steer;
LOCAL INT   speed_set = SPEED_NORMAL_MMPS;
LOCAL INT   last_cnt[2];
LOCAL INT   speed[2];
LOCAL INT   total_counts;	/* sum of |delta| both wheels, for distance */
LOCAL INT   drift_acc;
LOCAL INT   stuck_ticks[2];
LOCAL INT   duty[2];
LOCAL UW    done_seq;

EXPORT void motor_cyclic_handler(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_motor);
}

LOCAL void enter(UB new_state)
{
	if (new_state != state) {
		pid_reset(&pid[0]);
		pid_reset(&pid[1]);
		drift_acc = 0;
		stuck_ticks[0] = stuck_ticks[1] = 0;
	}
	state = new_state;
	s1_state = new_state;
}

LOCAL void stop_motors(void)
{
	duty[0] = duty[1] = 0;
	motor_stop_all();
}

/* Begin segment seg_i of the current sequence */
LOCAL void seg_begin(void)
{
	seg_base_l = enc_cnt[0];
	seg_base_r = enc_cnt[1];
	if (segs[seg_i].kind == SEG_TURN) {
		INT deg = segs[seg_i].val < 0 ? -segs[seg_i].val : segs[seg_i].val;

		seg_goal = deg_to_counts(deg);
	} else {
		seg_goal = mm_to_counts(segs[seg_i].val);
	}
}

LOCAL void seq_start(UB motor_state)
{
	seg_i = 0;
	seg_begin();
	enter(motor_state);
}

LOCAL void start_turn(INT degrees)
{
	segs[0].kind = SEG_TURN;
	segs[0].val = degrees;
	seg_n = 1;
	seq_start(MOT_TURNING);
}

/* Go round an obstacle of 'width' mm on 'side' (+1 right, -1 left):
   out, past, back, onto the line, then face along it again. */
LOCAL void start_bypass(INT side, INT width)
{
	INT lateral = width / 2 + BYPASS_MARGIN_MM;
	INT deg = 90 * (side >= 0 ? 1 : -1);

	segs[0].kind = SEG_TURN; segs[0].val = deg;
	segs[1].kind = SEG_FWD;  segs[1].val = lateral;
	segs[2].kind = SEG_TURN; segs[2].val = -deg;
	segs[3].kind = SEG_FWD;  segs[3].val = BYPASS_LEG_MM + width;
	segs[4].kind = SEG_TURN; segs[4].val = -deg;
	segs[5].kind = SEG_FWD;  segs[5].val = lateral;
	segs[6].kind = SEG_TURN; segs[6].val = deg;
	seg_n = 7;
	seq_start(MOT_BYPASS);
}

LOCAL void handle_nav(const MSG_NAV *m)
{
	if (state == MOT_ENCODER_FAULT) {
		return;
	}
	switch (m->type) {
	case NAV_STEER:
		steer = clamp_i(m->arg, -STEER_MAX, STEER_MAX);	/* used in DRIVING only */
		break;
	case NAV_SLOW:
		speed_set = SPEED_SLOW_MMPS;
		break;
	case NAV_RESUME:
		speed_set = SPEED_NORMAL_MMPS;
		break;
	case NAV_TURN:
		if (state != MOT_TURNING && state != MOT_BYPASS) {
			start_turn(m->arg);
		}
		break;
	case NAV_BYPASS:
		if (state != MOT_TURNING && state != MOT_BYPASS) {
			start_bypass(m->arg, m->arg2);
		}
		break;
	default:
		break;
	}
}

LOCAL void handle_events(void)
{
	UINT ptn;

	/* tk_wai_flg with TMO_POL: poll, never wait.  TWF_BITCLR clears only
	   the bits we asked for, so EVT_BTN stays for Mission. */
	if (tk_wai_flg(flg_evt, EVT_START | EVT_STOP | EVT_RESUME,
		       TWF_ORW | TWF_BITCLR, &ptn, TMO_POL) != E_OK) {
		return;
	}
	if (ptn & EVT_STOP) {
		steer = 0;
		stop_motors();
		enter(MOT_STOPPED);
	} else if (ptn & (EVT_START | EVT_RESUME)) {
		if (state == MOT_STOPPED || state == MOT_ENCODER_FAULT) {
			steer = 0;
			speed_set = SPEED_NORMAL_MMPS;
			enter(MOT_DRIVING);
		}
	}
}

/* Closed-loop one wheel: feed-forward plus PID on the measured speed */
LOCAL INT wheel_duty(INT w, INT target_mmps)
{
	INT d = DUTY_FF_PER_MMPS * target_mmps + pid_step(&pid[w], target_mmps, speed[w]);

	if (target_mmps == 0) {
		pid_reset(&pid[w]);
		return 0;
	}
	return clamp_i(d, -DUTY_MAX, DUTY_MAX);
}

LOCAL INT iabs(INT v)
{
	return v < 0 ? -v : v;
}

/* Has the current segment travelled far enough? */
LOCAL BOOL seg_done(void)
{
	INT dl = iabs(enc_cnt[0] - seg_base_l);
	INT dr = iabs(enc_cnt[1] - seg_base_r);

	return ((dl + dr) / 2 >= seg_goal) ? TRUE : FALSE;
}

LOCAL void step_sequence(INT *tl, INT *tr)
{
	INT sgn;

	if (seg_done()) {
		seg_i++;
		if (seg_i >= seg_n) {
			stop_motors();
			done_seq++;		/* tell Mission the command ended */
			enter(MOT_STOPPED);
			*tl = *tr = 0;
			return;
		}
		seg_begin();
	}
	if (segs[seg_i].kind == SEG_TURN) {
		sgn = segs[seg_i].val >= 0 ? 1 : -1;	/* + = right: left wheel forward */
		*tl = +sgn * SPEED_TURN_MMPS;
		*tr = -sgn * SPEED_TURN_MMPS;
	} else {
		*tl = *tr = SPEED_SLOW_MMPS;
	}
}

LOCAL void check_encoders(void)
{
	INT w;

	for (w = 0; w < 2; w++) {
		if (iabs(duty[w]) > ENC_FAULT_DUTY && speed[w] == 0) {
			stuck_ticks[w]++;
		} else {
			stuck_ticks[w] = 0;
		}
		if (stuck_ticks[w] >= ENC_FAULT_TICKS) {
			stop_motors();
			enter(MOT_ENCODER_FAULT);
			return;
		}
	}
}

LOCAL void publish(void)
{
	/* Never wait for the lock: a stale sample is better than a late PID */
	if (state_lock(TMO_POL) != E_OK) {
		return;
	}
	app_state.motor_state = state;
	app_state.speed_mmps = (speed[0] + speed[1]) / 2;
	app_state.dist_mm = (UW)(((total_counts / 2) * WHEEL_MM_PER_REV) / ENC_COUNTS_PER_REV);
	app_state.enc_left = enc_cnt[0];
	app_state.enc_right = enc_cnt[1];
	app_state.cmd_done_seq = done_seq;
	if (state == MOT_ENCODER_FAULT) {
		app_state.fault_flags |= FAULT_ENCODER;
	} else {
		app_state.fault_flags &= (UB)~FAULT_ENCODER;
	}
	state_unlock();
}

EXPORT void motor_task(INT stacd, void *exinf)
{
	MSG_NAV	nav;
	INT	now[2], dl, dr, tl, tr, w;

	(void)stacd; (void)exinf;

	pid_init(&pid[0], PID_KP, PID_KI, PID_KD, -DUTY_MAX, DUTY_MAX);
	pid_init(&pid[1], PID_KP, PID_KI, PID_KD, -DUTY_MAX, DUTY_MAX);
	last_cnt[0] = enc_cnt[0];
	last_cnt[1] = enc_cnt[1];
	enter(MOT_STOPPED);

	while (1) {
		tk_slp_tsk(TMO_FEVR);		/* woken by motor_cyclic_handler */
		tk_can_wup(TSK_SELF);		/* drop catch-up wake-ups */
		s1_ticks++;

		/* 1. commands */
		handle_events();
		while (MBF_RECV(mbf_nav, nav, TMO_POL) > 0) {
			handle_nav(&nav);
		}

		/* 2. wheel speeds from the encoders */
		now[0] = enc_cnt[0];
		now[1] = enc_cnt[1];
		dl = now[0] - last_cnt[0];
		dr = now[1] - last_cnt[1];
		last_cnt[0] = now[0];
		last_cnt[1] = now[1];
		speed[0] = counts_to_mmps(dl);
		speed[1] = counts_to_mmps(dr);
		total_counts += iabs(dl) + iabs(dr);

		/* 3. state machine -> wheel set-points */
		tl = tr = 0;
		switch (state) {
		case MOT_DRIVING:
			if (iabs(steer) < TRIM_DEADBAND) {
				/* FR-13: straight run, equalise the two wheels */
				drift_acc += dl - dr;
				w = clamp_i(drift_acc / 8, -TRIM_MAX_MMPS, TRIM_MAX_MMPS);
			} else {
				drift_acc = 0;
				w = 0;
			}
			tl = speed_set + (steer * STEER_GAIN_MMPS) / STEER_MAX - w;
			tr = speed_set - (steer * STEER_GAIN_MMPS) / STEER_MAX + w;
			break;
		case MOT_TURNING:
		case MOT_BYPASS:
			step_sequence(&tl, &tr);
			break;
		default:			/* STOPPED, ENCODER_FAULT */
			break;
		}

		/* 4. PID and PWM */
		if (state == MOT_STOPPED || state == MOT_ENCODER_FAULT) {
			stop_motors();
		} else {
			duty[0] = wheel_duty(0, tl);
			duty[1] = wheel_duty(1, tr);
			motor_drive(0, duty[0]);
			motor_drive(1, duty[1]);
			check_encoders();
		}

		/* 5. report */
		publish();
	}
}
