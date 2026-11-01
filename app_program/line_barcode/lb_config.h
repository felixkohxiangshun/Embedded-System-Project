/** @file lb_config.h
 *
 * @brief Every tunable constant of the line and barcode subsystem (Buddy 3).
 *
 * @par
 * Read this file first.  Each value says what it means, its unit and how to
 * measure it.  Values marked [CAL] are starting points that MUST be measured
 * again on the real car, track and venue lighting (docs/BUDDY3_GUIDE.md,
 * section "Calibration").  Nothing here touches hardware or the kernel, so
 * the same file is used by the car firmware and by the PC unit tests.
 *
 * @par
 * Conventions used by every module of this subsystem:
 *  - "dark scale": a count where a HIGHER number is a DARKER surface (the
 *    polarity switch below makes every sensor look like this).
 *  - "darkness": a normalised value, 0 (bare paper) to 1000 (fresh black).
 *  - "steer": -1000 (hard left) to +1000 (hard right), as in app_if.h.
 *  - "error": line position relative to the point midway between the left
 *    and the centre sensor, -500 (line under the left sensor) to +500 (line
 *    under the centre sensor).  Negative means "steer left".
 *
 * @par
 * Track geometry used by the barcode constants (Barcode & Line
 * Specification): the line is 18 mm wide, the barcode lies beside the line
 * with a 20 mm gap, and it is about 141 mm long.  A "*X*" Code 39 message is
 * 47 narrow units long (3 characters of 15 units plus 2 gaps of 1 unit), so
 * one narrow bar is 141 / 47 = 3 mm and one wide bar is 9 mm.
 *
 * @par
 * Documented deviation from BARR-C:2018 (rule 7.1j): the kernel object IDs
 * mbf_edge and mbf_steer are named by the team interface file app_if.h
 * (TEAM.md section 5), so they do not carry the "g_" prefix.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#ifndef LB_CONFIG_H
#define LB_CONFIG_H

/* ------------------------------------------------------------------------
 * 1. Sensor hardware (RP2040 12-bit ADC, three IR reflectance sensors)
 * --------------------------------------------------------------------- */

/* Largest value the 12-bit ADC can return. */
#define LB_ADC_MAX_COUNT            4095

/* Polarity: 1 if the sensor module outputs a HIGHER voltage over black
 * (typical for a TCRT5000 module wired to its analogue pin; applies to the
 * two line sensors, which are analogue), 0 if it outputs
 * a LOWER voltage over black.  [CAL] The guided calibration in the test
 * build (LB_TEST_CALIBRATE below) prints the right value. */
#define LB_IR_DARK_IS_HIGH          1

/* Barcode sensor input mode.  The right sensor (GP28) is wired to the
 * module's DIGITAL output pin (DO) and is read as a plain GPIO level:
 *   - the line sensors (GP26, GP27) stay analogue (AO) and use the ADC;
 *   - the barcode sensor (GP28) is digital (DO) and does not use the ADC.
 * Set the module's trimmer so that DO flips cleanly between a bar and bare
 * paper (check the module's DO LED).  The level is converted to two fixed
 * dark-scale counts and goes through the same slicer, so everything after
 * the sampler is identical to the analogue mode.
 * 1 = digital (current hardware), 0 = analogue (wire AO to GP28 instead). */
#define LB_BC_USE_GPIO_LEVEL        1

/* Digital mode: 1 if DO is HIGH over a dark bar, 0 if DO is LOW over a
 * dark bar.  [CAL] Typical TCRT5000 / LM393 modules pull DO LOW over a
 * reflective (white) surface and release it HIGH over black, so 1 is the
 * usual setting; verify on the car.  The two counts below stand for "bar"
 * and "paper"; the slicer only uses their difference. */
#define LB_BC_GPIO_DARK_IS_HIGH     1
#define LB_BC_GPIO_DARK_COUNT       3600
#define LB_BC_GPIO_LIGHT_COUNT      500

/* 1 kHz sampler: conversions per sensor that are averaged.  One extra
 * conversion is read and thrown away after each channel change so that the
 * multiplexer can settle (see ir_driver.c). */
#define LB_ADC_OVERSAMPLE           2

/* The two line sensors are averaged over this many 1 ms samples (a sliding
 * window).  10 samples span exactly one period of the 100 Hz flicker that
 * mains lighting (50 Hz grid) produces, so that flicker cancels out. */
#define LB_LINE_AVG_SAMPLES         10u

/* ------------------------------------------------------------------------
 * 2. Line sensor calibration (dark scale, counts)  [CAL]
 * ---------------------------------------------------------------------
 * white = sensor centred over bare paper, black = sensor centred over the
 * fresh black line.  The span must be at least LB_CAL_SPAN_MIN counts. */
#define LB_WHITE_LEFT               400
#define LB_BLACK_LEFT               3200
#define LB_WHITE_CENTER             400
#define LB_BLACK_CENTER             3200
#define LB_CAL_SPAN_MIN             800

/* Largest normalised darkness (see ir_calib.h). */
#define LB_DARK_MAX                 1000

/* ------------------------------------------------------------------------
 * 3. Line following (line_ctrl.c)
 * --------------------------------------------------------------------- */

/* The line task runs every this many milliseconds (requirement NFR-03). */
#define LB_LINE_TICK_MS             10

/* Steering limit.  Must equal STEER_MAX in app_if.h (checked at compile
 * time in line_task.c). */
#define LB_STEER_MAX                1000

/* Position (-1000 left sensor .. 0 centre sensor) at which the line is
 * exactly where we want it: midway between the two sensors. */
#define LB_LINE_TARGET              (-500)

/* PD gains in 1/256 units: steer = (KP * err + KD * (err - previous)) / 256.
 * err is -500..+500.  [CAL] Tune on the track: raise KP until the car just
 * starts to wobble, back off 30 percent, then add KD to damp the wobble. */
#define LB_GAIN_SCALE               256
#define LB_KP                       400
#define LB_KD                       500

/* Largest change of the steering output per tick.  Stops the wheels being
 * jerked when the error jumps (dirt, a junction edge, re-acquiring). */
#define LB_STEER_SLEW_MAX           250

/* The steering output is also kept as a slow average ("steer memory"):
 * mem = mem + (steer - mem) / this.  It is the value that is held while
 * the line is missing, so one noisy tick cannot send the car off. */
#define LB_STEER_MEM_DIV            4

/* "Line mass" = left darkness + centre darkness.  With the two sensors
 * straddling the line it stays near LB_MASS_NOMINAL wherever the line is
 * between them, so it directly measures how strongly the line shows.
 * [CAL] Drive over the plain line and log the mass, then set NOMINAL to the
 * LOWEST value you see there (not the average). */
#define LB_MASS_NOMINAL             1000

/* The line is "seen" when the mass is at least SEEN_ON and stays seen until
 * it falls below SEEN_OFF (hysteresis stops flicker at the limit).  These
 * are deliberately low (about 14 and 10 percent of nominal) so a faded
 * line is still followed.  They must stay well above the sensor noise on
 * white paper, which you measure in calibration step C2. */
#define LB_MASS_SEEN_ON             140
#define LB_MASS_SEEN_OFF            100

/* FADED mode: entered when the averaged mass falls below FADED_ON, left when
 * it rises above FADED_OFF (60 and 70 percent of nominal).  The mass is
 * averaged as avg = avg + (mass - avg) / LB_MASS_AVG_DIV. */
#define LB_MASS_FADED_ON            600
#define LB_MASS_FADED_OFF           700
#define LB_MASS_AVG_DIV             2

/* In FADED mode the position estimate is noisier, so steer more gently:
 * the gains are scaled by these percentages, the output is limited, and the
 * error is smoothed with err = err + (new - err) / LB_FADED_ERR_IIR_DIV. */
#define LB_FADED_KP_PCT             100
#define LB_FADED_KD_PCT             60
#define LB_FADED_STEER_MAX          900
#define LB_FADED_ERR_IIR_DIV        2

/* A single missing sample is bridged (the last steering is held); the line
 * is declared vanished after this many consecutive ticks without it. */
#define LB_LOST_MISS_TICKS          2

/* The line must be seen on this many consecutive ticks before the search
 * ends and normal tracking resumes. */
#define LB_REACQ_TICKS              3

/* ------------------------------------------------------------------------
 * 4. Recovery when the line vanishes or the car veers off (line_recov.c)
 * ---------------------------------------------------------------------
 * COAST: keep the course for a moment: the last smoothed steering is held
 *   (so a bend is still followed) and fades to straight over the last
 *   quarter of the coast.
 *   If the error was small when the line vanished, the LINE faded (the car
 *   is still on its path), so coast longer.  If the error was large, the CAR
 *   left the line, so coast only briefly and search at once. */
#define LB_VEER_ERR_MIN             300
#define LB_COAST_VEER_MS            100
#define LB_COAST_FADE_MS            400

/* SWEEP: swing toward the side where the line was last seen, on top of the
 * remembered steering, the swing ramping from MIN to MAX over RAMP_MS.  If
 * the line is not found within
 * NEAR_MS the sweep reverses for FAR_MS, then keeps alternating, each leg a
 * little stronger (GROW) up to CEIL.  Units: steer, milliseconds. */
#define LB_SWEEP_NEAR_MS            400
#define LB_SWEEP_FAR_MS             700
#define LB_SWEEP_RAMP_MS            250
#define LB_SWEEP_STEER_MIN          350
#define LB_SWEEP_STEER_MAX          750
#define LB_SWEEP_STEER_GROW         100
#define LB_SWEEP_STEER_CEIL         950

/* If the last error was smaller than this the side of the line is unknown
 * from the error, so the sign of the held steering decides.  If that is
 * also tiny, the sweep starts toward the side below (+1 = right). */
#define LB_SIDE_ERR_MIN             50
#define LB_SIDE_STEER_MIN           30
#define LB_SIDE_DEFAULT             1

/* The line has been missing this long: tell Mission (STEER_F_TIMEOUT). */
#define LB_TIMEOUT_MS               3000

/* 1 = count the lost-line time only while Mission is following the line.
 * While the car is idle, turning, or bypassing an obstacle the line is
 * expected to be missing and must not trigger a timeout.  The state is
 * read from app_state.sys_state with state_lock(TMO_POL). */
#define LB_PAUSE_TIMEOUT_OFF_LINE   1

/* The longest time step the recovery trusts, in milliseconds.  If the line
 * task is ever delayed, a late tick is treated as this long at most. */
#define LB_DT_MAX_MS                50

/* ------------------------------------------------------------------------
 * 5. Junction detection (line_ctrl.c)
 * ---------------------------------------------------------------------
 * Junction: both line sensors fully black, as when a bar crosses the line.
 * Only valid if the sensor spacing is at least the line width; otherwise
 * both sensors are black on a plain line.  Set LB_JUNCTION_ENABLE to 0 then.
 * While a junction is under the car the steering is held straight. */
#define LB_JUNCTION_ENABLE          1
#define LB_JUNCTION_DARK_MIN        850
#define LB_JUNCTION_TICKS           3u

/* ------------------------------------------------------------------------
 * 6. Barcode slicer: turns the right sensor into bar/space edges
 *    (bc_slicer.c)
 * ---------------------------------------------------------------------
 * The slicer follows the signal's own peaks and valleys instead of a fixed
 * threshold.  A move counts as a bar edge only if the signal travels at
 * least the PROMINENCE, and the edge is placed where the signal crosses
 * the mid-level between that valley and that peak.  This keeps the edges
 * in the right place when the sensor spot is wider than a narrow bar, when
 * room light shifts the whole signal, and when the ink is weak. */

/* Nominal paper and ink levels of the barcode sensor (dark scale, counts).
 * Only their difference matters: it is the full swing of a bar.  In digital
 * mode the sensor can only report the two fixed counts above, so the
 * nominal levels are taken from them.  In analogue mode they are measured
 * values.  [CAL] */
#if (LB_BC_USE_GPIO_LEVEL != 0)
#define LB_BC_WHITE_NOMINAL         LB_BC_GPIO_LIGHT_COUNT
#define LB_BC_BLACK_NOMINAL         LB_BC_GPIO_DARK_COUNT
#else
#define LB_BC_WHITE_NOMINAL         400
#define LB_BC_BLACK_NOMINAL         3200
#endif

/* Smallest signal movement that counts as an edge, in percent of the
 * nominal swing (about 8 percent of the swing).  It
 * must be several times the noise measured on bare paper (calibration step
 * C2) and well below the swing of the narrowest bars after blur.  [CAL] */
#define LB_BC_PROMINENCE_PCT        8

/* A rise or fall that has moved at least the prominence is accepted as
 * finished when the signal has not gone further for this many samples
 * (milliseconds).  Without it the last edge of a code, which is followed by
 * bare paper, would never be reported. */
#define LB_BC_SETTLE_SAMPLES        15u

/* Samples remembered to place an edge accurately.  It must cover the
 * longest rise or fall: the sensor spot width divided by the lowest car
 * speed (for example 10 mm at 40 mm/s = 250 ms), plus the settle time. */
#define LB_BC_HISTORY_SAMPLES       512u

/* No edges are reported for this many samples after start-up while the
 * ADC and the filters settle. */
#define LB_BC_WARMUP_SAMPLES        50

/* ------------------------------------------------------------------------
 * 7. Barcode decoder (bc_decode.c) and edge stream (bc_stream.c)
 * ---------------------------------------------------------------------
 * Code 39, "*X*": three characters of nine elements, 30 edges.  The decoder
 * needs no absolute width, only ratios. */

/* Within one character the narrowest wide element must be at least this
 * many tenths of the widest narrow element of the same kind (bar or space).
 * 12 means 1.2 times; the ideal printed ratio is 3.0.  A sensor spot wider
 * than a narrow bar makes every narrow element look about one spot wide,
 * which squeezes the real ratio (3 mm / 9 mm bars seen by a 6 mm spot look
 * like 6 mm / 9 mm, a ratio of 1.5), so this value must be low.  The
 * decoder stays safe because it also demands exactly two wide bars and one
 * wide space per character, two '*' guards, plausible gaps and a steady
 * speed. */
#define LB_BC_WIDE_RATIO_X10        12u

/* The three characters must be read at similar speed: their narrow bar
 * widths may differ by at most this many tenths (30 means 3 times). */
#define LB_BC_SPEED_RATIO_X10       30u

/* The gap between characters must lie between these tenths of the narrow
 * space width of its two neighbours (4 to 30 means 0.4 x to 3 x). */
#define LB_BC_GAP_MIN_X10           4u
#define LB_BC_GAP_MAX_X10           30u

/* A pulse shorter than this many tenths of the median element is a glitch
 * (a speck of dust, a scuff) and is merged away (4 means 0.4 x). */
#define LB_BC_GLITCH_X10            4u

/* 1 = a barcode read backwards (car driving the other way) is decoded and
 * acted on.  0 = it is recognised but reported as a failure.  The commands
 * are written for the forward direction, so the default is 0. */
#define LB_BC_ACCEPT_REVERSED       0

/* A capture that goes quiet ends after max(MIN, FACTOR x longest element),
 * limited to MAX milliseconds, so slow driving never cuts a code short. */
#define LB_BC_SILENCE_MIN_MS        150u
#define LB_BC_SILENCE_FACTOR        4u
#define LB_BC_SILENCE_MAX_MS        2000u

/* Most edges remembered for one capture (30 are a full code; the rest is
 * room for noise edges in front of or inside the code). */
#define LB_BC_MAX_EDGES             40u

/* Mission is told "barcode seen" only after this many edges were collected,
 * so one speck of dust or the edge of the line does not disturb it. */
#define LB_BC_SEEN_EDGES            6u

/* Messages that fit into the edge buffer between the 1 kHz sampler and the
 * barcode task (two complete codes). */
#define LB_EDGE_BUFFER_COUNT        64

/* ------------------------------------------------------------------------
 * 8. Test build only (make TEST=1; see line_barcode_test.c)
 * --------------------------------------------------------------------- */

/* 1 = at power-up, run the guided calibration (about 14 s): hold the car
 * over white paper, then over black, and read the constants to paste into
 * this file from the console. */
#define LB_TEST_CALIBRATE           0

/* Seconds of live trace printed after the tests (0 = none).  One line every
 * 100 ms with the raw readings, line error, steering, mode and barcode
 * state.  Press the start button and drive the car meanwhile. */
#define LB_TEST_TRACE_SECONDS       0

/* ------------------------------------------------------------------------
 * Compile-time checks of the relations the algorithms rely on.
 * --------------------------------------------------------------------- */
#if (LB_MASS_SEEN_OFF >= LB_MASS_SEEN_ON)
#error "LB_MASS_SEEN_OFF must be below LB_MASS_SEEN_ON (hysteresis)"
#endif

#if (LB_MASS_FADED_ON >= LB_MASS_FADED_OFF)
#error "LB_MASS_FADED_ON must be below LB_MASS_FADED_OFF (hysteresis)"
#endif

#if (LB_MASS_SEEN_ON >= LB_MASS_FADED_ON)
#error "LB_MASS_SEEN_ON must be below LB_MASS_FADED_ON"
#endif

#if ((LB_BC_USE_GPIO_LEVEL != 0) \
     && (LB_BC_GPIO_DARK_COUNT <= LB_BC_GPIO_LIGHT_COUNT))
#error "LB_BC_GPIO_DARK_COUNT must be larger than LB_BC_GPIO_LIGHT_COUNT"
#endif

#if (LB_MASS_FADED_OFF > LB_MASS_NOMINAL)
#error "LB_MASS_FADED_OFF must not exceed LB_MASS_NOMINAL"
#endif

#if (LB_BC_MAX_EDGES < 30u)
#error "LB_BC_MAX_EDGES must hold at least one complete code (30 edges)"
#endif

#if (LB_BC_SEEN_EDGES >= 30u)
#error "LB_BC_SEEN_EDGES must be below one complete code (30 edges)"
#endif

#if (LB_SWEEP_STEER_MIN > LB_SWEEP_STEER_MAX)
#error "LB_SWEEP_STEER_MIN must not exceed LB_SWEEP_STEER_MAX"
#endif

#if (LB_SWEEP_STEER_CEIL > LB_STEER_MAX)
#error "LB_SWEEP_STEER_CEIL must not exceed LB_STEER_MAX"
#endif

#if (LB_FADED_STEER_MAX > LB_STEER_MAX)
#error "LB_FADED_STEER_MAX must not exceed LB_STEER_MAX"
#endif

#if (LB_BC_SETTLE_SAMPLES >= LB_BC_HISTORY_SAMPLES)
#error "LB_BC_SETTLE_SAMPLES must be shorter than the sample history"
#endif

#if (LB_BC_PROMINENCE_PCT < 1) || (LB_BC_PROMINENCE_PCT > 50)
#error "LB_BC_PROMINENCE_PCT must be between 1 and 50"
#endif

#endif /* LB_CONFIG_H */
/*** end of file ***/
