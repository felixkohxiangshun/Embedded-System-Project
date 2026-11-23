/*
 *	replay.c
 *	Feed a recording made with S3_TEST_LOG (a PuTTY log with LOG_BEGIN ...
 *	LOG_END) through the REAL imu_logic.c, with the axis settings of
 *	imu_terrain.h, and print what the IMU would report (test S3-R01).
 *
 *	Build and run (from testing/imu_terrain):
 *	    gcc -O2 -I../host/stub -I../../app_program -o replay replay.c \
 *	        ../../app_program/imu_terrain/imu_logic.c
 *	    ./replay logs/2026-11-01_hump_run.txt          events + summary
 *	    ./replay logs/2026-11-01_hump_run.txt trace    + height every 10 samples
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "imu_terrain/imu_terrain.h"

static IMU_LOGIC l;

static void to_car(const int raw[3], IMU_VEC *a)
{
	a->fwd = IMU_SIGN_FWD * raw[IMU_AXIS_FWD];
	a->up = IMU_SIGN_UP * raw[IMU_AXIS_UP];
	a->side = IMU_SIGN_SIDE * raw[3 - IMU_AXIS_FWD - IMU_AXIS_UP];
}

int main(int argc, char **argv)
{
	FILE *f;
	char line[512];
	int in_log = 0, n = 0, rest[3] = { 0, 0, 1000 }, k, humps = 0;
	int trace = (argc > 2) && (strcmp(argv[2], "trace") == 0);
	long dist = 0;

	if (argc < 2 || (f = fopen(argv[1], "r")) == NULL) {
		fprintf(stderr, "usage: replay <putty log> [trace]\n");
		return 2;
	}
	imu_logic_init(&l);
	while (fgets(line, sizeof line, f)) {
		int i, raw[3], ds, rh, mot, st;
		IMU_VEC a;
		char *p = strstr(line, "rest=");

		if (strncmp(line, "LOG_BEGIN", 9) == 0) {
			if (p) {
				int rf = 0, ru = 2, rx[3];
				char *q = strstr(line, "axis_fwd=");
				char *u = strstr(line, "axis_up=");

				sscanf(p, "rest=%d,%d,%d", &rest[0], &rest[1], &rest[2]);
				if (q) sscanf(q, "axis_fwd=%d", &rf);
				if (u) sscanf(u, "axis_up=%d", &ru);
				/* rest was printed in the RECORDING firmware's car axes
				   (fwd, side, up with its axis_fwd / axis_up, side sign +):
				   turn it back into sensor X, Y, Z */
				rx[rf] = rest[0];
				rx[ru] = rest[2];
				rx[3 - rf - ru] = rest[1];
				to_car(rx, &a);
			} else {
				a.fwd = 0;
				a.side = 0;
				a.up = 1000;
			}
			/* the car stood still at power-up: calibrate on its rest vector */
			for (k = 0; k < IMU_CAL_SAMPLES + 5; k++) {
				(void)imu_feed(&l, &a, 0, TRUE, TRUE);
			}
			printf("calibrated=%d rest (car axes now) fwd %d side %d up %d\n", l.calibrated, a.fwd, a.side, a.up);
			in_log = 1;
			continue;
		}
		if (strncmp(line, "LOG_END", 7) == 0) {
			break;
		}
		if (!in_log || sscanf(line, "%d,%d,%d,%d,%d,%d,%d,%d", &i, &raw[0], &raw[1], &raw[2],
				      &ds, &rh, &mot, &st) != 8) {
			continue;
		}
		to_car(raw, &a);
		dist += ds;
		k = imu_feed(&l, &a, ds, (mot == MOT_STOPPED), (mot == MOT_STOPPED) || (mot == MOT_DRIVING));
		if (k == IMU_EV_HUMP_START) {
			printf("  sample %4d  %4ld mm  HUMP START\n", i, dist / 1000);
		} else if (k == IMU_EV_HUMP_DONE) {
			humps++;
			printf("  sample %4d  %4ld mm  HUMP DONE  height %d mm  (residual %d um, %d mm long)\n",
			       i, dist / 1000, l.height_mm, l.residual_um, l.len_mm);
		} else if (k == IMU_EV_LIFTED) {
			printf("  sample %4d  %4ld mm  LIFTED\n", i, dist / 1000);
		}
		if (trace && (i % 10) == 0) {
			printf("    %4d %4ld mm  lean %6.1f mm\n", i, dist / 1000,
			       (l.tan_roll_q15 * CAR_TRACK_MM * 125 / 4096) / 1000.0);
		}
		n++;
	}
	fclose(f);
	if (imu_finish(&l) == IMU_EV_HUMP_DONE) {
		humps++;
		printf("  (end)  HUMP DONE at the stop  height %d mm\n", l.height_mm);
	}
	printf("%d samples, %ld mm driven: %d hump(s), tallest %d mm, highest lean %d.%d mm\n",
	       n, dist / 1000, humps, l.max_height_mm, l.run_peak_um / 1000, (l.run_peak_um % 1000) / 100);
	return 0;
}
