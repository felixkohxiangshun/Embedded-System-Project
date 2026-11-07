# Host (PC) unit tests for the pure logic files.  No board, no kernel, no
# Pico SDK: just a C compiler.        cd testing/host && make
APP = ../../app_program
SRC = $(APP)/motor_control/motor_ctrl.c    $(APP)/imu_terrain/imu_logic.c \
      $(APP)/dashboard/dash_logic.c     $(APP)/mission_obstacle/mission_logic.c \
      $(APP)/mission_obstacle/scan_logic.c $(APP)/dashboard/protocol_bits.c

host_tests: host_tests.c $(SRC)
	$(CC) -std=c99 -Wall -Wextra -Werror -Wno-unused-function -Istub -I$(APP) -o $@ host_tests.c $(SRC)

run: host_tests
	./host_tests
	$(MAKE) -C ../line_barcode run

clean:
	rm -f host_tests
.PHONY: run clean
