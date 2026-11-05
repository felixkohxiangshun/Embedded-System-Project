# Host (PC) tests of the line and barcode subsystem.  No board, no kernel.
#   cd testing/line_barcode && make run
LB   = ../../app_program/line_barcode
SRC  = $(LB)/bc_decode.c $(LB)/bc_stream.c $(LB)/bc_slicer.c \
       $(LB)/ir_calib.c  $(LB)/ir_filter.c $(LB)/line_ctrl.c \
       $(LB)/line_recov.c lb_sim_bc.c lb_sim_line.c

# Firmware modules are compiled with the strict flags the project uses.
CFLAGS = -std=gnu11 -O1 -Wall -Wextra -Werror -Wconversion -Wshadow \
         -Wundef -Wmissing-prototypes -Wno-sign-conversion \
         -I$(LB) -I.

lb_host_tests: lb_host_tests.c $(SRC)
	$(CC) $(CFLAGS) -o $@ lb_host_tests.c $(SRC) -lm

run: lb_host_tests
	./lb_host_tests
	$(MAKE) digital

clean:
	rm -f lb_host_tests lb_digital_test
.PHONY: run clean

# Digital (DO pin) barcode path: offset and bounce cases.
CFLAGS_D = -std=gnu11 -O1 -Wall -Wextra -Werror -Wconversion -Wshadow \
           -Wundef -I$(LB) -I.

digital: lb_digital_test.c
	$(CC) $(CFLAGS_D) -o lb_digital_test lb_digital_test.c \
	  $(LB)/bc_slicer.c $(LB)/ir_filter.c $(LB)/bc_decode.c -lm
	./lb_digital_test
.PHONY: digital
