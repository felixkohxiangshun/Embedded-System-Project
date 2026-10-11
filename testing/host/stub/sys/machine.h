/* Host stub: register access used by the inline helper in app.h, never run. */
#ifndef HOST_STUB_MACHINE_H
#define HOST_STUB_MACHINE_H
static volatile unsigned host_reg;
#define RESETS_RESET		0
#define RESETS_RESET_DONE	0
#define RESETS_RESET_PWM	1
#define TIMER_TIMELR		0
#define clr_w(a, b)		((void)(host_reg &= ~(unsigned)(b)))
#define in_w(a)			(host_reg)
#endif
