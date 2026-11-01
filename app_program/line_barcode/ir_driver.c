/** @file ir_driver.c
 *
 * @brief ADC driver and 1 kHz sampler.  See ir_driver.h.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#include <stdint.h>
#include <stdbool.h>
#include "line_barcode.h"
#include "ir_calib.h"
#include "ir_filter.h"
#include "bc_slicer.h"
#include "ir_driver.h"

/* RP2040 ADC registers (datasheet section 4.9). */
#define ADC_BASE                0x4004C000u
#define ADC_CS                  (ADC_BASE + 0x00u)
#define ADC_RESULT              (ADC_BASE + 0x04u)
#define ADC_CS_EN               (1u << 0)
#define ADC_CS_START_ONCE       (1u << 2)
#define ADC_CS_READY            (1u << 8)
#define ADC_CS_AINSEL(n)        ((uint32_t)(n) << 12)
#define ADC_RESULT_MASK         0xFFFu

/* A conversion needs 96 ADC clocks, about 2 us.  If READY has not come
 * after this many polls something is wrong and the driver gives up instead
 * of freezing the system. */
#define ADC_POLL_LIMIT          2000u

#define NUM_CHANNELS            3u

#if (LB_ADC_OVERSAMPLE < 1)
#error "LB_ADC_OVERSAMPLE must be at least 1"
#endif

/* Input pins in channel order. */
static UINT const g_pin[NUM_CHANNELS] =
{
    PIN_IR_LEFT, PIN_IR_CENTER, PIN_IR_RIGHT
};

static ir_filter_t      g_filter[NUM_CHANNELS];
static bc_slicer_t      g_slicer;
static volatile int32_t g_dark[NUM_CHANNELS];

/* Private function prototypes. */
static uint16_t adc_convert(uint32_t channel);
static int32_t  adc_read_dark(uint32_t channel);
static int32_t  read_barcode_dark(void);
static void     send_edge(bc_slicer_edge_t const * p_edge);

void
ir_driver_init (void)
{
    uint32_t chan = 0u;

    for (chan = 0u; chan < NUM_CHANNELS; chan++)
    {
        ir_filter_init(&g_filter[chan]);
        g_dark[chan] = 0;

#if (LB_BC_USE_GPIO_LEVEL != 0)
        if (IR_CHANNEL_RIGHT == chan)
        {
            /* The right (barcode) sensor is wired to its digital output
             * (DO) and is read as a plain GPIO level, not with the ADC. */
            (void)gpio_set_pin(g_pin[chan], GPIO_MODE_IN);
        }
        else
#endif
        {
            /* Analogue line-sensor pin (AO): no digital function, no pull
             * resistors, so the pin follows the sensor voltage. */
            out_w(GPIO_CTRL(g_pin[chan]), GPIO_CTRL_FUNCSEL_NULL);
            clr_w(GPIO(g_pin[chan]), GPIO_IE | GPIO_PUE | GPIO_PDE);
        }
    }

    bc_slicer_init(&g_slicer);

    /* Switch the ADC on and wait until it is ready.  The kernel already
     * released the block from reset (USE_SDEV_DRV). */
    out_w(ADC_CS, ADC_CS_EN);
    (void)adc_convert(0u);

    return;
}

void
ir_sampler_isr (void * p_exinf)
{
    /* Cast: APP_NOW_US() is the kernel's 32-bit unsigned UW. */
    uint32_t         start_us = (uint32_t)APP_NOW_US();
    uint32_t         used_us  = 0u;
    uint32_t         chan     = 0u;
    int32_t          dark_bar = 0;
    bc_slicer_edge_t edge;

    (void)p_exinf;

    g_lb_stats.sampler_runs++;

    /* Line sensors: 10 ms average, published for line_task. */
    for (chan = IR_CHANNEL_LEFT; chan <= IR_CHANNEL_CENTRE; chan++)
    {
        g_dark[chan] = ir_filter_push(&g_filter[chan], adc_read_dark(chan));
    }

    /* Barcode sensor (digital level): same average, then the slicer finds
     * the edges.  The average adds the same delay to every edge, so the
     * element widths that the decoder measures are not changed. */
    dark_bar = ir_filter_push(&g_filter[IR_CHANNEL_RIGHT],
                              read_barcode_dark());
    g_dark[IR_CHANNEL_RIGHT] = dark_bar;

    if (bc_slicer_step(&g_slicer, dark_bar, start_us, &edge))
    {
        send_edge(&edge);
    }
    else
    {
        /* No edge in this millisecond (the normal case). */
    }

    /* Keep the longest run so the tests can check the CPU budget. */
    used_us = (uint32_t)APP_NOW_US() - start_us;
    if (used_us > g_lb_stats.sampler_max_us)
    {
        g_lb_stats.sampler_max_us = used_us;
    }
    else
    {
        /* Not a new maximum. */
    }

    return;
}

int32_t
ir_driver_get_dark (uint32_t channel)
{
    int32_t dark = 0;

    if (channel < NUM_CHANNELS)
    {
        dark = g_dark[channel];
    }
    else
    {
        /* Invalid channel: report 0. */
    }

    return (dark);
}

/*!
 * @brief One ADC conversion.
 *
 * @param[in] channel  ADC input 0 to 4.
 *
 * @return 12-bit result; 0 if the converter never became ready.
 */
static uint16_t
adc_convert (uint32_t channel)
{
    uint32_t polls  = 0u;
    uint16_t result = 0u;

    out_w(ADC_CS, ADC_CS_EN | ADC_CS_AINSEL(channel) | ADC_CS_START_ONCE);

    while ((0u == (in_w(ADC_CS) & ADC_CS_READY)) && (polls < ADC_POLL_LIMIT))
    {
        polls++;
    }

    if (polls < ADC_POLL_LIMIT)
    {
        /* Cast: the value is masked to 12 bits, so it fits uint16_t. */
        result = (uint16_t)(in_w(ADC_RESULT) & ADC_RESULT_MASK);
    }
    else
    {
        /* Converter stuck: return 0 (white on a dark-high sensor). */
    }

    return (result);
}

/*!
 * @brief Read one sensor and convert it to the dark scale.
 *
 * The first conversion after switching the multiplexer is thrown away: the
 * sample-and-hold capacitor still holds the previous channel's voltage.
 * The remaining LB_ADC_OVERSAMPLE conversions are averaged to reduce noise.
 *
 * @param[in] channel  IR_CHANNEL_xxx.
 *
 * @return Dark-scale count, 0 to 4095.
 */
static int32_t
adc_read_dark (uint32_t channel)
{
    int32_t  sum  = 0;
    uint32_t pass = 0u;

    (void)adc_convert(channel);     /* dummy conversion, see above */

    for (pass = 0u; pass < (uint32_t)LB_ADC_OVERSAMPLE; pass++)
    {
        sum += ir_calib_raw_to_dark(adc_convert(channel));
    }

    return (sum / LB_ADC_OVERSAMPLE);
}

/*!
 * @brief Dark-scale value of the barcode sensor.
 *
 * Digital mode (LB_BC_USE_GPIO_LEVEL 1, the wired hardware): the module's DO
 * pin on GP28 is read as a level and mapped to two fixed counts, so the
 * slicer and everything after it work the same way as in analogue mode.
 * Analogue mode (0) is kept as a fallback and reads the ADC instead.
 */
static int32_t
read_barcode_dark (void)
{
    int32_t dark = 0;

#if (LB_BC_USE_GPIO_LEVEL != 0)
    bool b_high = (0u != gpio_get_val(PIN_IR_RIGHT));
    bool b_dark = (b_high == (0 != LB_BC_GPIO_DARK_IS_HIGH));

    dark = b_dark ? LB_BC_GPIO_DARK_COUNT : LB_BC_GPIO_LIGHT_COUNT;
#else
    dark = adc_read_dark(IR_CHANNEL_RIGHT);
#endif

    return (dark);
}

/*!
 * @brief Put one edge on mbf_edge for barcode_task; count it or the loss.
 */
static void
send_edge (bc_slicer_edge_t const * p_edge)
{
    MSG_EDGE msg;

    /* Cast: MSG_EDGE uses the kernel's UW and UB types (app_if.h). */
    msg.t_us = (UW)p_edge->time_us;
    msg.dark = (UB)(p_edge->b_dark ? 1u : 0u);

    if (E_OK == MBF_SEND(mbf_edge, msg))
    {
        g_lb_stats.edges_sent++;
    }
    else
    {
        g_lb_stats.edges_dropped++;     /* buffer full: barcode_task is late */
    }

    return;
}

/*** end of file ***/
