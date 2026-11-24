/** @file bc_decode.c
 *
 * @brief Code 39 decoder for "*A*" to "*D*".  See bc_decode.h.
 *
 * @par
 * Reference: Code 39 character set (ANSI/AIM BC1-1995 / ISO/IEC 16388).  The
 * element patterns used below are written as N (narrow) and W (wide), bar
 * first.  Group 11 barcodes are "*A*" to "*D*" (Barcode & Line
 * Specification); the other letters of the table exist only so that a
 * wrong sample code (F, W, X, Y, Z) is recognised and then ignored.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Group 11. Coding standard: BARR-C:2018.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "lb_config.h"
#include "bc_decode.h"

#define ELEMENTS_PER_CHAR       9u      /* 5 bars and 4 spaces */
#define BARS_PER_CHAR           5u
#define SPACES_PER_CHAR         4u
#define NARROW_SPACES_PER_CHAR  3u      /* 4 spaces, 1 of them wide */
#define NUM_CHARS               3u      /* '*', command, '*' */
#define CHAR_STRIDE             10u     /* 9 elements + 1 gap */
#define FIRST_GAP_INDEX         9u
#define SECOND_GAP_INDEX        19u
#define RATIO_DIVISOR           10u     /* ratios are given in tenths */
#define MAX_ELEMENTS            (LB_BC_MAX_EDGES - 1u)

/* Value returned by pattern_lookup() for a pattern that is not in the table. */
#define SYMBOL_UNKNOWN          0u

/* Private types. */

/* What classify_character() learns about one character. */
typedef struct
{
    uint32_t wide_mask;         /* bit n set: element n (0 to 8) is wide */
    uint32_t narrow_space_us;   /* average width of its narrow spaces */
    uint32_t total_us;          /* time to read all nine elements */
} char_info_t;

/* One entry of the pattern table. */
typedef struct
{
    uint32_t wide_mask;         /* bit n set: element n is wide */
    uint8_t  symbol;            /* the character */
} pattern_t;

/* Wide elements of every character the decoder knows, element 0 = first
 * bar.  The mask is checked against the Code 39 table of the reportlab
 * library (see testing/line_barcode/gen_patterns.py).  '*' frames the code
 * and every letter A to Z is a valid command: which movement each letter
 * means is sent to the car over MQTT (TEAM.md section 7).  Every letter
 * has exactly two wide bars and one wide space. */
static pattern_t const g_pattern[] =
{
    { 0x052u, '*' },   /* N W N N W N W N N */
    { 0x121u, 'A' },   /* W N N N N W N N W */
    { 0x124u, 'B' },   /* N N W N N W N N W */
    { 0x025u, 'C' },   /* W N W N N W N N N */
    { 0x130u, 'D' },   /* N N N N W W N N W */
    { 0x031u, 'E' },   /* W N N N W W N N N */
    { 0x034u, 'F' },   /* N N W N W W N N N */
    { 0x160u, 'G' },   /* N N N N N W W N W */
    { 0x061u, 'H' },   /* W N N N N W W N N */
    { 0x064u, 'I' },   /* N N W N N W W N N */
    { 0x070u, 'J' },   /* N N N N W W W N N */
    { 0x181u, 'K' },   /* W N N N N N N W W */
    { 0x184u, 'L' },   /* N N W N N N N W W */
    { 0x085u, 'M' },   /* W N W N N N N W N */
    { 0x190u, 'N' },   /* N N N N W N N W W */
    { 0x091u, 'O' },   /* W N N N W N N W N */
    { 0x094u, 'P' },   /* N N W N W N N W N */
    { 0x1C0u, 'Q' },   /* N N N N N N W W W */
    { 0x0C1u, 'R' },   /* W N N N N N W W N */
    { 0x0C4u, 'S' },   /* N N W N N N W W N */
    { 0x0D0u, 'T' },   /* N N N N W N W W N */
    { 0x103u, 'U' },   /* W W N N N N N N W */
    { 0x106u, 'V' },   /* N W W N N N N N W */
    { 0x007u, 'W' },   /* W W W N N N N N N */
    { 0x112u, 'X' },   /* N W N N W N N N W */
    { 0x013u, 'Y' },   /* W W N N W N N N N */
    { 0x016u, 'Z' }    /* N W W N W N N N N */
};

#define NUM_PATTERNS    (sizeof(g_pattern) / sizeof(g_pattern[0]))

/* Private function prototypes. */
static void sort_descending(uint32_t * p_value, uint32_t count);
static void sort_ascending(uint32_t * p_value, uint32_t count);
static bool is_wider_by_ratio(uint32_t wide, uint32_t narrow,
                              uint32_t ratio_x10);
static bool classify_character(uint32_t const * p_width,
                               char_info_t * p_info);
static uint8_t pattern_lookup(uint32_t wide_mask);
static bool gap_is_valid(uint32_t gap_us, uint32_t space_a, uint32_t space_b);
static bool speed_is_consistent(char_info_t const * p_info);
static bc_decode_status_t decode_elements(uint32_t const * p_width,
                                          uint8_t * p_command);
static uint32_t median_width(uint32_t const * p_edge_us, uint32_t num_edges);
static bool find_glitch(uint32_t const * p_edge_us, uint32_t num_edges,
                        uint32_t limit_us, uint32_t * p_index);
static uint32_t status_rank(bc_decode_status_t status);

bc_decode_status_t
bc_decode_edges (uint32_t const * p_edge_us,
                 uint32_t num_edges,
                 uint8_t * p_command,
                 bool * p_b_reversed)
{
    bc_decode_status_t status                      = BC_DECODE_OK;
    bc_decode_status_t reverse_status              = BC_DECODE_OK;
    uint32_t           width[BC_DECODE_NUM_ELEMENTS] = { 0u };
    uint32_t           reversed[BC_DECODE_NUM_ELEMENTS] = { 0u };
    uint32_t           idx                         = 0u;

    if ((NULL == p_edge_us) || (NULL == p_command) || (NULL == p_b_reversed))
    {
        status = BC_DECODE_ERR_NULL;
    }
    else if (BC_DECODE_NUM_EDGES != num_edges)
    {
        status = BC_DECODE_ERR_COUNT;
    }
    else
    {
        *p_b_reversed = false;

        /* Element width = difference of two edge times.  Unsigned
         * subtraction gives the right answer even if the 32-bit microsecond
         * timer wrapped between the two edges. */
        for (idx = 0u; idx < BC_DECODE_NUM_ELEMENTS; idx++)
        {
            width[idx]    = p_edge_us[idx + 1u] - p_edge_us[idx];
            reversed[(BC_DECODE_NUM_ELEMENTS - 1u) - idx] = width[idx];
        }

        status = decode_elements(width, p_command);

        if (BC_DECODE_OK != status)
        {
            /* The car may have driven over the code backwards.  Reading the
             * elements in reverse order then gives a normal code. */
            reverse_status = decode_elements(reversed, p_command);

            if (BC_DECODE_OK == reverse_status)
            {
                status        = BC_DECODE_OK;
                *p_b_reversed = true;
            }
            else
            {
                /* Neither direction decodes: report the forward reason. */
            }
        }
        else
        {
            /* Forward reading decoded. */
        }
    }

    return (status);
}

bc_decode_status_t
bc_decode_scan (uint32_t const * p_edge_us,
                uint32_t num_edges,
                uint8_t * p_command,
                bool * p_b_reversed)
{
    bc_decode_status_t status      = BC_DECODE_ERR_COUNT;
    bc_decode_status_t try_status  = BC_DECODE_OK;
    uint32_t           offset      = 0u;
    bool               b_found     = false;

    if ((NULL == p_edge_us) || (NULL == p_command) || (NULL == p_b_reversed))
    {
        status = BC_DECODE_ERR_NULL;
    }
    else
    {
        /* A code starts with a dark bar, and the edges alternate, so only
         * every second edge can be the first edge of a code.  Stop at the
         * first window that decodes. */
        for (offset = 0u;
             ((offset + BC_DECODE_NUM_EDGES) <= num_edges) && (!b_found);
             offset += 2u)
        {
            try_status = bc_decode_edges(&p_edge_us[offset],
                                         BC_DECODE_NUM_EDGES,
                                         p_command,
                                         p_b_reversed);

            if (BC_DECODE_OK == try_status)
            {
                status  = BC_DECODE_OK;
                b_found = true;
            }
            else if (status_rank(try_status) > status_rank(status))
            {
                /* Remember the failure that came closest to a decode. */
                status = try_status;
            }
            else
            {
                /* Not closer than what is already known. */
            }
        }
    }

    return (status);
}

uint32_t
bc_decode_merge_glitches (uint32_t * p_edge_us, uint32_t num_edges)
{
    uint32_t count    = num_edges;
    uint32_t median   = 0u;
    uint64_t limit_us = 0u;
    uint32_t index    = 0u;
    uint32_t move     = 0u;

    if ((NULL != p_edge_us) && (num_edges >= 3u)
        && (num_edges <= LB_BC_MAX_EDGES))
    {
        median = median_width(p_edge_us, count);

        /* Cast: widening to 64 bits so the product cannot overflow. */
        limit_us = ((uint64_t)median * LB_BC_GLITCH_X10) / RATIO_DIVISOR;

        /* Delete the two edges of every glitch.  Every pass removes two
         * edges, so the loop ends after at most num_edges / 2 passes. */
        while ((count >= 3u)
               && find_glitch(p_edge_us, count, (uint32_t)limit_us, &index))
        {
            for (move = index; (move + 2u) < count; move++)
            {
                p_edge_us[move] = p_edge_us[move + 2u];
            }

            count -= 2u;
        }
    }
    else
    {
        /* Bad arguments or too few edges: nothing to merge. */
    }

    return (count);
}

/*!
 * @brief Sort values from the largest to the smallest (insertion sort).
 *
 * Used on 4 or 5 values, so the simple algorithm is also the fastest.
 */
static void
sort_descending (uint32_t * p_value, uint32_t count)
{
    uint32_t outer = 0u;
    uint32_t inner = 0u;
    uint32_t held  = 0u;

    for (outer = 1u; outer < count; outer++)
    {
        held  = p_value[outer];
        inner = outer;

        while ((inner > 0u) && (p_value[inner - 1u] < held))
        {
            p_value[inner] = p_value[inner - 1u];
            inner--;
        }

        p_value[inner] = held;
    }

    return;
}

/*!
 * @brief Sort values from the smallest to the largest (insertion sort).
 */
static void
sort_ascending (uint32_t * p_value, uint32_t count)
{
    uint32_t outer = 0u;
    uint32_t inner = 0u;
    uint32_t held  = 0u;

    for (outer = 1u; outer < count; outer++)
    {
        held  = p_value[outer];
        inner = outer;

        while ((inner > 0u) && (p_value[inner - 1u] > held))
        {
            p_value[inner] = p_value[inner - 1u];
            inner--;
        }

        p_value[inner] = held;
    }

    return;
}

/*!
 * @brief Is wide at least ratio_x10 / 10 times narrow?  False if narrow is 0.
 */
static bool
is_wider_by_ratio (uint32_t wide, uint32_t narrow, uint32_t ratio_x10)
{
    /* Cast: widening to 64 bits so the products cannot overflow even for a
     * corrupt width close to 2^32. */
    return ((narrow > 0u)
            && (((uint64_t)wide * RATIO_DIVISOR)
                >= ((uint64_t)narrow * ratio_x10)));
}

/*!
 * @brief Decide which of the nine elements of one character are wide.
 *
 * Bars are the elements 0, 2, 4, 6, 8 and spaces are 1, 3, 5, 7.  Every
 * supported character has exactly two wide bars and one wide space, so the
 * two widest bars and the widest space are the wide ones.  The decision is
 * accepted only if the narrowest wide element is clearly wider than the
 * widest narrow element of the same kind.
 *
 * @param[in]  p_width  The nine element widths in microseconds.
 * @param[out] p_info   Wide mask and average narrow widths.
 *
 * @return true if the character is clear, false if it is ambiguous.
 */
static bool
classify_character (uint32_t const * p_width, char_info_t * p_info)
{
    uint32_t bar[BARS_PER_CHAR]     = { 0u };
    uint32_t space[SPACES_PER_CHAR] = { 0u };
    uint32_t bar_sum                = 0u;
    uint32_t space_sum              = 0u;
    uint32_t idx                    = 0u;
    bool     b_clear                = false;

    for (idx = 0u; idx < BARS_PER_CHAR; idx++)
    {
        bar[idx] = p_width[2u * idx];
        bar_sum += bar[idx];
    }

    for (idx = 0u; idx < SPACES_PER_CHAR; idx++)
    {
        space[idx] = p_width[(2u * idx) + 1u];
        space_sum += space[idx];
    }

    /* After sorting, bar[0] and bar[1] are the wide bars, bar[2] is the
     * widest narrow bar; space[0] is the wide space, space[1] the widest
     * narrow space. */
    sort_descending(bar, BARS_PER_CHAR);
    sort_descending(space, SPACES_PER_CHAR);

    b_clear = is_wider_by_ratio(bar[1], bar[2], LB_BC_WIDE_RATIO_X10)
              && is_wider_by_ratio(space[0], space[1], LB_BC_WIDE_RATIO_X10);

    if (b_clear)
    {
        p_info->wide_mask = 0u;

        for (idx = 0u; idx < ELEMENTS_PER_CHAR; idx++)
        {
            /* Even index: a bar.  Odd index: a space. */
            if (0u == (idx % 2u))
            {
                if (p_width[idx] >= bar[1])
                {
                    p_info->wide_mask |= (1u << idx);
                }
            }
            else if (p_width[idx] >= space[0])
            {
                p_info->wide_mask |= (1u << idx);
            }
            else
            {
                /* Narrow space: bit stays 0. */
            }
        }

        p_info->narrow_space_us = (space_sum - space[0])
                                  / NARROW_SPACES_PER_CHAR;
        p_info->total_us        = bar_sum + space_sum;
    }
    else
    {
        /* Ambiguous widths: leave p_info alone, the caller rejects. */
    }

    return (b_clear);
}

/*!
 * @brief Find the character that belongs to a wide-element mask.
 *
 * @return The character, or SYMBOL_UNKNOWN if the mask is not in the table.
 */
static uint8_t
pattern_lookup (uint32_t wide_mask)
{
    /* Casts: SYMBOL_UNKNOWN is 0 and NUM_PATTERNS is a small positive count,
     * so both fit the target types. */
    uint8_t  symbol = (uint8_t)SYMBOL_UNKNOWN;
    uint32_t idx    = 0u;

    for (idx = 0u; idx < (uint32_t)NUM_PATTERNS; idx++)
    {
        if (wide_mask == g_pattern[idx].wide_mask)
        {
            symbol = g_pattern[idx].symbol;
        }
        else
        {
            /* Not this entry. */
        }
    }

    return (symbol);
}

/*!
 * @brief Is the gap between two characters about one narrow space wide?
 *
 * The reference is the mean of the narrow spaces of the two neighbours.  The
 * gap must be between LB_BC_GAP_MIN_X10 / 10 and LB_BC_GAP_MAX_X10 / 10 times
 * that reference.
 */
static bool
gap_is_valid (uint32_t gap_us, uint32_t space_a, uint32_t space_b)
{
    uint32_t reference = (space_a / 2u) + (space_b / 2u);
    uint64_t gap_x10   = 0u;
    bool     b_valid   = false;

    if (reference > 0u)
    {
        /* Cast: widening to 64 bits so the products cannot overflow. */
        gap_x10 = (uint64_t)gap_us * RATIO_DIVISOR;
        b_valid = (gap_x10 >= ((uint64_t)reference * LB_BC_GAP_MIN_X10))
                  && (gap_x10 <= ((uint64_t)reference * LB_BC_GAP_MAX_X10));
    }
    else
    {
        /* No usable narrow space to compare with: reject. */
    }

    return (b_valid);
}

/*!
 * @brief Were the three characters read at a similar speed?
 *
 * The character read slowest must not take more than
 * LB_BC_SPEED_RATIO_X10 / 10 times as long as the one read fastest.  Every
 * character is printed the same width, so its total time follows the speed
 * only.  (Comparing narrow bars failed on the car: the digital sensor makes
 * a narrow bar look 0.1 to 1 x its real width, which looked like a speed
 * change.)
 */
static bool
speed_is_consistent (char_info_t const * p_info)
{
    uint32_t fastest = p_info[0].total_us;
    uint32_t slowest = p_info[0].total_us;
    uint32_t idx     = 0u;

    for (idx = 1u; idx < NUM_CHARS; idx++)
    {
        if (p_info[idx].total_us < fastest)
        {
            fastest = p_info[idx].total_us;
        }
        else
        {
            /* Not the smallest so far. */
        }

        if (p_info[idx].total_us > slowest)
        {
            slowest = p_info[idx].total_us;
        }
        else
        {
            /* Not the largest so far. */
        }
    }

    /* Cast: widening to 64 bits so the products cannot overflow. */
    return ((fastest > 0u)
            && (((uint64_t)slowest * RATIO_DIVISOR)
                <= ((uint64_t)fastest * LB_BC_SPEED_RATIO_X10)));
}

/*!
 * @brief Decode 29 element widths, read forward, as "*X*".
 *
 * Checks, in this order: every character is clear (WIDTH), the first and the
 * last character are '*' (FRAME), the gaps are right (GAP), the speed is
 * steady (SPEED), the middle character is a letter A to Z (COMMAND).
 */
static bc_decode_status_t
decode_elements (uint32_t const * p_width, uint8_t * p_command)
{
    bc_decode_status_t status = BC_DECODE_OK;
    char_info_t        info[NUM_CHARS]   = { { 0u, 0u, 0u } };
    uint8_t            symbol[NUM_CHARS] = { 0u };
    uint32_t           idx               = 0u;

    for (idx = 0u; (idx < NUM_CHARS) && (BC_DECODE_OK == status); idx++)
    {
        if (classify_character(&p_width[idx * CHAR_STRIDE], &info[idx]))
        {
            symbol[idx] = pattern_lookup(info[idx].wide_mask);
        }
        else
        {
            status = BC_DECODE_ERR_WIDTH;
        }
    }

    if (BC_DECODE_OK == status)
    {
        if (('*' != symbol[0]) || ('*' != symbol[2]))
        {
            status = BC_DECODE_ERR_FRAME;
        }
        else if (!gap_is_valid(p_width[FIRST_GAP_INDEX],
                               info[0].narrow_space_us,
                               info[1].narrow_space_us)
                 || !gap_is_valid(p_width[SECOND_GAP_INDEX],
                                  info[1].narrow_space_us,
                                  info[2].narrow_space_us))
        {
            status = BC_DECODE_ERR_GAP;
        }
        else if (!speed_is_consistent(info))
        {
            status = BC_DECODE_ERR_SPEED;
        }
        else if ((symbol[1] < 'A') || (symbol[1] > 'Z'))
        {
            /* A well formed code, but not a letter ('*' in the middle).
             * Report which character it was (0 if it is not in the
             * table). */
            *p_command = symbol[1];
            status     = BC_DECODE_ERR_COMMAND;
        }
        else
        {
            *p_command = symbol[1];
        }
    }
    else
    {
        /* A character was ambiguous: status already says so. */
    }

    return (status);
}

/*!
 * @brief Median of the element widths of a list of edges.
 */
static uint32_t
median_width (uint32_t const * p_edge_us, uint32_t num_edges)
{
    uint32_t width[MAX_ELEMENTS] = { 0u };
    uint32_t num_elements        = num_edges - 1u;
    uint32_t idx                 = 0u;

    for (idx = 0u; idx < num_elements; idx++)
    {
        width[idx] = p_edge_us[idx + 1u] - p_edge_us[idx];
    }

    sort_ascending(width, num_elements);

    return (width[num_elements / 2u]);
}

/*!
 * @brief Find the first element shorter than limit_us.
 *
 * @param[out] p_index  Index of its first edge when the function returns true.
 *
 * @return true if a glitch was found.
 */
static bool
find_glitch (uint32_t const * p_edge_us,
             uint32_t num_edges,
             uint32_t limit_us,
             uint32_t * p_index)
{
    bool     b_found = false;
    uint32_t idx     = 0u;

    for (idx = 0u; (idx + 1u) < num_edges; idx++)
    {
        if ((!b_found) && ((p_edge_us[idx + 1u] - p_edge_us[idx]) < limit_us))
        {
            b_found  = true;
            *p_index = idx;
        }
        else
        {
            /* Long enough, or a glitch was already found. */
        }
    }

    return (b_found);
}

/*!
 * @brief How close to a successful decode is a failure?  Higher is closer.
 *
 * Used by bc_decode_scan() to report the most informative reason when no
 * window decodes.
 */
static uint32_t
status_rank (bc_decode_status_t status)
{
    uint32_t rank = 0u;

    switch (status)
    {
        case BC_DECODE_ERR_COMMAND:
            rank = 6u;
            break;

        case BC_DECODE_ERR_SPEED:
            rank = 5u;
            break;

        case BC_DECODE_ERR_GAP:
            rank = 4u;
            break;

        case BC_DECODE_ERR_FRAME:
            rank = 3u;
            break;

        case BC_DECODE_ERR_WIDTH:
            rank = 2u;
            break;

        case BC_DECODE_ERR_COUNT:
            rank = 1u;
            break;

        default:
            rank = 0u;
            break;
    }

    return (rank);
}

/*** end of file ***/
