/*
 * This file includes architecture-specific support for SpaceX-specific
 * time bootstrapping (see sx_time_bootstrap.c).
 *
 * In order for bootstrapping to work, the bootloader must estimate the
 * time that a counter started incrementing at constant rate (called
 * '/chosen/cpu-boot-time-unix-ns' in the device tree).  That counter
 * must continue incrementing at constant rate (i.e., not reset or roll over)
 * up until the point that this code is called.  The code in this file then
 * calculates the time elapsed based on the current value of the counter and
 * the increment rate. sx_time_bootstrap then adds this to the value from
 * cpu-boot-time-unix-ns to arrive at the current time.
 *
 * For ARM64, we use the timebase counter returned by arch_counter_get_cntpct()
 * for this purpose as it is not reset during boot.
 */

#ifndef __ARM64_SX_TIME_H
#define __ARM64_SX_TIME_H

#include <asm/arch_timer.h>
#include <linux/math64.h>
#include <linux/time64.h>

/**
 * sx_get_ns_since_cpu_boot() - Returns nanoseconds since CPU bootup.
 *
 * This function is called from sx_time_bootstrap(); it must use the same
 * counters used in U-Boot to calculate the cpu-boot-time-unix-ns.
 *
 * Note the order of calculation here, so that we avoid integer division
 * errors; normally one would calculate
 * (NSEC_PER_SEC / arch_timer_get_cntfrq) * arch_counter_get_cntpct()
 * i.e., ns_per_tick * ticks_since_boot.
 * But if arch_timer_get_cntfrq is 66666667, then
 * (NSEC_PER_SEC / arch_timer_get_cntfrq) becomes 14ns (incorrect) instead of
 * 15ns (correct) per clock tick.
 *
 * Unfortunately, we don't get a lot of uptime before the
 * multiplication (NSEC_PER_SEC * get_tb()) overflows 64 bits.
 * For a system clock of 66MHz, that comes out to:
 * 2**64 / 10**9 / 66666667 = 276 seconds, or just over 4.5 minutes.
 * To avoid this, we pre-divide NSEC_PER_SEC by a factor of 1000
 * (and multiply that back in at the end). This means we only have
 * microsecond resolution in the ns_since_boot, but we don't expect to
 * be more accurate than that anyway (due to OS scheduling and
 * networking delays). For a 1GHz system clock, this gives us over 5
 * hours of uptime before the calculation rolls over.
 */

static inline u64 sx_get_ns_since_cpu_boot(void)
{
	return div_u64((NSEC_PER_SEC / 1000) * __arch_counter_get_cntpct_stable(),
		       arch_timer_get_cntfrq()) * 1000;
}

#endif /* __ARM64_SX_TIME_H */
