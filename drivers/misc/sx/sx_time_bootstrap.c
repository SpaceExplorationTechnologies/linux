/*
 * This file includes SpaceX-specific functions for bootstrapping the system
 * time without a Real-Time Clock (RTC) chip, using information passed down
 * from the bootloader (specifically, 'cpu-boot-time-unix-ns' and 'utc-offset'
 * in the '/chosen' node of the device tree).
 */

#include <asm/sx_time.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/time64.h>
#include <linux/timekeeping.h>
#include <linux/timex.h>

/**
 * sx_time_bootstrap() - Sets the time precisely.
 *
 * This function uses the information in our enhanced rdate protocol.
 * U-Boot provides this data via the device tree, as
 * /chosen/cpu-boot-time-unix-ns.
 *
 * This also initializes the UTC-to-TAI offset, via /chosen/utc-offset.
 */
static int __init sx_time_bootstrap(void)
{
	int rc;
	u64 cpu_boot_time_unix_ns;
	u64 ns_since_cpu_boot;
	u64 current_unix_time_ns;
	u64 current_time_seconds_portion;
	u32 current_time_ns_portion;
	struct timespec64 ts;
	u32 utc_offset;

	if (of_chosen == NULL) {
		pr_warn("No '/chosen' node found in device tree!  "
			"Not setting time...\n");
		return 0;
	}

	rc = of_property_read_u64(of_chosen, "cpu-boot-time-unix-ns",
				&cpu_boot_time_unix_ns);
	if ((rc != 0) || (cpu_boot_time_unix_ns == 0)) {
		pr_info("No cpu-boot-time-unix-ns specified -- "
			"not setting time.\n");
		return 0;
	}

	ns_since_cpu_boot = sx_get_ns_since_cpu_boot();
	current_unix_time_ns = cpu_boot_time_unix_ns + ns_since_cpu_boot;
	current_time_seconds_portion = div_u64_rem(current_unix_time_ns,
						NSEC_PER_SEC,
						&current_time_ns_portion);
	ts.tv_sec = current_time_seconds_portion;
	ts.tv_nsec = current_time_ns_portion;

	rc = do_settimeofday64(&ts);
	/*
	 * Error is not fatal; just inform and move on.
	 */
	if (rc != 0) {
		pr_warn("failed to set system clock to %lld.%09ld\n",
			(long long) ts.tv_sec, ts.tv_nsec);
	} else {
		pr_info("system clock set to %lld.%09ld\n",
			(long long) ts.tv_sec, ts.tv_nsec);
	}

	if (of_property_read_u32(of_chosen,
				 "utc-offset", &utc_offset) != 0) {
		pr_warn("No utc-offset provided -- not setting UTC-to-TAI "
			"offset...\n");
	} else {
		/*
		 * Using the public do_adjtimex() API instead of the more
		 * convenient (but hidden) timekeeping_set_tai_offset().
		 */
		struct __kernel_timex timex = {
			.modes = ADJ_TAI,
			.constant = utc_offset
		};
		/*
		 * Failing to set the UTC offset is not a fatal error --
		 * just report it and move on.
		 * Note that this will return TIME_ERROR at this point, because
		 * time is not yet deemed "synchronized." So we ignore the
		 * return value and check the returned UTC-to-TAI offset.
		 */
		(void) do_adjtimex(&timex);
		if (timex.tai != utc_offset)
			pr_warn("Failed to set UTC offset to %lu -- "
				"'tai' reported as %d\n",
				(long unsigned) utc_offset, timex.tai);
	}

	return 0;
}

/*
 * We want this to happen after the architecture code has set up the clocks,
 * but before PTP device drivers come around.
 */
arch_initcall_sync(sx_time_bootstrap);
