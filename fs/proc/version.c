// SPDX-License-Identifier: GPL-2.0
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>

#ifdef CONFIG_SPACEX
#include <generated/compile.h>
#endif

#ifdef CONFIG_SPACEX
/* sx-linux-commit */
static int commit_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", SPACEX_COMMIT);
	return 0;
}

/* sx-linux-branch */
static int branch_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", SPACEX_BRANCH);
	return 0;
}

/* sx-linux-build */
static int build_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", SPACEX_BUILD);
	return 0;
}
#endif

/* version */
static int version_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, linux_proc_banner,
		utsname()->sysname,
		utsname()->release,
		utsname()->version);
	return 0;
}

static int __init proc_version_init(void)
{
	proc_create_single("version", 0, NULL, version_proc_show);
#ifdef CONFIG_SPACEX
	proc_create_single("sx-linux-commit", 0, NULL, commit_proc_show);
	proc_create_single("sx-linux-branch", 0, NULL, branch_proc_show);
	proc_create_single("sx-linux-build", 0, NULL, build_proc_show);
#endif
	return 0;
}
fs_initcall(proc_version_init);
