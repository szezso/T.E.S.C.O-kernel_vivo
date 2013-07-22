/*
 * ElementalX msm-sleeper by flar2 <asegaert@gmail.com>
 * 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/earlysuspend.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/cpufreq.h>

#define MSM_SLEEPER_MAJOR_VERSION	1
#define MSM_SLEEPER_MINOR_VERSION	1

extern uint32_t maxscroff;
extern uint32_t maxscroff_freq;
uint32_t old_max = 0;


#ifdef CONFIG_HAS_EARLYSUSPEND
static void __cpuinit msm_sleeper_early_suspend(struct early_suspend *h)
{
	struct cpufreq_policy *policy;

	if (maxscroff) {
		policy = cpufreq_cpu_get(0);
		old_max = policy->max;
		policy->max = maxscroff_freq;
		printk(KERN_INFO "[msm-sleeper]: Limited freq to '%u'\n", maxscroff_freq);
	}
}

static void __cpuinit msm_sleeper_late_resume(struct early_suspend *h)
{
	struct cpufreq_policy *policy;
	if (maxscroff) {
		policy = cpufreq_cpu_get(0);
		policy->max = old_max;
		printk(KERN_INFO "[msm-sleeper]: Restoring freq to '%u'\n", old_max);
	}
}

static struct early_suspend msm_sleeper_early_suspend_driver = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 10,
	.suspend = msm_sleeper_early_suspend,
	.resume = msm_sleeper_late_resume,
};
#endif

static int __init msm_sleeper_init(void)
{
	pr_info("msm-sleeper version %d.%d\n",
		 MSM_SLEEPER_MAJOR_VERSION,
		 MSM_SLEEPER_MINOR_VERSION);

#ifdef CONFIG_HAS_EARLYSUSPEND
		register_early_suspend(&msm_sleeper_early_suspend_driver);
#endif
	return 0;
}

MODULE_AUTHOR("flar2 <asegaert at gmail.com>");
MODULE_DESCRIPTION("'msm-sleeper' - Limit max frequency while screen is off");
MODULE_LICENSE("GPL");

late_initcall(msm_sleeper_init);

