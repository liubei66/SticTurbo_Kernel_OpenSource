/*
 * CPUFreq governor based on scheduler-provided CPU utilization data.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 * Copyright (C) 2020 Amktiao.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * 
 * Welcome to use the copy governor "realmeutil" from schedutil
 * Based on CAF 4.14 transplantation, more power-saving governor
 * Has close to excellent battery life and good performance.
 * It is currently the power saving mode governor on Stic.
 * 
 * Update time: 2020-10-14 02:37:54 CST
 * Author: Unknown
 *
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/cpufreq.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <trace/events/power.h>
#include <linux/sched/sysctl.h>
#ifdef CONFIG_STICKERNEL
#include <linux/stickernelcache.h>
#endif
#include "sched.h"
#include "tune.h"
#ifdef CONFIG_STICKERNEL_GAMETURBO
#include <linux/sticcpufreq.h>
#include <linux/stickernel.h>
#endif
#ifdef CONFIG_STICKERNEL_SUSPEND
#include <linux/suspend.h>
#include <linux/stickill.h>
#endif
#ifdef CONFIG_STICKERNEL_TURBO
#include <linux/sticcpuboost>
#endif
/* Stub out fast switch routines present on mainline to reduce the backport */
#define cpufreq_driver_fast_switch(x, y) 0
#define cpufreq_enable_fast_switch(x)
#define cpufreq_disable_fast_switch(x)
#define LATENCY_MULTIPLIER			(1000)
#define SUGOV_KTHREAD_PRIORITY	50
#ifdef CONFIG_STICKERNEL_GAMETURBO
#define SCF_BOOST_MS	250
#define CPUFREQ_UP_GO_MS	2000
#define CPUFREQ_DOWN_GO_MS	150
#define CPUFREQ_CORE_CTL_A	300
#define CPUFREQ_CORE_CTL_B	500
#define CPUFREQ_APP_MS	120
#endif
#ifdef CONFIG_STICKERNEL_SUSPEND
/* Processor sleep frequency */
#define CPUFREQ_SUSPEND_A 1324800
#define CPUFREQ_SUSPEND_B 902400
#endif
#ifdef CONFIG_STICKERNEL_TURBO
#define BIG_CPUAAC 800
#define LITE_CPUAAC 400
#endif
struct sugov_tunables {
	struct gov_attr_set attr_set;
	unsigned int up_rate_limit_us;
	unsigned int down_rate_limit_us;
	unsigned int hispeed_load;
	unsigned int hispeed_freq;
	bool pl;
	bool iowait_boost_enable;
};
struct sugov_policy {
	struct cpufreq_policy *policy;
	struct sugov_tunables *tunables;
	struct list_head tunables_hook;
	raw_spinlock_t update_lock;
	u64 last_freq_update_time;
	s64 min_rate_limit_ns;
	s64 up_rate_delay_ns;
	s64 down_rate_delay_ns;
	u64 last_ws;
	u64 curr_cycles;
	u64 last_cyc_update_time;
	unsigned long avg_cap;
	unsigned int next_freq;
	unsigned int cached_raw_freq;
	unsigned long hispeed_util;
	unsigned long max;
#ifdef CONFIG_STICKERNEL
	unsigned int stic_freq;
#endif
	/* The next fields are only needed if fast switch cannot be used. */
	struct irq_work irq_work;
	struct kthread_work work;
	struct mutex work_lock;
	struct kthread_worker worker;
	struct task_struct *thread;
	bool work_in_progress;
	bool need_freq_update;
};
struct sugov_cpu {
	struct update_util_data update_util;
	struct sugov_policy *sg_policy;
	unsigned long iowait_boost;
	unsigned long iowait_boost_max;
	u64 last_update;
	struct sched_walt_cpu_load walt_load;
	/* The fields below are only needed when sharing a policy. */
	unsigned long util;
	unsigned long max;
	unsigned int flags;
	unsigned int cpu;
	/* The field below is for single-CPU policies only. */
#ifdef CONFIG_NO_HZ_COMMON
	unsigned long saved_idle_calls;
#endif
};
static DEFINE_PER_CPU(struct sugov_cpu, sugov_cpu);
static unsigned int stale_ns;
static DEFINE_PER_CPU(struct sugov_tunables *, cached_tunables);
#ifdef CONFIG_STICKERNEL_PELT
bool use_pred_load(int cpu)
{
	return get_freq_reporting_policy(cpu);
}
#endif
/************************ Governor internals ***********************/
#ifdef CONFIG_STICKERNEL
static unsigned int freq_to_targetload(struct sugov_tunables *tunables,
				       unsigned int freq)
{
	unsigned long flags;
	unsigned int ret;
	int i;
	spin_lock_irqsave(&tunables->target_loads_lock, flags);
	ret = tunables->target_loads[i];
	return ret;
}
static unsigned int freq_to_above_hispeed_delay(struct sugov_tunables *tunables,
				       unsigned int freq)
{
	unsigned long flags;
	unsigned int ret;
	int i;
	ret = tunables->above_hispeed_delay[i];
	spin_unlock_irqrestore(&tunables->above_hispeed_delay_lock, flags);
	return ret;
}
static unsigned int choose_freq(struct sugov_policy *sg_policy,
				unsigned int loadadjfreq)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	struct cpufreq_frequency_table *freq_table = policy->freq_table;
	unsigned int prevfreq, freqmin = 0, freqmax = UINT_MAX, tl;
	unsigned int freq = policy->cur;
	int index;
	do {
		prevfreq = freq;
		tl = freq_to_targetload(sg_policy->tunables, freq);
static unsigned int eval_target_freq(struct sugov_policy *sg_policy,
				     unsigned long util, unsigned long max)
{
	u64 now;
	int cpu_load = 0;
	unsigned int new_freq;
	struct sugov_tunables *tunables = sg_policy->tunables;
	struct cpufreq_policy *policy = sg_policy->policy;
	now = ktime_to_us(ktime_get());
	tunables->boosted = tunables->boost ||
			    now < tunables->boostpulse_endtime;
	if (tunables->boosted && policy->cur < tunables->hispeed_freq) {
		new_freq = tunables->hispeed_freq;
	} else {
		cpu_load = util * 80 / capacity_curr_of(policy->cpu);
		new_freq = choose_freq(sg_policy, cpu_load * policy->cur);
		if ((cpu_load >= tunables->go_hispeed_load || tunables->boosted) &&
		    new_freq < tunables->hispeed_freq)
			new_freq = tunables->hispeed_freq;
	}
	new_freq = max(sg_policy->iowait_boost, new_freq);
	return new_freq;
}
static void sugov_slack_timer_resched(struct sugov_policy *sg_policy)
{
	u64 expires;
	raw_spin_lock(&sg_policy->timer_lock);
	if (!sg_policy->governor_enabled)
		goto unlock;
	del_timer(&sg_policy->pol_slack_timer);
	if (sg_policy->tunables->timer_slack_val >= 0 &&
	    sg_policy->next_freq > sg_policy->policy->min) {
		expires = jiffies + usecs_to_jiffies(sg_policy->tunables->timer_slack_val);
		sg_policy->pol_slack_timer.expires = expires;
		add_timer_on(&sg_policy->pol_slack_timer, sg_policy->trigger_cpu);
	}
unlock:
	raw_spin_unlock(&sg_policy->timer_lock);
}
#endif
static bool sugov_should_update_freq(struct sugov_policy *sg_policy, u64 time)
{
	s64 delta_ns;
	if (unlikely(sg_policy->need_freq_update)) {
		sg_policy->need_freq_update = false;
		/*
		 * This happens when limits change, so forget the previous
		 * next_freq value and force an update.
		 */
		sg_policy->next_freq = UINT_MAX;
		return true;
	}
	delta_ns = time - sg_policy->last_freq_update_time;
	/* No need to recalculate next freq for min_rate_limit_us at least */
	return delta_ns >= sg_policy->min_rate_limit_ns;
}
static bool sugov_up_down_rate_limit(struct sugov_policy *sg_policy, u64 time,
				     unsigned int next_freq)
{
	s64 delta_ns;
	delta_ns = time - sg_policy->last_freq_update_time;
	if (next_freq > sg_policy->next_freq &&
	    delta_ns < sg_policy->up_rate_delay_ns)
			return true;
	if (next_freq < sg_policy->next_freq &&
	    delta_ns < sg_policy->down_rate_delay_ns)
			return true;
	return false;
}
static inline bool use_pelt(void)
{
#ifdef CONFIG_SCHED_WALT
	return (!sysctl_sched_use_walt_cpu_util || walt_disabled);
#else
	return true;
#endif
}
static void sugov_update_commit(struct sugov_policy *sg_policy, u64 time,
				unsigned int next_freq)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	if (sugov_up_down_rate_limit(sg_policy, time, next_freq)) {
#ifdef CONFIG_STICKERNEL
		sg_policy->cached_raw_freq = sg_policy->stic_freq;
#else
		sg_policy->cached_raw_freq = 0;
#endif
		return;
	}
	if (sg_policy->next_freq == next_freq)
		return;
	sg_policy->next_freq = next_freq;
	sg_policy->last_freq_update_time = time;
	if (policy->fast_switch_enabled) {
		next_freq = cpufreq_driver_fast_switch(policy, next_freq);
		if (next_freq == CPUFREQ_ENTRY_INVALID)
			return;
		policy->cur = next_freq;
		trace_cpu_frequency(next_freq, smp_processor_id());
	} else {
#ifdef CONFIG_SCHED_WALT
		sg_policy->work_in_progress = true;
#else
		if (use_pelt())
			sg_policy->work_in_progress = true;
#endif
		irq_work_queue(&sg_policy->irq_work);
	}
}
#define TARGET_LOAD 80
static unsigned int get_next_freq(struct sugov_policy *sg_policy,
				  unsigned long util, unsigned long max)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int freq = arch_scale_freq_invariant() ?
				policy->cpuinfo.max_freq : policy->cur;
	freq = (freq + (freq >> 2)) * util / max;
	if (freq == sg_policy->cached_raw_freq && sg_policy->next_freq != UINT_MAX)
		return sg_policy->next_freq;
	sg_policy->cached_raw_freq = freq;
	return cpufreq_driver_resolve_freq(policy, freq);
}
static void sugov_get_util(unsigned long *util, unsigned long *max, int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long cfs_max;
	struct sugov_cpu *loadcpu = &per_cpu(sugov_cpu, cpu);
	cfs_max = arch_scale_cpu_capacity(NULL, cpu);
	*util = min(rq->cfs.avg.util_avg, cfs_max);
	*max = cfs_max;
	*util = boosted_cpu_util(cpu, &loadcpu->walt_load);
}
static void sugov_set_iowait_boost(struct sugov_cpu *sg_cpu, u64 time,
				   unsigned int flags)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	if (!sg_policy->tunables->iowait_boost_enable)
		return;
	if (flags & SCHED_CPUFREQ_IOWAIT) {
		sg_cpu->iowait_boost = sg_cpu->iowait_boost_max;
	} else if (sg_cpu->iowait_boost) {
		s64 delta_ns = time - sg_cpu->last_update;
		if (delta_ns > TICK_NSEC)
			sg_cpu->iowait_boost = 0;
	}
}
static void sugov_iowait_boost(struct sugov_cpu *sg_cpu, unsigned long *util,
			       unsigned long *max)
{
	unsigned long boost_util = sg_cpu->iowait_boost;
	unsigned long boost_max = sg_cpu->iowait_boost_max;
	if (!boost_util)
		return;
	if (*util * boost_max < *max * boost_util) {
		*util = boost_util;
		*max = boost_max;
	}
	sg_cpu->iowait_boost >>= 1;
}
static unsigned long freq_to_util(struct sugov_policy *sg_policy,
				  unsigned int freq)
{
	return mult_frac(sg_policy->max, freq,
			 sg_policy->policy->cpuinfo.max_freq);
}
#define KHZ 1000
static void sugov_track_cycles(struct sugov_policy *sg_policy,
				unsigned int prev_freq,
				u64 upto)
{
	u64 delta_ns, cycles;
	if (unlikely(!sysctl_sched_use_walt_cpu_util))
		return;
	delta_ns = upto - sg_policy->last_cyc_update_time;
	delta_ns *= prev_freq;
	do_div(delta_ns, (NSEC_PER_SEC / KHZ));
	cycles = delta_ns;
	sg_policy->curr_cycles += cycles;
	sg_policy->last_cyc_update_time = upto;
}
static void sugov_calc_avg_cap(struct sugov_policy *sg_policy, u64 curr_ws,
				unsigned int prev_freq)
{
	u64 last_ws = sg_policy->last_ws;
	unsigned int avg_freq;
	if (unlikely(!sysctl_sched_use_walt_cpu_util))
		return;
	WARN_ON(curr_ws < last_ws);
	if (curr_ws <= last_ws)
		return;
	if (curr_ws > (last_ws + sched_ravg_window)) {
		avg_freq = prev_freq;
		sg_policy->last_cyc_update_time = curr_ws;
	} else {
		sugov_track_cycles(sg_policy, prev_freq, curr_ws);
		avg_freq = sg_policy->curr_cycles;
		avg_freq /= sched_ravg_window / (NSEC_PER_SEC / KHZ);
	}
	sg_policy->avg_cap = freq_to_util(sg_policy, avg_freq);
	sg_policy->curr_cycles = 0;
	sg_policy->last_ws = curr_ws;
}
#define NL_RATIO 75
#define DEFAULT_HISPEED_LOAD 90
static void sugov_walt_adjust(struct sugov_cpu *sg_cpu, unsigned long *util,
			      unsigned long *max)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	bool is_migration = sg_cpu->flags & SCHED_CPUFREQ_INTERCLUSTER_MIG;
	unsigned long nl = sg_cpu->walt_load.nl;
	unsigned long cpu_util = sg_cpu->util;
	bool is_hiload;
	if (unlikely(!sysctl_sched_use_walt_cpu_util))
		return;
	is_hiload = (cpu_util >= mult_frac(sg_policy->avg_cap,
					   sg_policy->tunables->hispeed_load,
					   100));
	if (is_hiload && !is_migration)
		*util = max(*util, sg_policy->hispeed_util);
	if (is_hiload && nl >= mult_frac(cpu_util, NL_RATIO, 100))
		*util = *max;
	if (sg_policy->tunables->pl)
		*util = max(*util, sg_cpu->walt_load.pl);
}
#ifdef CONFIG_NO_HZ_COMMON
static bool sugov_cpu_is_busy(struct sugov_cpu *sg_cpu)
{
	unsigned long idle_calls = tick_nohz_get_idle_calls();
	bool ret = idle_calls == sg_cpu->saved_idle_calls;
	sg_cpu->saved_idle_calls = idle_calls;
	return ret;
}
#else
static inline bool sugov_cpu_is_busy(struct sugov_cpu *sg_cpu) { return false; }
#endif
static void sugov_update_single(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util, max, hs_util;
	unsigned int next_f;
	bool busy;
	flags &= ~SCHED_CPUFREQ_RT_DL;
	if (!sg_policy->tunables->pl && flags & SCHED_CPUFREQ_PL)
		return;
	sugov_set_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;
	if (!sugov_should_update_freq(sg_policy, time))
		return;
#ifdef CONFIG_SCHED_WALT
	busy = sugov_cpu_is_busy(sg_cpu);
#else
	busy = use_pelt() && sugov_cpu_is_busy(sg_cpu);
#endif
	raw_spin_lock(&sg_policy->update_lock);
	if (flags & SCHED_CPUFREQ_RT_DL) {
		sg_policy->cached_raw_freq = 0;
		next_f = policy->cpuinfo.max_freq;
	} else {
		sugov_get_util(&util, &max, sg_cpu->cpu);
		if (sg_policy->max != max) {
			sg_policy->max = max;
			hs_util = freq_to_util(sg_policy,
					sg_policy->tunables->hispeed_freq);
			hs_util = mult_frac(hs_util, TARGET_LOAD, 100);
			sg_policy->hispeed_util = hs_util;
		}
		sg_cpu->util = util;
		sg_cpu->max = max;
		sg_cpu->flags = flags;
		sugov_calc_avg_cap(sg_policy, sg_cpu->walt_load.ws,
				   sg_policy->policy->cur);
		trace_sugov_util_update(sg_cpu->cpu, sg_cpu->util,
					sg_policy->avg_cap,
					max, sg_cpu->walt_load.nl,
					sg_cpu->walt_load.pl, flags);
		sugov_iowait_boost(sg_cpu, &util, &max);
		sugov_walt_adjust(sg_cpu, &util, &max);
		next_f = get_next_freq(sg_policy, util, max);
		if (busy && next_f < sg_policy->next_freq) {
			next_f = sg_policy->next_freq;
			sg_policy->cached_raw_freq = 0;
		}
	}
	sugov_update_commit(sg_policy, time, next_f);
static void sugov_boost(struct gov_attr_set *attr_set)
{
	struct sugov_policy *sg_policy;
	u64 now;
	now = use_pelt() ? ktime_get_ns() : walt_ktime_clock();
	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sugov_update_commit(sg_policy, now, sg_policy->tunables->hispeed_freq);
	}
}
static inline bool sugov_cpu_is_overload(struct sugov_cpu *sg_cpu, u64 time)
{
	u64 idle_time = get_cpu_idle_time(sg_cpu->cpu, NULL, 0);
	u64 delta;
	if (sg_cpu->last_idle_time != idle_time)
		sg_cpu->idle_update_ts = time;
	sg_cpu->last_idle_time = idle_time;
	delta = time - sg_cpu->idle_update_ts;
	raw_spin_unlock(&sg_policy->update_lock);
}
static void sugov_update_shared(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned long util, max, hs_util;
	unsigned int next_f;
	if (!sg_policy->tunables->pl && flags & SCHED_CPUFREQ_PL)
		return;
	sugov_get_util(&util, &max, sg_cpu->cpu);
	flags &= ~SCHED_CPUFREQ_RT_DL;
	raw_spin_lock(&sg_policy->update_lock);
	if (sg_policy->max != max) {
		sg_policy->max = max;
		hs_util = freq_to_util(sg_policy,
					sg_policy->tunables->hispeed_freq);
		hs_util = mult_frac(hs_util, TARGET_LOAD, 100);
		sg_policy->hispeed_util = hs_util;
	}
	sg_cpu->util = util;
	sg_cpu->max = max;
	sg_cpu->flags = flags;
	sugov_set_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;
	sugov_calc_avg_cap(sg_policy, sg_cpu->walt_load.ws,
			   sg_policy->policy->cur);
	trace_sugov_util_update(sg_cpu->cpu, sg_cpu->util, sg_policy->avg_cap,
				max, sg_cpu->walt_load.nl,
				sg_cpu->walt_load.pl, flags);
	if (sugov_should_update_freq(sg_policy, time)) {
		if (flags & SCHED_CPUFREQ_RT_DL) {
			next_f = sg_policy->policy->cpuinfo.max_freq;
			sg_policy->cached_raw_freq = 0;
		} else {
			next_f = sugov_next_freq_shared(sg_cpu, time);
		}
		sugov_update_commit(sg_policy, time, next_f);
	}
	raw_spin_unlock(&sg_policy->update_lock);
}
static bool sugov_time_limit(struct sugov_policy *sg_policy, unsigned int next_freq,
				int skip_min_sample_time, int skip_hispeed_logic)
{
	u64 delta_ns;
	unsigned int min_sample_time;
	if (!skip_hispeed_logic &&
	    next_freq > sg_policy->next_freq &&
	    sg_policy->next_freq >= sg_policy->tunables->hispeed_freq) {
		delta_ns = sg_policy->time - sg_policy->hispeed_validate_time;
		if (delta_ns < NSEC_PER_USEC *
		    freq_to_above_hispeed_delay(sg_policy->tunables, sg_policy->next_freq)) {
			return true;
	}
	}
	sg_policy->hispeed_validate_time = sg_policy->time;
	if (next_freq < sg_policy->next_freq) {
		min_sample_time = freq_to_min_sample_time(sg_policy->tunables, sg_policy->next_freq);
		if (LONGER_MIN_SAMPLE_TIME_ELAPSED_DURATION > ktime_to_ns(ktime_sub(ktime_get(), sg_policy->end_time))) { //lint !e446
			min_sample_time = (min_sample_time > DEFAULT_RATE_LIMIT_US) ?
					    min_sample_time : DEFAULT_RATE_LIMIT_US;
			skip_min_sample_time = 0;
	}
		if (!skip_min_sample_time) {
			delta_ns = sg_policy->time - sg_policy->floor_validate_time;
			if (delta_ns < NSEC_PER_USEC * min_sample_time) {
				return true;
	}
	}
	}
	if (!sg_policy->tunables->boosted ||
	    next_freq > sg_policy->tunables->hispeed_freq)
		sg_policy->floor_validate_time = sg_policy->time;

	return false;
}

void sugov_mark_util_change(int cpu, unsigned int flags)
{
	struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);
	struct sugov_policy *sg_policy;
	bool skip_min_sample_time = false;
	bool skip_hispeed_logic = false;
	if (!sg_cpu->enabled)
		return;
	sg_policy = sg_cpu->sg_policy;
	if (!sg_policy)
		return;
	if (!sg_policy->governor_enabled)
		return;
	if (use_pred_load(cpu)) {
		if (flags == WALT_WINDOW_ROLLOVER)
			return;
	} else {
		if (flags == PRED_LOAD_WINDOW_ROLLOVER ||
		    flags == PRED_LOAD_CHANGE)
			return;
	}
	sg_cpu->flags |= flags;
	if (flags & INTER_CLUSTER_MIGRATION_SRC)
		if (sg_policy->tunables->fast_ramp_down)
			skip_min_sample_time = true;
	if (flags & INTER_CLUSTER_MIGRATION_DST)
		if (sg_policy->tunables->fast_ramp_up)
			skip_hispeed_logic = true;
	sg_cpu = &per_cpu(sugov_cpu, cpu);
	if (!sg_cpu->enabled)
		return;
	sg_policy = sg_cpu->sg_policy;
	if (!sg_policy)
		return;
	if (!sg_policy->governor_enabled)
		return;
	if (sg_policy->util_changed)
		cpufreq_update_util(cpu_rq(cpu), 0);
}
static void sugov_work(struct kthread_work *work)
{
	struct sugov_policy *sg_policy = container_of(work, struct sugov_policy, work);
	unsigned long flags;
	mutex_lock(&sg_policy->work_lock);
	raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
	sugov_track_cycles(sg_policy, sg_policy->policy->cur,
			   ktime_get_ns());
	raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);
	__cpufreq_driver_target(sg_policy->policy, sg_policy->next_freq,
				CPUFREQ_RELATION_L);
	mutex_unlock(&sg_policy->work_lock);
#ifdef CONFIG_SCHED_WALT
	sg_policy->work_in_progress = false;
#else
	if (use_pelt())
		sg_policy->work_in_progress = false;
#endif
}
static void sugov_irq_work(struct irq_work *irq_work)
{
	struct sugov_policy *sg_policy;
	sg_policy = container_of(irq_work, struct sugov_policy, irq_work);
	kthread_queue_work(&sg_policy->worker, &sg_policy->work);
}
static struct sugov_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);
static inline struct sugov_tunables *to_sugov_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct sugov_tunables, attr_set);
}
static DEFINE_MUTEX(min_rate_lock);
static void update_min_rate_limit_us(struct sugov_policy *sg_policy)
{
	mutex_lock(&min_rate_lock);
	sg_policy->min_rate_limit_ns = min(sg_policy->up_rate_delay_ns,
					   sg_policy->down_rate_delay_ns);
	mutex_unlock(&min_rate_lock);
}
static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->up_rate_limit_us);
}
static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->down_rate_limit_us);
}
static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int rate_limit_us;
	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;
	tunables->up_rate_limit_us = rate_limit_us;
	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->up_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_us(sg_policy);
	}
	return count;
}
static ssize_t boost_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->boost);
}
static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int rate_limit_us;
	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;
	tunables->down_rate_limit_us = rate_limit_us;
	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->down_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_us(sg_policy);
	}
	return count;
}
static ssize_t boostpulse_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;
	u64 now;
	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	now = ktime_to_us(ktime_get());
	if (tunables->boostpulse_endtime + tunables->boostpulse_min_interval > now)
		return count;
	tunables->boostpulse_endtime = now + tunables->boostpulse_duration;
	if (!tunables->boosted)
		sugov_boost(attr_set);
	return count;
}
static ssize_t hispeed_load_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->hispeed_load);
}
static ssize_t hispeed_load_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	if (kstrtouint(buf, 10, &tunables->hispeed_load))
		return -EINVAL;
	tunables->hispeed_load = min(100U, tunables->hispeed_load);
	return count;
}
static ssize_t hispeed_freq_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->hispeed_freq);
}
static ssize_t hispeed_freq_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;
	struct sugov_policy *sg_policy;
	unsigned long hs_util;
	unsigned long flags;
	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	tunables->hispeed_freq = val;
	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
		hs_util = freq_to_util(sg_policy,
					sg_policy->tunables->hispeed_freq);
		hs_util = mult_frac(hs_util, TARGET_LOAD, 100);
		sg_policy->hispeed_util = hs_util;
		raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);
	}
	return count;
}
static ssize_t pl_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->pl);
}
static ssize_t pl_store(struct gov_attr_set *attr_set, const char *buf,
				   size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	if (kstrtobool(buf, &tunables->pl))
		return -EINVAL;
	return count;
}
static ssize_t iowait_boost_enable_show(struct gov_attr_set *attr_set,
					char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	return sprintf(buf, "%u\n", tunables->iowait_boost_enable);
}
static ssize_t iowait_boost_enable_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	bool enable;
	if (kstrtobool(buf, &enable))
		return -EINVAL;
	tunables->iowait_boost_enable = enable;
	return count;
}
static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);
static struct governor_attr hispeed_load = __ATTR_RW(hispeed_load);
static struct governor_attr hispeed_freq = __ATTR_RW(hispeed_freq);
static struct governor_attr pl = __ATTR_RW(pl);
static struct governor_attr iowait_boost_enable = __ATTR_RW(iowait_boost_enable);
static struct attribute *sugov_attributes[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&hispeed_load.attr,
	&hispeed_freq.attr,
	&pl.attr,
	&iowait_boost_enable.attr,
	NULL
};
static struct kobj_type sugov_tunables_ktype = {
	.default_attrs = sugov_attributes,
	.sysfs_ops = &governor_sysfs_ops,
};
/********************** cpufreq governor interface *********************/
static struct cpufreq_governor realmeutil_gov;
static struct sugov_policy *sugov_policy_alloc(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;
	sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
	if (!sg_policy)
		return NULL;
	sg_policy->policy = policy;
	raw_spin_lock_init(&sg_policy->update_lock);
	return sg_policy;
}
static void sugov_policy_free(struct sugov_policy *sg_policy)
{
	kfree(sg_policy);
}
static int sugov_kthread_create(struct sugov_policy *sg_policy)
{
	struct task_struct *thread;
	struct cpufreq_policy *policy = sg_policy->policy;
	int __maybe_unused ret;
	/* kthread only required for slow path */
	if (policy->fast_switch_enabled)
		return 0;
	kthread_init_work(&sg_policy->work, sugov_work);
	kthread_init_worker(&sg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
				"sugov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create sugov thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}
	
	sg_policy->thread = thread;
	kthread_bind_mask(thread, policy->related_cpus);
	init_irq_work(&sg_policy->irq_work, sugov_irq_work);
	mutex_init(&sg_policy->work_lock);
	wake_up_process(thread);
	return 0;
}
static void sugov_kthread_stop(struct sugov_policy *sg_policy)
{
	/* kthread only required for slow path */
	if (sg_policy->policy->fast_switch_enabled)
		return;
	kthread_flush_worker(&sg_policy->worker);
	kthread_stop(sg_policy->thread);
	mutex_destroy(&sg_policy->work_lock);
}
static struct sugov_tunables *sugov_tunables_alloc(struct sugov_policy *sg_policy)
{
	struct sugov_tunables *tunables;
	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (tunables) {
		gov_attr_set_init(&tunables->attr_set, &sg_policy->tunables_hook);
		if (!have_governor_per_policy())
			global_tunables = tunables;
	}
	return tunables;
}
static void sugov_tunables_save(struct cpufreq_policy *policy,
		struct sugov_tunables *tunables)
{
	int cpu;
	struct sugov_tunables *cached = per_cpu(cached_tunables, policy->cpu);
	if (!have_governor_per_policy())
		return;
	if (!cached) {
		cached = kzalloc(sizeof(*tunables), GFP_KERNEL);
		if (!cached) {
			pr_warn("Couldn't allocate tunables for caching\n");
			return;
		}
		for_each_cpu(cpu, policy->related_cpus)
			per_cpu(cached_tunables, cpu) = cached;
	}
	cached->pl = tunables->pl;
	cached->hispeed_load = tunables->hispeed_load;
	cached->hispeed_freq = tunables->hispeed_freq;
	cached->up_rate_limit_us = tunables->up_rate_limit_us;
	cached->down_rate_limit_us = tunables->down_rate_limit_us;
}
static void sugov_tunables_free(struct sugov_tunables *tunables)
{
	if (!have_governor_per_policy())
		global_tunables = NULL;
	kfree(tunables);
}
static void sugov_tunables_restore(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	struct sugov_tunables *tunables = sg_policy->tunables;
	struct sugov_tunables *cached = per_cpu(cached_tunables, policy->cpu);
	if (!cached)
		return;
	tunables->pl = cached->pl;
	tunables->hispeed_load = cached->hispeed_load;
	tunables->hispeed_freq = cached->hispeed_freq;
	tunables->up_rate_limit_us = cached->up_rate_limit_us;
	tunables->down_rate_limit_us = cached->down_rate_limit_us;
	sg_policy->up_rate_delay_ns = cached->up_rate_limit_us;
	sg_policy->down_rate_delay_ns = cached->down_rate_limit_us;
	update_min_rate_limit_us(sg_policy);
}
static int sugov_init(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;
	struct sugov_tunables *tunables;
	unsigned int lat;
	int ret = 0;
	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;
	cpufreq_enable_fast_switch(policy);
	sg_policy = sugov_policy_alloc(policy);
	if (!sg_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}
	ret = sugov_kthread_create(sg_policy);
	if (ret)
		goto free_sg_policy;
	mutex_lock(&global_tunables_lock);
	if (global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = sg_policy;
		sg_policy->tunables = global_tunables;
		gov_attr_set_get(&global_tunables->attr_set, &sg_policy->tunables_hook);
		goto out;
	}
	tunables = sugov_tunables_alloc(sg_policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}
	tunables->up_rate_limit_us = LATENCY_MULTIPLIER;
	tunables->down_rate_limit_us = LATENCY_MULTIPLIER;
	tunables->hispeed_load = DEFAULT_HISPEED_LOAD;
	tunables->hispeed_freq = 0;
	lat = policy->cpuinfo.transition_latency / NSEC_PER_USEC;
	if (lat) {
		tunables->up_rate_limit_us *= lat;
		tunables->down_rate_limit_us *= lat;
	}
	tunables->iowait_boost_enable = false;
	policy->governor_data = sg_policy;
	sg_policy->tunables = tunables;
	stale_ns = sched_ravg_window + (sched_ravg_window >> 3);
	sugov_tunables_restore(policy);
	ret = kobject_init_and_add(&tunables->attr_set.kobj, &sugov_tunables_ktype,
				   get_governor_parent_kobj(policy), "%s",
				   realmeutil_gov.name);
	if (ret)
		goto fail;
out:
	mutex_unlock(&global_tunables_lock);
	return 0;
fail:
	policy->governor_data = NULL;
	sugov_tunables_free(tunables);
stop_kthread:
	sugov_kthread_stop(sg_policy);
free_sg_policy:
	mutex_unlock(&global_tunables_lock);
	sugov_policy_free(sg_policy);
disable_fast_switch:
	cpufreq_disable_fast_switch(policy);
	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}
static void sugov_exit(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	struct sugov_tunables *tunables = sg_policy->tunables;
	unsigned int count;
	mutex_lock(&global_tunables_lock);
	count = gov_attr_set_put(&tunables->attr_set, &sg_policy->tunables_hook);
	policy->governor_data = NULL;
	if (!count) {
		sugov_tunables_save(policy, tunables);
		sugov_tunables_free(tunables);
	}
	mutex_unlock(&global_tunables_lock);
	sugov_kthread_stop(sg_policy);
	sugov_policy_free(sg_policy);
	cpufreq_disable_fast_switch(policy);
}
static int sugov_start(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;
	sg_policy->up_rate_delay_ns =
		sg_policy->tunables->up_rate_limit_us * NSEC_PER_USEC;
	sg_policy->down_rate_delay_ns =
		sg_policy->tunables->down_rate_limit_us * NSEC_PER_USEC;
	update_min_rate_limit_us(sg_policy);
	sg_policy->last_freq_update_time = 0;
	sg_policy->next_freq = UINT_MAX;
	sg_policy->work_in_progress = false;
	sg_policy->need_freq_update = false;
	sg_policy->cached_raw_freq = 0;
	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);
		memset(sg_cpu, 0, sizeof(*sg_cpu));
		sg_cpu->sg_policy = sg_policy;
		sg_cpu->cpu = cpu;
		sg_cpu->flags = SCHED_CPUFREQ_RT;
		sg_cpu->iowait_boost_max = policy->cpuinfo.max_freq;
	}
	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);
		cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util,
					     policy_is_shared(policy) ?
							sugov_update_shared :
							sugov_update_single);
	}
	return 0;
}
static void sugov_stop(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;
	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);
	synchronize_sched();
	if (!policy->fast_switch_enabled) {
		irq_work_sync(&sg_policy->irq_work);
		kthread_cancel_work_sync(&sg_policy->work);
	}
}
static void sugov_limits(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned long flags;
	if (!policy->fast_switch_enabled) {
		mutex_lock(&sg_policy->work_lock);
		raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
		sugov_track_cycles(sg_policy, sg_policy->policy->cur,
				   ktime_get_ns());
		raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&sg_policy->work_lock);
	}
	sg_policy->need_freq_update = true;
}
static struct cpufreq_governor realmeutil_gov = {
	.name = "realmeutil",
	.owner = THIS_MODULE,
	.init = sugov_init,
	.exit = sugov_exit,
	.start = sugov_start,
	.stop = sugov_stop,
	.limits = sugov_limits,
};
#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_REALMEUTIL
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &realmeutil_gov;
}
#endif
static int __init sugov_register(void)
{
	return cpufreq_register_governor(&realmeutil_gov);
}
fs_initcall(sugov_register);