/****************************************************************************
 * (C) 2010 : The George Washingtonton University   
 ****************************************************************************
 *
 *        File: common/sched_vdi.c
 *      Author: Jinho Hwang
 *
 * Description: VDI-based SMP CPU scheduler
 *              Started from Credit Scheduler
 */

#include <xen/config.h>
#include <xen/init.h>
#include <xen/lib.h>
#include <xen/sched.h>
#include <xen/domain.h>
#include <xen/delay.h>
#include <xen/event.h>
#include <xen/time.h>
#include <xen/perfc.h>
#include <xen/sched-if.h>
#include <xen/softirq.h>
#include <asm/atomic.h>
#include <xen/errno.h>
#include <xen/keyhandler.h>


// jinho flag
#define VDI_SCHEDULE
//#define CREDIT_SCHEDULE

#define UTILITY_BASED
//#define PRI_DELAY_BASED

/*
 * VSCHED_STATS
 *
 * Manage very basic per-vCPU counters and stats.
 *
 * Useful for debugging live systems. The stats are displayed
 * with runq dumps ('r' on the Xen console).
 */
#ifdef PERF_COUNTERS
#define VSCHED_STATS
#endif


#define VSCHED_TICKS_PER_GRANUL     5
/*
 * Basic constants
 */
#define VSCHED_DEFAULT_WEIGHT       256
#define VSCHED_TICKS_PER_TSLICE     3
#define VSCHED_TICKS_PER_ACCT       3
#define VSCHED_MSECS_PER_TICK       10
#define VSCHED_MSECS_PER_TSLICE     \
    (VSCHED_MSECS_PER_TICK * VSCHED_TICKS_PER_TSLICE)
#define VSCHED_CREDITS_PER_MSEC     10
#define VSCHED_CREDITS_PER_TSLICE   \
    (VSCHED_CREDITS_PER_MSEC * VSCHED_MSECS_PER_TSLICE)
#define VSCHED_CREDITS_PER_ACCT     \
    (VSCHED_CREDITS_PER_MSEC * VSCHED_MSECS_PER_TICK * VSCHED_TICKS_PER_ACCT)


/*
 * Priorities
 */
#define VSCHED_PRI_TS_BOOST      0      /* time-share waking up */
#define VSCHED_PRI_TS_UNDER     -1      /* time-share w/ credits */
#define VSCHED_PRI_TS_OVER      -2      /* time-share w/o credits */
#define VSCHED_PRI_IDLE         -64     /* idle */


/*
 * Flags
 */
#define VSCHED_FLAG_VCPU_PARKED    0x0001  /* VCPU over capped credits */
#define VSCHED_FLAG_VCPU_YIELD     0x0002  /* VCPU yielding */


/*
 * Useful macros
 */
#define VSCHED_PRIV(_ops)   \
    ((struct vsched_private *)((_ops)->sched_data))
#define VSCHED_PCPU(_c)     \
    ((struct vsched_pcpu *)per_cpu(schedule_data, _c).sched_priv)
#define VSCHED_VCPU(_vcpu)  ((struct vsched_vcpu *) (_vcpu)->sched_priv)
#define VSCHED_DOM(_dom)    ((struct vsched_dom *) (_dom)->sched_priv)
#define RUNQ(_cpu)          (&(VSCHED_PCPU(_cpu)->runq))
#define VSCHED_CPUONLINE(_pool)    \
    (((_pool) == NULL) ? &cpupool_free_cpus : &(_pool)->cpu_valid)


/*
 * Stats
 */
#define VSCHED_STAT_CRANK(_X)               (perfc_incr(_X))

#ifdef VSCHED_STATS

#define VSCHED_VCPU_STATS_RESET(_V)                     \
    do                                                  \
    {                                                   \
        memset(&(_V)->stats, 0, sizeof((_V)->stats));   \
    } while ( 0 )

#define VSCHED_VCPU_STAT_CRANK(_V, _X)      (((_V)->stats._X)++)

#define VSCHED_VCPU_STAT_SET(_V, _X, _Y)    (((_V)->stats._X) = (_Y))

#else /* VSCHED_STATS */

#define VSCHED_VCPU_STATS_RESET(_V)         do {} while ( 0 )
#define VSCHED_VCPU_STAT_CRANK(_V, _X)      do {} while ( 0 )
#define VSCHED_VCPU_STAT_SET(_V, _X, _Y)    do {} while ( 0 )

#endif /* VSCHED_STATS */


/*
 * Boot parameters
 */
static bool_t __read_mostly sched_credit_default_yield;
boolean_param("sched_credit_default_yield", sched_credit_default_yield);

/*
 * Physical CPU
 */
struct vsched_pcpu {
    struct list_head runq;
    uint32_t runq_sort_last;
    struct timer ticker;
    unsigned int tick;
    unsigned int idle_bias;
};

/*
 * Virtual CPU
 */
struct vsched_vcpu {
    struct list_head runq_elem;
    struct list_head active_vcpu_elem;
    struct vsched_dom *sdom;
    struct vcpu *vcpu;
    atomic_t credit;
    s_time_t start_time;   /* When we were scheduled (used for credit) */
    // jinho added
    s_time_t sched_time;
    uint16_t flags;
    int16_t pri;
#ifdef VSCHED_STATS
    struct {
        int credit_last;
        uint32_t credit_incr;
        uint32_t state_active;
        uint32_t state_idle;
        uint32_t migrate_q;
        uint32_t migrate_r;
    } stats;
#endif
};

/*
 * Domain
 */
struct vsched_dom {
    struct list_head active_vcpu;
    struct list_head active_sdom_elem;
    struct domain *dom;
    uint16_t active_vcpu_count;
    uint16_t weight;
    uint16_t cap;
};

/*
 * System-wide private data
 */
struct vsched_private {
    spinlock_t lock;
    struct list_head active_sdom;
    uint32_t ncpus;
    struct timer  master_ticker;
    unsigned int master;
    cpumask_t idlers;
    cpumask_t cpus;
    uint32_t weight;
    uint32_t credit;
    int credit_balance;
    uint32_t runq_sort;
};

// jinho modified - start

#define jprintk printk
#define DOMID_IDLE_INT 32767

/*
 * Utility function parameters 
 */
struct vsched_util {
    uint32_t alpha;
    uint32_t time_limit; // us
    uint32_t ui;
    uint32_t weight;
};
struct vsched_util util_param[SCHED_SVC_NUM] = {
//    {1, 3000, 2500, 100}, {2, 5000, 1600, 50}, {3, 8000, 1300, 30}, {4, 10000, 1000, 10}
    {1, 1000, 2000, 100}, {2, 3000, 1950, 50}, {3, 5000, 1850, 30}, {4, 7000, 1000, 10}
};
uint32_t tmax = 2000;
uint32_t duration = 10;
uint32_t partial = 25; // 4%

#define VSCHED_STAT_MAX_DOM 10
struct vsched_stat {
    uint32_t dom[VSCHED_STAT_MAX_DOM]; // scheduled count
};
struct vsched_stat vstat;

struct vsched_vcpu* fake_vcpu[2] = {NULL, NULL};
// jinho modified - end

static inline int
__vcpu_on_runq(struct vsched_vcpu *svc)
{
    return !list_empty(&svc->runq_elem);
}

static inline struct vsched_vcpu *
__runq_elem(struct list_head *elem)
{
    return list_entry(elem, struct vsched_vcpu, runq_elem);
}

static inline void
__runq_insert(unsigned int cpu, struct vsched_vcpu *svc)
{
    const struct list_head * const runq = RUNQ(cpu);
    struct list_head *iter;

    BUG_ON( __vcpu_on_runq(svc) );
    BUG_ON( cpu != svc->vcpu->processor );

#ifdef CREDIT_SCHEDULE
    list_for_each( iter, runq )
    {
        const struct vsched_vcpu * const iter_svc = __runq_elem(iter);
        if ( svc->pri > iter_svc->pri )
            break;
    }

    /* If the vcpu yielded, try to put it behind one lower-priority
     * runnable vcpu if we can.  The next runq_sort will bring it forward
     * within 30ms if the queue too long. */
    if ( svc->flags & VSCHED_FLAG_VCPU_YIELD
         && __runq_elem(iter)->pri > VSCHED_PRI_IDLE )
    {
        iter=iter->next;

        /* Some sanity checks */
        BUG_ON(iter == runq);
    }
#endif

#ifdef VDI_SCHEDULE
    // jinho: just put in the queue: I don't need to care about yield
    // TODO: in order to reduce the overhead, I can put everything in queue aligned based on utility???
    iter =  runq->next;
#endif

    list_add_tail(&svc->runq_elem, iter);
}

static inline void
__runq_remove(struct vsched_vcpu *svc)
{
    BUG_ON( !__vcpu_on_runq(svc) );
    list_del_init(&svc->runq_elem);
}

// jinho modified start
/*
static inline struct vsched_vcpu * 
__runq_get(unsigned int cpu, struct vsched_vcpu* curr_vcpu)
{
    const struct list_head * const runq = RUNQ(cpu);
    struct list_head *iter;
    struct vsched_vcpu *result = curr_vcpu;
    uint32_t temp_util = 0;

    list_for_each( iter, runq )
    {
        struct vsched_vcpu *iter_svc = __runq_elem(iter);
        if ( iter_svc->vcpu->utility > temp_util ) {
            result = iter_svc;
            temp_util = iter_svc->vcpu->utility;
        }
    }

    if(result->sdom != NULL && result->vcpu != NULL) 
        printk("mylog - %d:%d:%d\n", result->sdom->dom->domain_id, result->vcpu->vcpu_id, temp_util);

    return result;
}

void displayQueue(unsigned int cpu)
{
    const struct list_head * const runq = RUNQ(cpu);
    struct list_head *iter;
    int temp = 0;

    list_for_each( iter, runq )
    {
        struct vsched_vcpu *iter_svc = __runq_elem(iter);
        jprintk("(%d:%d:%d:%d)", cpu, iter_svc->vcpu->domain->domain_id, 
            iter_svc->vcpu->vcpu_id, iter_svc->vcpu->utility);

        temp=1;
    }
    if(temp) printk("\n");
}
*/
// jinho modified end


// jinho modified - start
/* 
 * This returns utility value based on average time for each VCPU
 * TODO I need to control precisely
 */
uint32_t getUtil(struct vsched_vcpu *svc)
{
    uint32_t svt = svc->vcpu->domain->service_type; // service type
    int util = 0; // utility value to return
    s_time_t avgtime = svc->vcpu->avgtime; // us
    //s_time_t delay = svc->vcpu->delay; // us

    BUG_ON( svt == SCHED_SVC_UNKNOWN );

    // service 2 for reference : {3, 10000, 1850, 30}

    // {1, 1000, 2000, 100}, {2, 3000, 1950, 50}, {3, 5000, 1850, 30}, {4, 7000, 1000, 10}

    // 1. Original Utility Function
    if( avgtime < util_param[svt].time_limit ) { // this has not be satisfied
        util = util_param[svt].ui; // maximize to be scheduled
    } else {
        util = -util_param[svt].alpha * (avgtime/duration) + tmax; // scale down using duration
        util = (util < 0)? 1:util;
    }

    // 2. Delay consideration
    //util = (int)((double)util_param[svt].weight * ((double)delay / (double)(avgtime + 1)));

    return util;
}

/*
 * Update utilities for all VCPUs
 * TODO : CPU division
 */
#ifndef NSEC_TO_USEC
#define NSEC_TO_USEC(_nsec)     ((_nsec) / 1000ULL)
#endif
struct vsched_vcpu* updateAndPick(const int cpu, const struct scheduler *ops, struct vsched_vcpu *curr_svc) 
{
    // new interation
    const struct list_head * const runq = RUNQ(cpu);
    struct list_head *iter;

    // parameters
    struct domain *curr_dom = curr_svc->vcpu->domain;
    struct vsched_vcpu *top = curr_svc;

    s_time_t now = NOW();
    s_time_t sched_time = NSEC_TO_USEC(now - curr_svc->sched_time);
    int in_q_flag= 0;

    //int i = 0;

    // Never been scheduled
    if( curr_svc->sched_time == 0 ) {
        sched_time = 0;
    }

    /*
    jprintk("Scheduled %lld us [%d] (%d:%d:%d:%lld us:%lld us:%d)\n", sched_time, cpu, 
        top->vcpu->domain->domain_id, top->vcpu->vcpu_id, 
        top->vcpu->domain->service_type, top->vcpu->avgtime, top->vcpu->delay,
        top->vcpu->utility);
    */

    // Current One: The first one possibly dummy
    if( curr_dom->domain_id != DOMID_IDLE_INT ) {
        top->vcpu->avgtime = top->vcpu->avgtime + sched_time
                            - top->vcpu->avgtime/partial;
        top->vcpu->avgtime = (top->vcpu->avgtime < 0)? duration:top->vcpu->avgtime;
        top->vcpu->delay = 0;

        #ifdef UTILITY_BASED
        top->vcpu->utility = getUtil(top);
        #endif

        if( curr_dom->domain_id != 0 ) { // fix service type for dom0
            curr_dom->st_time -= sched_time;
            if(curr_dom->st_time <= 0) {
                curr_dom->st_time = SCHED_SVC_TYPE_TIME_MAX;
                if(curr_dom->service_type < SCHED_SVC_DEFAULT) {
                    curr_dom->service_type += 1;
                } else {
                    //curr_dom->service_type = SCHED_SVC_OFFLINE_INACTIVE;
                }
            }
        }
    }

    // Rest
    list_for_each( iter, runq )
    {
        struct vsched_vcpu *svc = __runq_elem(iter);
        struct domain *dom = svc->vcpu->domain;

        if( curr_svc == svc ) { in_q_flag = 1; }
        if( dom->domain_id == DOMID_IDLE_INT ) continue;

        // not consider the same one
        if(curr_svc != svc) {
            svc->vcpu->avgtime = svc->vcpu->avgtime - svc->vcpu->avgtime/partial;
            svc->vcpu->avgtime = (svc->vcpu->avgtime < 0)? duration:svc->vcpu->avgtime;
            svc->vcpu->delay = NSEC_TO_USEC(now - svc->sched_time);

            // TODO : service time for service management
            if( dom->domain_id != 0 ) {
                dom->st_time -= sched_time;
                if( dom->st_time <= 0 ) {
                    dom->st_time = SCHED_SVC_TYPE_TIME_MAX;
                    if( dom->service_type < SCHED_SVC_DEFAULT ) {
                        dom->service_type += 1;
                    } else {
                        //dom->service_type = SCHED_SVC_OFFLINE_INACTIVE; // this means idle, so we don't need this for scheduling
                    }
                }
            }

            #ifdef UTILITY_BASED
            svc->vcpu->utility = getUtil(svc);

            // TODO: Equality should hold? yes or no?
            // If Current Domain is DOMMY, force changing
            if( curr_svc->vcpu->domain->domain_id == DOMID_IDLE_INT 
                || svc->vcpu->utility >= top->vcpu->utility ) {
                top = svc;
            }
            #endif

            #ifdef PRI_DELAY_BASED
            if( dom->service_type < curr_dom->service_type ) {
                top = svc;
                curr_dom = dom;
            } else if( dom->service_type == curr_dom->service_type ) { 
                if( svc->vcpu->delay > top->vcpu->delay ) {
                    top = svc;
                    curr_dom = dom;
                }
            }
            #endif
        } 
    }

    // Check whether the result is in the run queue 
    // we should not schedule one which is not in the run queue
    if(in_q_flag == 0) {
        #ifdef UTILITY_BASED
        uint32_t temp_util = 0;
        #endif

        #ifdef PRI_DELAY_BASED
        uint32_t temp_service_type = SCHED_SVC_NUM; //max
        uint32_t temp_delay = 0;
        #endif

        list_for_each( iter, runq )
        {
            struct vsched_vcpu *svc = __runq_elem(iter);

            #ifdef UTILITY_BASED
            if( svc->vcpu->utility >= temp_util ) { // include 0
                top = svc;
                temp_util = svc->vcpu->utility;
            }
            #endif

            #ifdef PRI_DELAY_BASED
            struct domain *dom = svc->vcpu->domain;
            if( dom->service_type < temp_service_type ) {
                top = svc;
                curr_dom = dom;
                temp_service_type = dom->service_type;
            } else if( dom->service_type == temp_service_type ) { 
                if( svc->vcpu->delay >= temp_delay ) {
                    top = svc;
                    curr_dom = dom;
                    temp_delay = svc->vcpu->delay;
                }
            }
            #endif
        }
    }

    top->sched_time = now;

    // TODO : either sort queue or get biggest when scheduling
    // Inform each CPU that its runq needs to be sorted
    //prv->runq_sort++; // change sorting value, not based on the credit, but based on the utility

    return top;
}

struct vsched_vcpu* pinSched(const int cpu, struct vsched_vcpu *curr_svc) 
{
    // new interation
    const struct list_head * const runq = RUNQ(cpu);
    struct list_head *iter;

    // parameters
    struct vsched_vcpu *svc = curr_svc;
    struct vsched_vcpu *top = curr_svc;

    if( curr_svc->vcpu->domain->domain_id == DOMID_IDLE_INT ) {
        list_for_each( iter, runq )
        {
            svc = __runq_elem(iter);
            top = svc; // simply change
        }
    }

    return top;
}

// jinho modified - end

static void vsched_tick(void *_cpu);
static void vsched_acct(void *dummy);

static void burn_credits(struct vsched_vcpu *svc, s_time_t now)
{
    s_time_t delta;
    unsigned int credits;

    /* Assert svc is current */
    ASSERT(svc==VSCHED_VCPU(per_cpu(schedule_data, svc->vcpu->processor).curr));

    if ( (delta = now - svc->start_time) <= 0 )
        return;

    credits = (delta*VSCHED_CREDITS_PER_MSEC + MILLISECS(1)/2) / MILLISECS(1);
    atomic_sub(credits, &svc->credit);
    svc->start_time += (credits * MILLISECS(1)) / VSCHED_CREDITS_PER_MSEC;
}

static bool_t __read_mostly opt_tickle_one_idle = 1;
boolean_param("tickle_one_idle_cpu", opt_tickle_one_idle);

DEFINE_PER_CPU(unsigned int, vdi_last_tickle_cpu);

static inline void
__runq_tickle(unsigned int cpu, struct vsched_vcpu *new)
{
    struct vsched_vcpu * const cur =
        VSCHED_VCPU(per_cpu(schedule_data, cpu).curr);
    struct vsched_private *prv = VSCHED_PRIV(per_cpu(scheduler, cpu));
    cpumask_t mask;

    ASSERT(cur);
    cpus_clear(mask);

    /* If strictly higher priority than current VCPU, signal the CPU */
    if ( new->pri > cur->pri )
    {
        if ( cur->pri == VSCHED_PRI_IDLE )
            VSCHED_STAT_CRANK(tickle_local_idler);
        else if ( cur->pri == VSCHED_PRI_TS_OVER )
            VSCHED_STAT_CRANK(tickle_local_over);
        else if ( cur->pri == VSCHED_PRI_TS_UNDER )
            VSCHED_STAT_CRANK(tickle_local_under);
        else
            VSCHED_STAT_CRANK(tickle_local_other);

        cpu_set(cpu, mask);
    }

    /*
     * If this CPU has at least two runnable VCPUs, we tickle any idlers to
     * let them know there is runnable work in the system...
     */
    if ( cur->pri > VSCHED_PRI_IDLE )
    {
        if ( cpus_empty(prv->idlers) )
        {
            VSCHED_STAT_CRANK(tickle_idlers_none);
        }
        else
        {
            cpumask_t idle_mask;

            cpus_and(idle_mask, prv->idlers, new->vcpu->cpu_affinity);
            if ( !cpus_empty(idle_mask) )
            {
                VSCHED_STAT_CRANK(tickle_idlers_some);
                if ( opt_tickle_one_idle )
                {
                    this_cpu(vdi_last_tickle_cpu) = 
                        cycle_cpu(this_cpu(vdi_last_tickle_cpu), idle_mask);
                    cpu_set(this_cpu(vdi_last_tickle_cpu), mask);
                }
                else
                    cpus_or(mask, mask, idle_mask);
            }
            cpus_and(mask, mask, new->vcpu->cpu_affinity);
        }
    }

    /* Send scheduler interrupts to designated CPUs */
    if ( !cpus_empty(mask) )
        cpumask_raise_softirq(mask, SCHEDULE_SOFTIRQ);
}

static void
vsched_free_pdata(const struct scheduler *ops, void *pcpu, int cpu)
{
    struct vsched_private *prv = VSCHED_PRIV(ops);
    struct vsched_pcpu *spc = pcpu;
    unsigned long flags;

    if ( spc == NULL )
        return;

    spin_lock_irqsave(&prv->lock, flags);

    prv->credit -= VSCHED_CREDITS_PER_ACCT;
    prv->ncpus--;
    cpu_clear(cpu, prv->idlers);
    cpu_clear(cpu, prv->cpus);
    if ( (prv->master == cpu) && (prv->ncpus > 0) )
    {
        prv->master = first_cpu(prv->cpus);
        migrate_timer(&prv->master_ticker, prv->master);
    }
    kill_timer(&spc->ticker);
    if ( prv->ncpus == 0 )
        kill_timer(&prv->master_ticker);

    spin_unlock_irqrestore(&prv->lock, flags);

    xfree(spc);
}

static void *
vsched_alloc_pdata(const struct scheduler *ops, int cpu)
{
    struct vsched_pcpu *spc;
    struct vsched_private *prv = VSCHED_PRIV(ops);
    unsigned long flags;

    /* Allocate per-PCPU info */
    spc = xmalloc(struct vsched_pcpu);
    if ( spc == NULL )
        return NULL;
    memset(spc, 0, sizeof(*spc));

    spin_lock_irqsave(&prv->lock, flags);

    /* Initialize/update system-wide config */
    prv->credit += VSCHED_CREDITS_PER_ACCT;
    prv->ncpus++;
    cpu_set(cpu, prv->cpus);

    // jinho modified: we actually need timer to avoid monopoly
    // soft irq seems to happen when the cpu goes to idle
    if ( prv->ncpus == 1 )
    {
        prv->master = cpu;
        init_timer(&prv->master_ticker, vsched_acct, prv, cpu);
        //set_timer(&prv->master_ticker, NOW() +
        //          MILLISECS(VSCHED_MSECS_PER_TICK) * VSCHED_TICKS_PER_ACCT); // 30 ms
    }

    init_timer(&spc->ticker, vsched_tick, (void *)(unsigned long)cpu, cpu);
    //set_timer(&spc->ticker, NOW() + MILLISECS(VSCHED_MSECS_PER_TICK)); // 10 ms

    INIT_LIST_HEAD(&spc->runq);
    spc->runq_sort_last = prv->runq_sort;
    spc->idle_bias = NR_CPUS - 1;
    if ( per_cpu(schedule_data, cpu).sched_priv == NULL )
        per_cpu(schedule_data, cpu).sched_priv = spc;

    /* Start off idling... */
    BUG_ON(!is_idle_vcpu(per_cpu(schedule_data, cpu).curr));
    cpu_set(cpu, prv->idlers);

    spin_unlock_irqrestore(&prv->lock, flags);

    return spc;
}

#ifndef NDEBUG
static inline void
__vsched_vcpu_check(struct vcpu *vc)
{
    struct vsched_vcpu * const svc = VSCHED_VCPU(vc);
    struct vsched_dom * const sdom = svc->sdom;

    BUG_ON( svc->vcpu != vc );
    BUG_ON( sdom != VSCHED_DOM(vc->domain) );
    if ( sdom )
    {
        BUG_ON( is_idle_vcpu(vc) );
        BUG_ON( sdom->dom != vc->domain );
    }
    else
    {
        BUG_ON( !is_idle_vcpu(vc) );
    }

    VSCHED_STAT_CRANK(vcpu_check);
}
#define VSCHED_VCPU_CHECK(_vc)  (__vsched_vcpu_check(_vc))
#else
#define VSCHED_VCPU_CHECK(_vc)
#endif

/*
 * Delay, in microseconds, between migrations of a VCPU between PCPUs.
 * This prevents rapid fluttering of a VCPU between CPUs, and reduces the
 * implicit overheads such as cache-warming. 1ms (1000) has been measured
 * as a good value.
 */
static unsigned int vcpu_migration_delay;
integer_param("vcpu_migration_delay", vcpu_migration_delay);

/* jinho removed
void set_vcpu_migration_delay(unsigned int delay)
{
    vcpu_migration_delay = delay;
}

unsigned int get_vcpu_migration_delay(void)
{
    return vcpu_migration_delay;
}
*/

static inline int
__vsched_vcpu_is_cache_hot(struct vcpu *v)
{
#ifdef VDI_SCHEDULE
    // TODO: changing condition 
    int hot = 0;
    //= (v->domain->service_type > SCHED_SVC_ONLINE_ACTIVE);

    return hot;
#endif

#ifdef CREDIT_SCHEDULE
    int hot = ((NOW() - v->last_run_time) <
               ((uint64_t)vcpu_migration_delay * 1000u));

    if ( hot )
        VSCHED_STAT_CRANK(vcpu_hot);

    return hot;
#endif
}

static inline int
__vsched_vcpu_is_migrateable(struct vcpu *vc, int dest_cpu)
{
#ifdef VDI_SCHEDULE
    return !vc->is_running && !__vsched_vcpu_is_cache_hot(vc);
#endif

#ifdef CREDIT_SCHEDULE
    /*
     * Don't pick up work that's in the peer's scheduling tail or hot on
     * peer PCPU. Only pick up work that's allowed to run on our CPU.
     */
    return !vc->is_running &&
           !__vsched_vcpu_is_cache_hot(vc) &&
           cpu_isset(dest_cpu, vc->cpu_affinity);
#endif
}

static int
_vsched_cpu_pick(const struct scheduler *ops, struct vcpu *vc, bool_t commit)
{
    cpumask_t cpus;
    cpumask_t idlers;
    cpumask_t *online;
    int cpu;

    /*
     * Pick from online CPUs in VCPU's affinity mask, giving a
     * preference to its current processor if it's in there.
     */
    online = VSCHED_CPUONLINE(vc->domain->cpupool);
    cpus_and(cpus, *online, vc->cpu_affinity);
    cpu = cpu_isset(vc->processor, cpus)
            ? vc->processor
            : cycle_cpu(vc->processor, cpus);
    ASSERT( !cpus_empty(cpus) && cpu_isset(cpu, cpus) );

    /*
     * Try to find an idle processor within the above constraints.
     *
     * In multi-core and multi-threaded CPUs, not all idle execution
     * vehicles are equal!
     *
     * We give preference to the idle execution vehicle with the most
     * idling neighbours in its grouping. This distributes work across
     * distinct cores first and guarantees we don't do something stupid
     * like run two VCPUs on co-hyperthreads while there are idle cores
     * or sockets.
     */
    cpus_and(idlers, cpu_online_map, VSCHED_PRIV(ops)->idlers);
    cpu_set(cpu, idlers);
    cpus_and(cpus, cpus, idlers);
    cpu_clear(cpu, cpus);

    while ( !cpus_empty(cpus) )
    {
        cpumask_t cpu_idlers;
        cpumask_t nxt_idlers;
        int nxt, weight_cpu, weight_nxt;
        int migrate_factor;

        nxt = cycle_cpu(cpu, cpus);

        if ( cpu_isset(cpu, per_cpu(cpu_core_map, nxt)) )
        {
            /* We're on the same socket, so check the busy-ness of threads.
             * Migrate if # of idlers is less at all */
            ASSERT( cpu_isset(nxt, per_cpu(cpu_core_map, cpu)) );
            migrate_factor = 1;
            cpus_and(cpu_idlers, idlers, per_cpu(cpu_sibling_map, cpu));
            cpus_and(nxt_idlers, idlers, per_cpu(cpu_sibling_map, nxt));
        }
        else
        {
            /* We're on different sockets, so check the busy-ness of cores.
             * Migrate only if the other core is twice as idle */
            ASSERT( !cpu_isset(nxt, per_cpu(cpu_core_map, cpu)) );
            migrate_factor = 2;
            cpus_and(cpu_idlers, idlers, per_cpu(cpu_core_map, cpu));
            cpus_and(nxt_idlers, idlers, per_cpu(cpu_core_map, nxt));
        }

        weight_cpu = cpus_weight(cpu_idlers);
        weight_nxt = cpus_weight(nxt_idlers);
        /* smt_power_savings: consolidate work rather than spreading it */
        if ( ( sched_smt_power_savings
               && (weight_cpu > weight_nxt) )
             || ( !sched_smt_power_savings
                  && (weight_cpu * migrate_factor < weight_nxt) ) )
        {
            cpus_and(nxt_idlers, cpus, nxt_idlers);
            cpu = cycle_cpu(VSCHED_PCPU(nxt)->idle_bias, nxt_idlers);
            if ( commit )
               VSCHED_PCPU(nxt)->idle_bias = cpu;
            cpus_andnot(cpus, cpus, per_cpu(cpu_sibling_map, cpu));
        }
        else
        {
            cpus_andnot(cpus, cpus, nxt_idlers);
        }
    }

    return cpu;
}

static int
vsched_cpu_pick(const struct scheduler *ops, struct vcpu *vc)
{
    return _vsched_cpu_pick(ops, vc, 1);
}

static inline void
__vsched_vcpu_acct_start(struct vsched_private *prv, struct vsched_vcpu *svc)
{
    struct vsched_dom * const sdom = svc->sdom;
    unsigned long flags;

    spin_lock_irqsave(&prv->lock, flags);

    if ( list_empty(&svc->active_vcpu_elem) )
    {
        VSCHED_VCPU_STAT_CRANK(svc, state_active);
        VSCHED_STAT_CRANK(acct_vcpu_active);

        sdom->active_vcpu_count++;
        list_add(&svc->active_vcpu_elem, &sdom->active_vcpu);
        /* Make weight per-vcpu */
        prv->weight += sdom->weight;
        if ( list_empty(&sdom->active_sdom_elem) )
        {
            list_add(&sdom->active_sdom_elem, &prv->active_sdom);
        }
    }

    spin_unlock_irqrestore(&prv->lock, flags);
}

static inline void
__vsched_vcpu_acct_stop_locked(struct vsched_private *prv,
    struct vsched_vcpu *svc)
{
    struct vsched_dom * const sdom = svc->sdom;

    BUG_ON( list_empty(&svc->active_vcpu_elem) );

    VSCHED_VCPU_STAT_CRANK(svc, state_idle);
    VSCHED_STAT_CRANK(acct_vcpu_idle);

    BUG_ON( prv->weight < sdom->weight );
    sdom->active_vcpu_count--;
    list_del_init(&svc->active_vcpu_elem);
    prv->weight -= sdom->weight;
    if ( list_empty(&sdom->active_vcpu) )
    {
        list_del_init(&sdom->active_sdom_elem);
    }
}

static void
vsched_vcpu_acct(struct vsched_private *prv, unsigned int cpu)
{
    struct vsched_vcpu * const svc = VSCHED_VCPU(current);
    const struct scheduler *ops = per_cpu(scheduler, cpu);

    ASSERT( current->processor == cpu );
    ASSERT( svc->sdom != NULL );

    /*
     * If this VCPU's priority was boosted when it last awoke, reset it.
     * If the VCPU is found here, then it's consuming a non-negligeable
     * amount of CPU resources and should no longer be boosted.
     */
    if ( svc->pri == VSCHED_PRI_TS_BOOST )
        svc->pri = VSCHED_PRI_TS_UNDER;

    /*
     * Update credits
     */
    if ( !is_idle_vcpu(svc->vcpu) )
        burn_credits(svc, NOW());

    /*
     * Put this VCPU and domain back on the active list if it was
     * idling.
     *
     * If it's been active a while, check if we'd be better off
     * migrating it to run elsewhere (see multi-core and multi-thread
     * support in vsched_cpu_pick()).
     */
    if ( list_empty(&svc->active_vcpu_elem) )
    {
        __vsched_vcpu_acct_start(prv, svc);
    }
    else if ( _vsched_cpu_pick(ops, current, 0) != cpu )
    {
        VSCHED_VCPU_STAT_CRANK(svc, migrate_r);
        VSCHED_STAT_CRANK(migrate_running);
        set_bit(_VPF_migrating, &current->pause_flags);
        cpu_raise_softirq(cpu, SCHEDULE_SOFTIRQ);
    }
}

static void *
vsched_alloc_vdata(const struct scheduler *ops, struct vcpu *vc, void *dd)
{
    struct vsched_vcpu *svc;

    /* Allocate per-VCPU info */
    svc = xmalloc(struct vsched_vcpu);
    if ( svc == NULL )
        return NULL;
    memset(svc, 0, sizeof(*svc));

    INIT_LIST_HEAD(&svc->runq_elem);
    INIT_LIST_HEAD(&svc->active_vcpu_elem);
    svc->sdom = dd;
    svc->vcpu = vc;
    atomic_set(&svc->credit, 0);
    svc->flags = 0U;
    svc->pri = is_idle_domain(vc->domain) ?
        VSCHED_PRI_IDLE : VSCHED_PRI_TS_UNDER;
    VSCHED_VCPU_STATS_RESET(svc);
    VSCHED_STAT_CRANK(vcpu_init);
    return svc;
}

static void
vsched_vcpu_insert(const struct scheduler *ops, struct vcpu *vc)
{
    struct vsched_vcpu *svc = vc->sched_priv;

    if ( !__vcpu_on_runq(svc) && vcpu_runnable(vc) && !vc->is_running )
        __runq_insert(vc->processor, svc);
}

static void
vsched_free_vdata(const struct scheduler *ops, void *priv)
{
    struct vsched_vcpu *svc = priv;

    BUG_ON( !list_empty(&svc->runq_elem) );

    xfree(svc);
}

static void
vsched_vcpu_remove(const struct scheduler *ops, struct vcpu *vc)
{
    struct vsched_private *prv = VSCHED_PRIV(ops);
    struct vsched_vcpu * const svc = VSCHED_VCPU(vc);
    struct vsched_dom * const sdom = svc->sdom;
    unsigned long flags;

    VSCHED_STAT_CRANK(vcpu_destroy);

    if ( __vcpu_on_runq(svc) )
        __runq_remove(svc);

    spin_lock_irqsave(&(prv->lock), flags);

    if ( !list_empty(&svc->active_vcpu_elem) )
        __vsched_vcpu_acct_stop_locked(prv, svc);

    spin_unlock_irqrestore(&(prv->lock), flags);

    BUG_ON( sdom == NULL );
    BUG_ON( !list_empty(&svc->runq_elem) );
}

static void
vsched_vcpu_sleep(const struct scheduler *ops, struct vcpu *vc)
{
    struct vsched_vcpu * const svc = VSCHED_VCPU(vc);

    VSCHED_STAT_CRANK(vcpu_sleep);

    BUG_ON( is_idle_vcpu(vc) );

    if ( per_cpu(schedule_data, vc->processor).curr == vc )
        cpu_raise_softirq(vc->processor, SCHEDULE_SOFTIRQ);
    else if ( __vcpu_on_runq(svc) )
        __runq_remove(svc);
}

static void
vsched_vcpu_wake(const struct scheduler *ops, struct vcpu *vc)
{
    struct vsched_vcpu * const svc = VSCHED_VCPU(vc);
    const unsigned int cpu = vc->processor;

    BUG_ON( is_idle_vcpu(vc) );

    if ( unlikely(per_cpu(schedule_data, cpu).curr == vc) )
    {
        VSCHED_STAT_CRANK(vcpu_wake_running);
        return;
    }
    if ( unlikely(__vcpu_on_runq(svc)) )
    {
        VSCHED_STAT_CRANK(vcpu_wake_onrunq);
        return;
    }

    if ( likely(vcpu_runnable(vc)) )
        VSCHED_STAT_CRANK(vcpu_wake_runnable);
    else
        VSCHED_STAT_CRANK(vcpu_wake_not_runnable);

    /*
     * We temporarly boost the priority of awaking VCPUs!
     *
     * If this VCPU consumes a non negligeable amount of CPU, it
     * will eventually find itself in the credit accounting code
     * path where its priority will be reset to normal.
     *
     * If on the other hand the VCPU consumes little CPU and is
     * blocking and awoken a lot (doing I/O for example), its
     * priority will remain boosted, optimizing it's wake-to-run
     * latencies.
     *
     * This allows wake-to-run latency sensitive VCPUs to preempt
     * more CPU resource intensive VCPUs without impacting overall 
     * system fairness.
     *
     * The one exception is for VCPUs of capped domains unpausing
     * after earning credits they had overspent. We don't boost
     * those.
     */
    if ( svc->pri == VSCHED_PRI_TS_UNDER &&
         !(svc->flags & VSCHED_FLAG_VCPU_PARKED) )
    {
        svc->pri = VSCHED_PRI_TS_BOOST;
    }

    /* Put the VCPU on the runq and tickle CPUs */
    __runq_insert(cpu, svc);
    __runq_tickle(cpu, svc);
}

static void
vsched_vcpu_yield(const struct scheduler *ops, struct vcpu *vc)
{
    struct vsched_vcpu * const sv = VSCHED_VCPU(vc);

    if ( !sched_credit_default_yield )
    {
        /* Let the scheduler know that this vcpu is trying to yield */
        sv->flags |= VSCHED_FLAG_VCPU_YIELD;
    }
}

static int
vsched_dom_cntl(
    const struct scheduler *ops,
    struct domain *d,
    struct xen_domctl_scheduler_op *op)
{
    struct vsched_dom * const sdom = VSCHED_DOM(d);
    struct vsched_private *prv = VSCHED_PRIV(ops);
    unsigned long flags;

    if ( op->cmd == XEN_DOMCTL_SCHEDOP_getinfo )
    {
        op->u.credit.weight = sdom->weight;
        op->u.credit.cap = sdom->cap;
    }
    else
    {
        ASSERT(op->cmd == XEN_DOMCTL_SCHEDOP_putinfo);

        spin_lock_irqsave(&prv->lock, flags);

        if ( op->u.credit.weight != 0 )
        {
            if ( !list_empty(&sdom->active_sdom_elem) )
            {
                prv->weight -= sdom->weight * sdom->active_vcpu_count;
                prv->weight += op->u.credit.weight * sdom->active_vcpu_count;
            }
            sdom->weight = op->u.credit.weight;
        }

        if ( op->u.credit.cap != (uint16_t)~0U )
            sdom->cap = op->u.credit.cap;

        spin_unlock_irqrestore(&prv->lock, flags);
    }

    return 0;
}

static void *
vsched_alloc_domdata(const struct scheduler *ops, struct domain *dom)
{
    struct vsched_dom *sdom;

    sdom = xmalloc(struct vsched_dom);
    if ( sdom == NULL )
        return NULL;
    memset(sdom, 0, sizeof(*sdom));

    /* Initialize credit and weight */
    INIT_LIST_HEAD(&sdom->active_vcpu);
    sdom->active_vcpu_count = 0;
    INIT_LIST_HEAD(&sdom->active_sdom_elem);
    sdom->dom = dom;
    sdom->weight = VSCHED_DEFAULT_WEIGHT;
    sdom->cap = 0U;

    return (void *)sdom;
}

static int
vsched_dom_init(const struct scheduler *ops, struct domain *dom)
{
    struct vsched_dom *sdom;

    VSCHED_STAT_CRANK(dom_init);

    if ( is_idle_domain(dom) )
        return 0;

    sdom = vsched_alloc_domdata(ops, dom);
    if ( sdom == NULL )
        return -ENOMEM;

    dom->sched_priv = sdom;

    return 0;
}

static void
vsched_free_domdata(const struct scheduler *ops, void *data)
{
    xfree(data);
}

static void
vsched_dom_destroy(const struct scheduler *ops, struct domain *dom)
{
    VSCHED_STAT_CRANK(dom_destroy);
    vsched_free_domdata(ops, VSCHED_DOM(dom));
}

/*
 * This is a O(n) optimized sort of the runq.
 *
 * Time-share VCPUs can only be one of two priorities, UNDER or OVER. We walk
 * through the runq and move up any UNDERs that are preceded by OVERS. We
 * remember the last UNDER to make the move up operation O(1).
 */
static void
vsched_runq_sort(struct vsched_private *prv, unsigned int cpu)
{
    struct vsched_pcpu * const spc = VSCHED_PCPU(cpu);
    struct list_head *runq, *elem, *next, *last_under;
    struct vsched_vcpu *svc_elem;
    unsigned long flags;
    int sort_epoch;

    sort_epoch = prv->runq_sort;
    if ( sort_epoch == spc->runq_sort_last )
        return;

    spc->runq_sort_last = sort_epoch;

    pcpu_schedule_lock_irqsave(cpu, flags);

    runq = &spc->runq;
    elem = runq->next;
    last_under = runq;

    while ( elem != runq )
    {
        next = elem->next;
        svc_elem = __runq_elem(elem);

        if ( svc_elem->pri >= VSCHED_PRI_TS_UNDER )
        {
            // does elem need to move up the runq? 
            if ( elem->prev != last_under )
            {
                list_del(elem);
                list_add(elem, last_under);
            }
            last_under = elem;
        }

        elem = next;
    }

    pcpu_schedule_unlock_irqrestore(cpu, flags);
}

// jinho modified start
/*
static void
vsched_runq_util_sort(struct vsched_private *prv, unsigned int cpu)
{
    struct vsched_pcpu * const spc = VSCHED_PCPU(cpu);
    struct list_head *runq, *elem, *next, *last_under;
    struct vsched_vcpu *svc_elem;
    unsigned long flags;
    int sort_epoch;

    sort_epoch = prv->runq_sort;
    if ( sort_epoch == spc->runq_sort_last )
        return;

    spc->runq_sort_last = sort_epoch;

    pcpu_schedule_lock_irqsave(cpu, flags);

    runq = &spc->runq;
    elem = runq->next;
    last_under = runq;

    while ( elem != runq )
    {
        next = elem->next;
        svc_elem = __runq_elem(elem);

        if ( svc_elem->pri >= VSCHED_PRI_TS_UNDER )
        {
            if ( elem->prev != last_under )
            {
                list_del(elem);
                list_add(elem, last_under);
            }
            last_under = elem;
        }

        elem = next;
    }

    pcpu_schedule_unlock_irqrestore(cpu, flags);
}
*/
// jinho modified end

static void
vsched_acct(void* dummy)
{
    struct vsched_private *prv = dummy;
    unsigned long flags;
    struct list_head *iter_vcpu, *next_vcpu;
    struct list_head *iter_sdom, *next_sdom;
    struct vsched_vcpu *svc;
    struct vsched_dom *sdom;
    uint32_t credit_total;
    uint32_t weight_total;
    uint32_t weight_left;
    uint32_t credit_fair;
    uint32_t credit_peak;
    uint32_t credit_cap;
    int credit_balance;
    int credit_xtra;
    int credit;


    spin_lock_irqsave(&prv->lock, flags);

    weight_total = prv->weight;
    credit_total = prv->credit;

    /* Converge balance towards 0 when it drops negative */
    if ( prv->credit_balance < 0 )
    {
        credit_total -= prv->credit_balance;
        VSCHED_STAT_CRANK(acct_balance);
    }

    if ( unlikely(weight_total == 0) )
    {
        prv->credit_balance = 0;
        spin_unlock_irqrestore(&prv->lock, flags);
        VSCHED_STAT_CRANK(acct_no_work);
        goto out;
    }

    VSCHED_STAT_CRANK(acct_run);

    weight_left = weight_total;
    credit_balance = 0;
    credit_xtra = 0;
    credit_cap = 0U;

    list_for_each_safe( iter_sdom, next_sdom, &prv->active_sdom )
    {
        sdom = list_entry(iter_sdom, struct vsched_dom, active_sdom_elem);

        BUG_ON( is_idle_domain(sdom->dom) );
        BUG_ON( sdom->active_vcpu_count == 0 );
        BUG_ON( sdom->weight == 0 );
        BUG_ON( (sdom->weight * sdom->active_vcpu_count) > weight_left );

        weight_left -= ( sdom->weight * sdom->active_vcpu_count );

        /*
         * A domain's fair share is computed using its weight in competition
         * with that of all other active domains.
         *
         * At most, a domain can use credits to run all its active VCPUs
         * for one full accounting period. We allow a domain to earn more
         * only when the system-wide credit balance is negative.
         */
        credit_peak = sdom->active_vcpu_count * VSCHED_CREDITS_PER_ACCT;
        if ( prv->credit_balance < 0 )
        {
            credit_peak += ( ( -prv->credit_balance
                               * sdom->weight
                               * sdom->active_vcpu_count) +
                             (weight_total - 1)
                           ) / weight_total;
        }

        if ( sdom->cap != 0U )
        {
            credit_cap = ((sdom->cap * VSCHED_CREDITS_PER_ACCT) + 99) / 100;
            if ( credit_cap < credit_peak )
                credit_peak = credit_cap;

            /* FIXME -- set cap per-vcpu as well...? */
            credit_cap = ( credit_cap + ( sdom->active_vcpu_count - 1 )
                         ) / sdom->active_vcpu_count;
        }

        credit_fair = ( ( credit_total
                          * sdom->weight
                          * sdom->active_vcpu_count )
                        + (weight_total - 1)
                      ) / weight_total;

        if ( credit_fair < credit_peak )
        {
            credit_xtra = 1;
        }
        else
        {
            if ( weight_left != 0U )
            {
                /* Give other domains a chance at unused credits */
                credit_total += ( ( ( credit_fair - credit_peak
                                    ) * weight_total
                                  ) + ( weight_left - 1 )
                                ) / weight_left;
            }

            if ( credit_xtra )
            {
                /*
                 * Lazily keep domains with extra credits at the head of
                 * the queue to give others a chance at them in future
                 * accounting periods.
                 */
                VSCHED_STAT_CRANK(acct_reorder);
                list_del(&sdom->active_sdom_elem);
                list_add(&sdom->active_sdom_elem, &prv->active_sdom);
            }

            credit_fair = credit_peak;
        }

        /* Compute fair share per VCPU */
        credit_fair = ( credit_fair + ( sdom->active_vcpu_count - 1 )
                      ) / sdom->active_vcpu_count;


        list_for_each_safe( iter_vcpu, next_vcpu, &sdom->active_vcpu )
        {
            svc = list_entry(iter_vcpu, struct vsched_vcpu, active_vcpu_elem);
            BUG_ON( sdom != svc->sdom );

            /* Increment credit */
            atomic_add(credit_fair, &svc->credit);
            credit = atomic_read(&svc->credit);

            /*
             * Recompute priority or, if VCPU is idling, remove it from
             * the active list.
             */
            if ( credit < 0 )
            {
                svc->pri = VSCHED_PRI_TS_OVER;

                /* Park running VCPUs of capped-out domains */
                if ( sdom->cap != 0U &&
                     credit < -credit_cap &&
                     !(svc->flags & VSCHED_FLAG_VCPU_PARKED) )
                {
                    VSCHED_STAT_CRANK(vcpu_park);
                    vcpu_pause_nosync(svc->vcpu);
                    svc->flags |= VSCHED_FLAG_VCPU_PARKED;
                }

                /* Lower bound on credits */
                if ( credit < -VSCHED_CREDITS_PER_TSLICE )
                {
                    VSCHED_STAT_CRANK(acct_min_credit);
                    credit = -VSCHED_CREDITS_PER_TSLICE;
                    atomic_set(&svc->credit, credit);
                }
            }
            else
            {
                svc->pri = VSCHED_PRI_TS_UNDER;

                /* Unpark any capped domains whose credits go positive */
                if ( svc->flags & VSCHED_FLAG_VCPU_PARKED)
                {
                    /*
                     * It's important to unset the flag AFTER the unpause()
                     * call to make sure the VCPU's priority is not boosted
                     * if it is woken up here.
                     */
                    VSCHED_STAT_CRANK(vcpu_unpark);
                    vcpu_unpause(svc->vcpu);
                    svc->flags &= ~VSCHED_FLAG_VCPU_PARKED;
                }

                // jinho modified: send this guy to idle ??
                /* Upper bound on credits means VCPU stops earning */
                if ( credit > VSCHED_CREDITS_PER_TSLICE )
                {
                    __vsched_vcpu_acct_stop_locked(prv, svc);
                    /* Divide credits in half, so that when it starts
                     * accounting again, it starts a little bit "ahead" */
                    credit /= 2;
                    atomic_set(&svc->credit, credit);
                }
            }

            VSCHED_VCPU_STAT_SET(svc, credit_last, credit);
            VSCHED_VCPU_STAT_SET(svc, credit_incr, credit_fair);
            credit_balance += credit;
        }
    }

    prv->credit_balance = credit_balance;

    spin_unlock_irqrestore(&prv->lock, flags);

    /* Inform each CPU that its runq needs to be sorted */
    prv->runq_sort++;

out:
    set_timer( &prv->master_ticker, NOW() +
            MILLISECS(VSCHED_MSECS_PER_TICK) * VSCHED_TICKS_PER_ACCT );
}

static void
vsched_tick(void *_cpu)
{
    unsigned int cpu = (unsigned long)_cpu;
    struct vsched_pcpu *spc = VSCHED_PCPU(cpu);
    struct vsched_private *prv = VSCHED_PRIV(per_cpu(scheduler, cpu));

    spc->tick++;

    //
    // Accounting for running VCPU
    // 
    if ( !is_idle_vcpu(current) )
        vsched_vcpu_acct(prv, cpu);

    //
    // Check if runq needs to be sorted
    // 
    // Every physical CPU resorts the runq after the accounting master has
    // modified priorities. This is a special O(n) sort and runs at most
    // once per accounting period (currently 30 milliseconds).
    //
    vsched_runq_sort(prv, cpu);

    set_timer(&spc->ticker, NOW() + MILLISECS(VSCHED_MSECS_PER_TICK));
}

#if 0
static struct vsched_vcpu *
vsched_runq_steal(int peer_cpu, int cpu, int pri)
{
#ifdef VDI_SCHEDULE
    const struct vsched_pcpu * const peer_pcpu = VSCHED_PCPU(peer_cpu);
    const struct vcpu * const peer_vcpu = per_cpu(schedule_data, peer_cpu).curr;
    struct vsched_vcpu *speer;
    struct list_head *iter;
    struct vcpu *vc;

    //
    // Don't steal from an idle CPU's runq because it's about to
    // pick up work from it itself.
    //
    if ( peer_pcpu != NULL && !is_idle_vcpu(peer_vcpu) )
    {
        list_for_each( iter, &peer_pcpu->runq )
        {
            speer = __runq_elem(iter);

            //
            // If next available VCPU here is not of strictly higher
            // priority than ours, this PCPU is useless to us.
            //
            if ( speer->pri <= pri )
                break;

            // Is this VCPU is runnable on our PCPU? 
            vc = speer->vcpu;
            BUG_ON( is_idle_vcpu(vc) );

            if (__vsched_vcpu_is_migrateable(vc, cpu))
            {
                // We got a candidate. Grab it! 
                WARN_ON(vc->is_urgent);
                __runq_remove(speer);
                vc->processor = cpu;
                return speer;
            }
        }
    }

    //VSCHED_STAT_CRANK(steal_peer_idle);
    return NULL;
#endif

#ifdef CREDIT_SCHEDULE
    const struct vsched_pcpu * const peer_pcpu = VSCHED_PCPU(peer_cpu);
    const struct vcpu * const peer_vcpu = per_cpu(schedule_data, peer_cpu).curr;
    struct vsched_vcpu *speer;
    struct list_head *iter;
    struct vcpu *vc;

    //
    // Don't steal from an idle CPU's runq because it's about to
    // pick up work from it itself.
    //
    if ( peer_pcpu != NULL && !is_idle_vcpu(peer_vcpu) )
    {
        list_for_each( iter, &peer_pcpu->runq )
        {
            speer = __runq_elem(iter);

            //
            // If next available VCPU here is not of strictly higher
            // priority than ours, this PCPU is useless to us.
            //
            if ( speer->pri <= pri )
                break;

            // Is this VCPU is runnable on our PCPU? 
            vc = speer->vcpu;
            BUG_ON( is_idle_vcpu(vc) );

            if (__vsched_vcpu_is_migrateable(vc, cpu))
            {
                // We got a candidate. Grab it! 
                VSCHED_VCPU_STAT_CRANK(speer, migrate_q);
                VSCHED_STAT_CRANK(migrate_queued);
                WARN_ON(vc->is_urgent);
                __runq_remove(speer);
                vc->processor = cpu;
                return speer;
            }
        }
    }

    VSCHED_STAT_CRANK(steal_peer_idle);
    return NULL;
#endif
}

static struct vsched_vcpu *
vsched_load_balance(struct vsched_private *prv, int cpu,
    struct vsched_vcpu *snext, bool_t *stolen)
{
#ifdef VDI_SCHEDULE
    struct vsched_vcpu *speer;
    cpumask_t workers;
    cpumask_t *online;
    int peer_cpu;

    BUG_ON( cpu != snext->vcpu->processor );
    online = VSCHED_CPUONLINE(per_cpu(cpupool, cpu));

    // If this CPU is going offline we shouldn't steal work. 
    if ( unlikely(!cpu_isset(cpu, *online)) )
        goto out;

    //
    // Peek at non-idling CPUs in the system, starting with our
    // immediate neighbour.
    //
    cpus_andnot(workers, *online, prv->idlers);
    cpu_clear(cpu, workers);
    peer_cpu = cpu;

    while ( !cpus_empty(workers) )
    {
        peer_cpu = cycle_cpu(peer_cpu, workers);
        cpu_clear(peer_cpu, workers);

        //
        // Get ahold of the scheduler lock for this peer CPU.
        // 
        // Note: We don't spin on this lock but simply try it. Spinning could
        // cause a deadlock if the peer CPU is also load balancing and trying
        // to lock this CPU.
        //
        if ( !pcpu_schedule_trylock(peer_cpu) )
        {
            //VSCHED_STAT_CRANK(steal_trylock_failed);
            continue;
        }

        //
        // Any work over there to steal?
        //
        speer = cpu_isset(peer_cpu, *online) ?
            vsched_runq_steal(peer_cpu, cpu, snext->pri) : NULL;
        pcpu_schedule_unlock(peer_cpu);
        if ( speer != NULL )
        {
            *stolen = 1;
            return speer;
        }
    }
#endif

#ifdef CREDIT_SCHEDULE
    struct vsched_vcpu *speer;
    cpumask_t workers;
    cpumask_t *online;
    int peer_cpu;

    BUG_ON( cpu != snext->vcpu->processor );
    online = VSCHED_CPUONLINE(per_cpu(cpupool, cpu));

    // If this CPU is going offline we shouldn't steal work. 
    if ( unlikely(!cpu_isset(cpu, *online)) )
        goto out;

    if ( snext->pri == VSCHED_PRI_IDLE )
        VSCHED_STAT_CRANK(load_balance_idle);
    else if ( snext->pri == VSCHED_PRI_TS_OVER )
        VSCHED_STAT_CRANK(load_balance_over);
    else
        VSCHED_STAT_CRANK(load_balance_other);

    //
    // Peek at non-idling CPUs in the system, starting with our
    // immediate neighbour.
    //
    cpus_andnot(workers, *online, prv->idlers);
    cpu_clear(cpu, workers);
    peer_cpu = cpu;

    while ( !cpus_empty(workers) )
    {
        peer_cpu = cycle_cpu(peer_cpu, workers);
        cpu_clear(peer_cpu, workers);

        //
        // Get ahold of the scheduler lock for this peer CPU.
        // 
        // Note: We don't spin on this lock but simply try it. Spinning could
        // cause a deadlock if the peer CPU is also load balancing and trying
        // to lock this CPU.
        //
        if ( !pcpu_schedule_trylock(peer_cpu) )
        {
            VSCHED_STAT_CRANK(steal_trylock_failed);
            continue;
        }

        //
        // Any work over there to steal?
        //
        speer = cpu_isset(peer_cpu, *online) ?
            vsched_runq_steal(peer_cpu, cpu, snext->pri) : NULL;
        pcpu_schedule_unlock(peer_cpu);
        if ( speer != NULL )
        {
            *stolen = 1;
            return speer;
        }
    }
#endif

 out:
    // Failed to find more important work elsewhere...
    __runq_remove(snext);
    return snext;
}
#endif

/*
 * This function is in the critical path. It is designed to be simple and
 * fast for the common case.
 */
static struct task_slice
vsched_schedule(
    const struct scheduler *ops, s_time_t now, bool_t tasklet_work_scheduled)
{
    const int cpu = smp_processor_id();
    struct list_head * const runq = RUNQ(cpu);
    struct vsched_vcpu * const scurr = VSCHED_VCPU(current);
    struct vsched_private *prv = VSCHED_PRIV(ops);
    struct vsched_vcpu *snext;
    struct task_slice ret;

    VSCHED_STAT_CRANK(schedule);
    VSCHED_VCPU_CHECK(current);

    /*
    if ( !is_idle_vcpu(scurr->vcpu) )
    {
        // Update credits of a non-idle VCPU. 
        burn_credits(scurr, now);
        scurr->start_time -= now;
    }
    else
    {
        // Re-instate a boosted idle VCPU as normal-idle. 
        scurr->pri = VSCHED_PRI_IDLE;
    }
    */

    //
    // Select next runnable local VCPU (ie top of local runq)
    //
    if ( vcpu_runnable(current) )
        __runq_insert(cpu, scurr);
    else
        BUG_ON( is_idle_vcpu(current) || list_empty(runq) );

    //snext = __runq_elem(runq->next);

    snext = updateAndPick(cpu, ops, scurr); // all the utilites will be stored in vcpu
    ret.migrated = 0;

    // Tasklet work (which runs in idle VCPU context) overrides all else.
    /*
    if ( tasklet_work_scheduled )
    {
        snext = VSCHED_VCPU(idle_vcpu[cpu]);
        snext->pri = VSCHED_PRI_TS_BOOST;
    }
    */

    //
    // Clear YIELD flag before scheduling out
    //
    // jinho: this should not be deleted... schedule.c controls this and when __insert_runq...
    /*
    if ( scurr->flags & VSCHED_FLAG_VCPU_YIELD )
        scurr->flags &= ~(VSCHED_FLAG_VCPU_YIELD);
    */

    //
    // SMP Load balance:
    //
    // If the next highest priority local runnable VCPU has already eaten
    // through its credits, look on other PCPUs to see if we have more
    // urgent work... If not, vsched_load_balance() will return snext, but
    // already removed from the runq.
    //

    // jinho: I don't want to get other work to this cpu.. but definitely we need to consider load_balance when multiple vdis are there..
    /*
    if ( snext->pri > VSCHED_PRI_TS_OVER )
        __runq_remove(snext);
    else
        snext = vsched_load_balance(prv, cpu, snext, &ret.migrated);
    */

    __runq_remove(snext);

    // jinho: let's follow the same way.. this is good
    //
    // Update idlers mask if necessary. When we're idling, other CPUs
    // will tickle us when they get extra work.
    //
    //if ( snext->pri == VSCHED_PRI_IDLE )
    if ( !is_idle_vcpu(snext->vcpu) ) // when Idle domain vcpu is selected
    {
        if ( !cpu_isset(cpu, prv->idlers) )
            cpu_set(cpu, prv->idlers);
    }
    else if ( cpu_isset(cpu, prv->idlers) )
    {
        cpu_clear(cpu, prv->idlers);
    }

    // jinho: I don't use this parameter
    /* 
    if ( !is_idle_vcpu(snext->vcpu) )
        snext->start_time += now;
    */

    //
    // Return task to run next...
    //
    /*
    ret.time = (is_idle_vcpu(snext->vcpu) ?
                -1 : MILLISECS(VSCHED_MSECS_PER_TSLICE));
    */

    // jinho: make it more smaller granularity.. check switching overhead..
    ret.time = (is_idle_vcpu(snext->vcpu) ?
//                -1 : MILLISECS(VSCHED_TICKS_PER_TSLICE * VSCHED_MSECS_PER_TICK)); // TODO 30 ms: should be modified if necessary
//                -1 : MILLISECS(VSCHED_MSECS_PER_TICK)); // TODO 10 ms: should be modified if necessary
                -1 : MILLISECS(VSCHED_TICKS_PER_GRANUL)); // TODO 5 ms: should be modified if necessary
//                  -1 : MILLISECS(VSCHED_TICKS_PER_TSLICE)); // TODO 3 ms: should be modified if necessary
//                  -1 : MILLISECS(VSCHED_TICKS_PER_TSLICE)); // TODO 1 ms: should be modified if necessary

    ret.task = snext->vcpu;

    VSCHED_VCPU_CHECK(ret.task);

    return ret;
}

static void
vsched_dump_vcpu(struct vsched_vcpu *svc)
{
    struct vsched_dom * const sdom = svc->sdom;

    printk("[%i.%i] pri=%i flags=%x cpu=%i",
            svc->vcpu->domain->domain_id,
            svc->vcpu->vcpu_id,
            svc->pri,
            svc->flags,
            svc->vcpu->processor);

    if ( sdom )
    {
        printk(" credit=%i [w=%u]", atomic_read(&svc->credit), sdom->weight);
#ifdef VSCHED_STATS
        printk(" (%d+%u) {a/i=%u/%u m=%u+%u}",
                svc->stats.credit_last,
                svc->stats.credit_incr,
                svc->stats.state_active,
                svc->stats.state_idle,
                svc->stats.migrate_q,
                svc->stats.migrate_r);
#endif
    }

    printk("\n");
}

static void
vsched_dump_pcpu(const struct scheduler *ops, int cpu)
{
    struct list_head *runq, *iter;
    struct vsched_pcpu *spc;
    struct vsched_vcpu *svc;
    int loop;
#define cpustr keyhandler_scratch

    spc = VSCHED_PCPU(cpu);
    runq = &spc->runq;

    cpumask_scnprintf(cpustr, sizeof(cpustr), per_cpu(cpu_sibling_map, cpu));
    printk(" sort=%d, sibling=%s, ", spc->runq_sort_last, cpustr);
    cpumask_scnprintf(cpustr, sizeof(cpustr), per_cpu(cpu_core_map, cpu));
    printk("core=%s\n", cpustr);

    /* current VCPU */
    svc = VSCHED_VCPU(per_cpu(schedule_data, cpu).curr);
    if ( svc )
    {
        printk("\trun: ");
        vsched_dump_vcpu(svc);
    }

    loop = 0;
    list_for_each( iter, runq )
    {
        svc = __runq_elem(iter);
        if ( svc )
        {
            printk("\t%3d: ", ++loop);
            vsched_dump_vcpu(svc);
        }
    }
#undef cpustr
}

static void
vsched_dump(const struct scheduler *ops)
{
    struct list_head *iter_sdom, *iter_svc;
    struct vsched_private *prv = VSCHED_PRIV(ops);
    int loop;
#define idlers_buf keyhandler_scratch

    printk("info:\n"
           "\tncpus              = %u\n"
           "\tmaster             = %u\n"
           "\tcredit             = %u\n"
           "\tcredit balance     = %d\n"
           "\tweight             = %u\n"
           "\trunq_sort          = %u\n"
           "\tdefault-weight     = %d\n"
           "\tmsecs per tick     = %dms\n"
           "\tcredits per msec   = %d\n"
           "\tticks per tslice   = %d\n"
           "\tticks per acct     = %d\n"
           "\tmigration delay    = %uus\n",
           prv->ncpus,
           prv->master,
           prv->credit,
           prv->credit_balance,
           prv->weight,
           prv->runq_sort,
           VSCHED_DEFAULT_WEIGHT,
           VSCHED_MSECS_PER_TICK,
           VSCHED_CREDITS_PER_MSEC,
           VSCHED_TICKS_PER_TSLICE,
           VSCHED_TICKS_PER_ACCT,
           vcpu_migration_delay);

    cpumask_scnprintf(idlers_buf, sizeof(idlers_buf), prv->idlers);
    printk("idlers: %s\n", idlers_buf);

    printk("active vcpus:\n");
    loop = 0;
    list_for_each( iter_sdom, &prv->active_sdom )
    {
        struct vsched_dom *sdom;
        sdom = list_entry(iter_sdom, struct vsched_dom, active_sdom_elem);

        list_for_each( iter_svc, &sdom->active_vcpu )
        {
            struct vsched_vcpu *svc;
            svc = list_entry(iter_svc, struct vsched_vcpu, active_vcpu_elem);

            printk("\t%3d: ", ++loop);
            vsched_dump_vcpu(svc);
        }
    }
#undef idlers_buf
}

static int
vsched_init(struct scheduler *ops)
{
    struct vsched_private *prv;

    prv = xmalloc(struct vsched_private);
    if ( prv == NULL )
        return -ENOMEM;

    memset(prv, 0, sizeof(*prv));
    ops->sched_data = prv;
    spin_lock_init(&prv->lock);
    INIT_LIST_HEAD(&prv->active_sdom);
    prv->master = UINT_MAX;

    // jinho added
    memset(&vstat, 0, sizeof(struct vsched_stat));

    return 0;
}

static void
vsched_deinit(const struct scheduler *ops)
{
    struct vsched_private *prv;

    prv = VSCHED_PRIV(ops);
    if ( prv != NULL )
        xfree(prv);
}

static void vsched_tick_suspend(const struct scheduler *ops, unsigned int cpu)
{
    struct vsched_pcpu *spc;

    spc = VSCHED_PCPU(cpu);

    stop_timer(&spc->ticker);
}

static void vsched_tick_resume(const struct scheduler *ops, unsigned int cpu)
{
    struct vsched_pcpu *spc;
    uint64_t now = NOW();

    spc = VSCHED_PCPU(cpu);

    set_timer(&spc->ticker, now + MILLISECS(VSCHED_MSECS_PER_TICK)
            - now % MILLISECS(VSCHED_MSECS_PER_TICK) );
}

static struct vsched_private _vsched_priv;

const struct scheduler sched_vdi_def = {
    .name           = "SMP VDI Scheduler",
    .opt_name       = "vdi",
    .sched_id       = XEN_SCHEDULER_VDI,
    .sched_data     = &_vsched_priv,

    .init_domain    = vsched_dom_init,
    .destroy_domain = vsched_dom_destroy,

    .insert_vcpu    = vsched_vcpu_insert,
    .remove_vcpu    = vsched_vcpu_remove,

    .sleep          = vsched_vcpu_sleep,
    .wake           = vsched_vcpu_wake,
    .yield          = vsched_vcpu_yield,

    .adjust         = vsched_dom_cntl,

    .pick_cpu       = vsched_cpu_pick,
    .do_schedule    = vsched_schedule,

    .dump_cpu_state = vsched_dump_pcpu,
    .dump_settings  = vsched_dump,
    .init           = vsched_init,
    .deinit         = vsched_deinit,
    .alloc_vdata    = vsched_alloc_vdata,
    .free_vdata     = vsched_free_vdata,
    .alloc_pdata    = vsched_alloc_pdata,
    .free_pdata     = vsched_free_pdata,
    .alloc_domdata  = vsched_alloc_domdata,
    .free_domdata   = vsched_free_domdata,

    .tick_suspend   = vsched_tick_suspend,
    .tick_resume    = vsched_tick_resume,
};
