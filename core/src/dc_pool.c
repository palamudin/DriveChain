#if defined(__linux__) && !defined(_GNU_SOURCE)
#define _GNU_SOURCE   /* pthread_setaffinity_np, cpu_set_t */
#endif
#if defined(_WIN32) && (!defined(_WIN32_WINNT) || _WIN32_WINNT < 0x0601)
#undef _WIN32_WINNT
#define _WIN32_WINNT 0x0601   /* GetLogicalProcessorInformationEx */
#endif
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200112L   /* clock_gettime, pthreads under -std=c11 */
#endif
/* dc_pool.c - see dc_pool.h. Helpers wait on a generation counter: they spin (with a pause hint) for up to spin_us
   after their last job, then sleep on a condition variable, so a 1-2 kHz step loop pays no wake-up latency while an
   idle host does not burn the cores. */
#include "dc_pool.h"
#include "dc_prof.h"
#include "drivechain.h"
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <time.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <sched.h>
#include <unistd.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#endif
#if defined(__SSE__) || defined(_M_X64)
#include <xmmintrin.h>
#define HAVE_CSR 1
#define CPU_PAUSE() _mm_pause()
#else
#define CPU_PAUSE() ((void)0)
#endif
#define POOL_MAX 16
struct dc_pool {
    int n; double spin_s;
    pthread_t th[POOL_MAX];
    pthread_mutex_t mu; pthread_cond_t cv;
    atomic_uint gen;          /* bumped per run */
    atomic_int done;          /* helpers finished in this run */
    atomic_int sleepers;
    atomic_int quit;
    atomic_uint poke;         /* bumped by dc_pool_prewake: sleeping helpers return to spinning */
    dc_task_fn fn; void* ctx; int ntasks; unsigned csr;
    atomic_int prio_want;     /* -1: leave the helpers' priority alone */
    atomic_int prio_res[POOL_MAX];
    struct { dc_pool* p; int id; } arg[POOL_MAX];
};
static int set_prio(int prio, void** mm);
static double wall(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + 1e-9 * t.tv_nsec; }
static void* worker(void* a) {
    dc_pool* p = ((struct { dc_pool* p; int id; }*)a)->p; int id = ((struct { dc_pool* p; int id; }*)a)->id;
    int prio_have = -1; void* mm = NULL;
    unsigned seen = 0;   /* gen is 0 before the helpers exist: reading it here could miss a run dispatched before this thread started */
    for (;;) {
        double t0 = wall(); int it = 0; unsigned g;
        while ((g = atomic_load_explicit(&p->gen, memory_order_acquire)) == seen && !atomic_load_explicit(&p->quit, memory_order_relaxed)) {
            CPU_PAUSE();
            if ((++it & 255) == 0 && wall() - t0 > p->spin_s) {
                pthread_mutex_lock(&p->mu); atomic_fetch_add(&p->sleepers, 1);
                unsigned pk = atomic_load(&p->poke);
                while (atomic_load(&p->gen) == seen && !atomic_load(&p->quit) && atomic_load(&p->poke) == pk) pthread_cond_wait(&p->cv, &p->mu);
                atomic_fetch_sub(&p->sleepers, 1); pthread_mutex_unlock(&p->mu);
                t0 = wall();
            }
        }
        if (atomic_load(&p->quit)) { if (mm) set_prio(0, &mm); return NULL; }
        seen = g;
        { int pw = atomic_load_explicit(&p->prio_want, memory_order_relaxed);
          if (pw >= 0 && pw != prio_have) { prio_have = pw; atomic_store(&p->prio_res[id], set_prio(pw, &mm)); } }
#ifdef HAVE_CSR
        if (_mm_getcsr() != p->csr) _mm_setcsr(p->csr);
#endif
        for (int k = id; k < p->ntasks; k += p->n) p->fn(p->ctx, k);
        atomic_fetch_add_explicit(&p->done, 1, memory_order_release);
    }
}
dc_pool* dc_pool_create(int n, double spin_us) {
    if (n < 2) return NULL; if (n > POOL_MAX) n = POOL_MAX;
    dc_pool* p = (dc_pool*)calloc(1, sizeof *p); if (!p) return NULL;
    p->n = n; p->spin_s = (spin_us > 0 ? spin_us : 500) * 1e-6;
    pthread_mutex_init(&p->mu, NULL); pthread_cond_init(&p->cv, NULL);
    atomic_init(&p->prio_want, -1); for (int i = 0; i < POOL_MAX; i++) atomic_init(&p->prio_res[i], -1);
    atomic_init(&p->gen, 0); atomic_init(&p->done, 0); atomic_init(&p->sleepers, 0); atomic_init(&p->quit, 0); atomic_init(&p->poke, 0);
    for (int i = 1; i < n; i++) {
        p->arg[i].p = p; p->arg[i].id = i;
        if (pthread_create(&p->th[i], NULL, worker, &p->arg[i]) != 0) { p->n = i; dc_pool_destroy(p); return NULL; }
    }
    return p;
}
int dc_pool_threads(const dc_pool* p) { return p ? p->n : 1; }
void dc_pool_run(dc_pool* p, dc_task_fn fn, void* ctx, int ntasks) {
    if (!p) { for (int k = 0; k < ntasks; k++) fn(ctx, k); return; }
    p->fn = fn; p->ctx = ctx; p->ntasks = ntasks;
#ifdef HAVE_CSR
    p->csr = _mm_getcsr();
#endif
    atomic_store_explicit(&p->done, 0, memory_order_relaxed);
    atomic_fetch_add(&p->gen, 1);   /* seq_cst: pairs with the helper's sleepers++ then gen load (no lost wake-up) */
    if (atomic_load(&p->sleepers) > 0) { DCP_CNT(pool_wakes, 1); pthread_mutex_lock(&p->mu); pthread_cond_broadcast(&p->cv); pthread_mutex_unlock(&p->mu); }
    for (int k = 0; k < ntasks; k += p->n) fn(ctx, k);
    DCP_T(tw); DCP_CNT(pool_runs, 1);
    while (atomic_load_explicit(&p->done, memory_order_acquire) < p->n - 1) CPU_PAUSE();
    DCP(double dw = dc_wall() - tw; dc_prof_g.pool_wait_s += dw; if (dw > dc_prof_g.pool_wait_max_s) dc_prof_g.pool_wait_max_s = dw;)
}
void dc_pool_prewake(dc_pool* p) {
    if (!p || atomic_load(&p->sleepers) == 0) return;
    DCP_CNT(pool_prewakes, 1);
    pthread_mutex_lock(&p->mu); atomic_fetch_add(&p->poke, 1); pthread_cond_broadcast(&p->cv); pthread_mutex_unlock(&p->mu);
}
void dc_pool_set_spin(dc_pool* p, double spin_us) { if (p && spin_us > 0) p->spin_s = spin_us * 1e-6; }
int dc_pool_set_affinity(dc_pool* p, int first) {
    if (!p) return 0; int ok = 1;
    for (int i = 1; i < p->n; i++) {
#ifdef _WIN32
        HANDLE h = pthread_gethandle(p->th[i]);
        DWORD_PTR m = first < 0 ? (DWORD_PTR)-1 : ((DWORD_PTR)1 << ((first + i) % (8 * (int)sizeof(DWORD_PTR))));
        if (first < 0) { DWORD_PTR pm, sm; if (GetProcessAffinityMask(GetCurrentProcess(), &pm, &sm)) m = pm; }
        ok &= SetThreadAffinityMask(h, m) != 0;
#elif defined(__linux__)
        cpu_set_t cs; CPU_ZERO(&cs);
        if (first < 0) { for (int c = 0; c < CPU_SETSIZE; c++) CPU_SET(c, &cs); } else CPU_SET(first + i, &cs);
        ok &= pthread_setaffinity_np(p->th[i], sizeof cs, &cs) == 0;
#else
        (void)first; ok = 0;
#endif
    }
    return ok;
}
void dc_pool_destroy(dc_pool* p) {
    if (!p) return;
    atomic_store(&p->quit, 1);
    pthread_mutex_lock(&p->mu); pthread_cond_broadcast(&p->cv); pthread_mutex_unlock(&p->mu);
    for (int i = 1; i < p->n; i++) pthread_join(p->th[i], NULL);
    pthread_mutex_destroy(&p->mu); pthread_cond_destroy(&p->cv); free(p);
}

/* ------------------------------------------------------------------ thread placement (see drivechain.h) */
#define AFF_MAXL 256   /* logical CPUs considered (Windows: the current processor group) */
typedef struct {
    int n;                            /* allowed logical CPUs */
    int cpu[AFF_MAXL], core[AFF_MAXL], dom[AFF_MAXL], eff[AFF_MAXL];   /* per allowed CPU: physical core id, L3 domain, efficiency class */
    int ndom; double dom_kb[AFF_MAXL];
    int ok;
} aff_topo;
static int parse_list(const char* s, int* out, int max) {   /* "0-3,8,10-11" */
    int n = 0;
    while (s && *s && n < max) {
        while (*s == ' ' || *s == ',' || *s == '\n') s++;
        if (*s < '0' || *s > '9') break;
        char* e; long a = strtol(s, &e, 10), b = a; s = e;
        if (*s == '-') { s++; b = strtol(s, &e, 10); s = e; }
        for (long c = a; c <= b && n < max; c++) out[n++] = (int)c;
    }
    return n;
}
#if defined(__linux__)
static int rd(const char* path, char* b, int n) {
    FILE* f = fopen(path, "r"); if (!f) return 0;
    int k = (int)fread(b, 1, (size_t)n - 1, f); fclose(f); b[k > 0 ? k : 0] = 0; return k > 0;
}
#ifdef __linux__
/* the process's CPU mask as it was before this library pinned any thread (taskset / cgroup limits are kept; a thread
   pinned earlier must not shrink later plans, and "unpinned" restores this mask, not all CPUs) */
static int proc_mask(cpu_set_t* out) {
    static cpu_set_t m; static int st;   /* 0 unread, 1 ok, -1 failed; first call from the configuring thread */
    if (!st) st = sched_getaffinity(0, sizeof m, &m) == 0 ? 1 : -1;
    if (st > 0) *out = m;
    return st > 0;
}
#endif
static void topo_read(aff_topo* T) {
    memset(T, 0, sizeof *T);
    cpu_set_t cs; CPU_ZERO(&cs);
    const char* R = getenv("DC_AFF_SYSFS");   /* test hook: a copied/synthetic sysfs cpu tree (all its CPUs allowed) */
    char path[200], b[512];
    if (R) { snprintf(path, sizeof path, "%s/online", R); int l[AFF_MAXL], n = rd(path, b, sizeof b) ? parse_list(b, l, AFF_MAXL) : 0; for (int k = 0; k < n; k++) CPU_SET(l[k], &cs); }
    else { R = "/sys/devices/system/cpu"; if (!proc_mask(&cs)) return; }
    int domfirst[AFF_MAXL], ndf = 0;
    for (int c = 0; c < CPU_SETSIZE && c < AFF_MAXL && T->n < AFF_MAXL; c++) {
        if (!CPU_ISSET(c, &cs)) continue;
        int i = T->n++;
        T->cpu[i] = c; T->core[i] = c; T->dom[i] = 0; T->eff[i] = 0;
        int pkg = 0, cid = c;
        snprintf(path, sizeof path, "%s/cpu%d/topology/physical_package_id", R, c); if (rd(path, b, sizeof b)) pkg = atoi(b);
        snprintf(path, sizeof path, "%s/cpu%d/topology/core_id", R, c); if (rd(path, b, sizeof b)) cid = atoi(b);
        T->core[i] = pkg * 65536 + cid;
        snprintf(path, sizeof path, "%s/cpu%d/cpu_capacity", R, c); if (rd(path, b, sizeof b)) T->eff[i] = atoi(b);
        int first = -1; double kb = 0;
        for (int k = 0; k < 8; k++) {
            snprintf(path, sizeof path, "%s/cpu%d/cache/index%d/level", R, c, k); if (!rd(path, b, sizeof b)) break;
            if (atoi(b) != 3) continue;
            snprintf(path, sizeof path, "%s/cpu%d/cache/index%d/size", R, c, k);
            if (rd(path, b, sizeof b)) { kb = atof(b); if (strchr(b, 'M')) kb *= 1024; }
            snprintf(path, sizeof path, "%s/cpu%d/cache/index%d/shared_cpu_list", R, c, k);
            int l[AFF_MAXL]; if (rd(path, b, sizeof b) && parse_list(b, l, AFF_MAXL) > 0) first = l[0];
            break;
        }
        if (first < 0) first = 100000 + pkg;   /* no L3 info: one domain per package */
        int d = 0; while (d < ndf && domfirst[d] != first) d++;
        if (d == ndf) { domfirst[ndf++] = first; T->dom_kb[d] = kb; }
        T->dom[i] = d;
    }
    T->ndom = ndf; T->ok = T->n > 0;
}
#elif defined(_WIN32)
static void topo_read(aff_topo* T) {
    memset(T, 0, sizeof *T);
    DWORD_PTR pm = 0, sm = 0; if (!GetProcessAffinityMask(GetCurrentProcess(), &pm, &sm) || !pm) return;
    DWORD len = 0; GetLogicalProcessorInformationEx(RelationAll, NULL, &len);
    if (!len) return;
    char* buf = (char*)malloc(len); if (!buf) return;
    if (!GetLogicalProcessorInformationEx(RelationAll, (PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX)buf, &len)) { free(buf); return; }
    WORD grp = 0; { PROCESSOR_NUMBER pn; GetCurrentProcessorNumberEx(&pn); grp = pn.Group; }
    const int B = 8 * (int)sizeof(DWORD_PTR);
    int coreof[64], effof[64], domof[64]; for (int c = 0; c < 64; c++) { coreof[c] = c; effof[c] = 0; domof[c] = -1; }
    int nd = 0; KAFFINITY dmask[AFF_MAXL];
    for (DWORD o = 0, k = 0; o < len; k++) {
        PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX e = (PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX)(buf + o);
        if (e->Relationship == RelationProcessorCore && e->Processor.GroupMask[0].Group == grp) {
            KAFFINITY m = e->Processor.GroupMask[0].Mask;
            for (int c = 0; c < B && c < 64; c++) if (m >> c & 1) { coreof[c] = (int)k; effof[c] = e->Processor.EfficiencyClass; }
        } else if (e->Relationship == RelationCache && e->Cache.Level == 3 && e->Cache.GroupMask.Group == grp && nd < AFF_MAXL) {
            dmask[nd] = e->Cache.GroupMask.Mask; T->dom_kb[nd] = e->Cache.CacheSize / 1024.0;
            for (int c = 0; c < B && c < 64; c++) if (dmask[nd] >> c & 1) domof[c] = nd;
            nd++;
        }
        o += e->Size; if (!e->Size) break;
    }
    free(buf);
    if (nd == 0) { nd = 1; T->dom_kb[0] = 0; }
    for (int c = 0; c < B && c < 64; c++) {
        if (!(pm >> c & 1)) continue;
        int i = T->n++; T->cpu[i] = c; T->core[i] = coreof[c]; T->eff[i] = effof[c]; T->dom[i] = domof[c] < 0 ? 0 : domof[c];
    }
    T->ndom = nd; T->ok = T->n > 0;
}
#else
static void topo_read(aff_topo* T) { memset(T, 0, sizeof *T); }
#endif

/* physical cores of domain d (allowed, optionally only the fastest efficiency class): first allowed logical CPU of each
   core in CPU order, and with sib = 1 the remaining SMT siblings appended after them */
static int dom_cpus(const aff_topo* T, int d, int effmin, int sib, int* out) {
    int n = 0, seen[AFF_MAXL], ns = 0;
    for (int i = 0; i < T->n; i++) {
        if (T->dom[i] != d || T->eff[i] < effmin) continue;
        int dup = 0; for (int k = 0; k < ns; k++) if (seen[k] == T->core[i]) dup = 1;
        if (!dup) { seen[ns++] = T->core[i]; out[n++] = T->cpu[i]; }
    }
    if (sib) for (int i = 0; i < T->n; i++) {
        if (T->dom[i] != d || T->eff[i] < effmin) continue;
        int in = 0; for (int k = 0; k < n; k++) if (out[k] == T->cpu[i]) in = 1;
        if (!in) out[n++] = T->cpu[i];
    }
    return n;
}
DC_API int dc_affinity_plan(const char* spec, int nt, dc_affinity_info* o) {
    memset(o, 0, sizeof *o);
    if (nt < 1) nt = 1; if (nt > DC_AFF_MAXT) nt = DC_AFF_MAXT;
    o->nthreads = nt; o->dom = -1; for (int t = 0; t < DC_AFF_MAXT; t++) o->cpu[t] = -1;
    static aff_topo T; topo_read(&T);   /* host-side, outside the step */
    o->topo_ok = T.ok; o->nlogical = T.n; o->ndom = T.ndom;
    { int seen[AFF_MAXL], ns = 0; for (int i = 0; i < T.n; i++) { int dup = 0; for (int k = 0; k < ns; k++) if (seen[k] == T.core[i]) dup = 1; if (!dup) seen[ns++] = T.core[i]; } o->ncores = ns; }
    int effmax = 0; for (int i = 0; i < T.n; i++) if (T.eff[i] > effmax) effmax = T.eff[i];
    for (int d = 0; d < T.ndom && d < 8; d++) { int tmp[AFF_MAXL]; o->l3_mb[d] = T.dom_kb[d] / 1024.0; o->dom_cores[d] = dom_cpus(&T, d, 0, 0, tmp); }
    while (spec && *spec == ' ') spec++;
    if (!spec || !*spec || !strcmp(spec, "off") || !strcmp(spec, "-1") || !strcmp(spec, "none")) { snprintf(o->why, sizeof o->why, "off"); return 0; }
    if (!strcmp(spec, "auto") || !strcmp(spec, "vcache")) {
        int vc = spec[0] == 'v'; o->mode = vc ? 2 : 1;
        if (!T.ok) { snprintf(o->why, sizeof o->why, "topology not available, not pinned"); return 0; }
        int best = -1, bestn = 0, list[AFF_MAXL], effuse = effmax;
        for (int pass = 0; pass < 2 && best < 0; pass++) {   /* pass 0: fastest efficiency class only; pass 1: any core */
            effuse = pass ? 0 : effmax;
            for (int d = 0; d < T.ndom; d++) {
                int n = dom_cpus(&T, d, effuse, 0, list);
                if (!vc && n < nt) continue;
                if (vc && n == 0) continue;
                if (best < 0 || T.dom_kb[d] > T.dom_kb[best] || (T.dom_kb[d] == T.dom_kb[best] && n > bestn)) { best = d; bestn = n; }
            }
            if (vc) break;
        }
        if (best < 0) { snprintf(o->why, sizeof o->why, "no L3 domain with %d free physical cores, not pinned", nt); return 0; }
        int n = dom_cpus(&T, best, effuse, vc, list), k0 = 0;
        int ncore = dom_cpus(&T, best, effuse, 0, (int[AFF_MAXL]){0});
        if (list[0] == 0 && ncore > nt) k0 = 1;   /* leave CPU 0 (interrupts, the OS) to others when a spare core exists */
        for (int t = 0; t < nt && k0 + t < n; t++) o->cpu[t] = list[k0 + t];
        o->dom = best; o->dom_l3_mb = T.dom_kb[best] / 1024.0;
        snprintf(o->why, sizeof o->why, "%s: L3 domain %d of %d (%.0f MB)%s", vc ? "vcache" : "auto", best, T.ndom, o->dom_l3_mb, k0 ? ", CPU 0 skipped" : "");
    } else {
        int l[DC_AFF_MAXT * 4], n = parse_list(spec, l, DC_AFF_MAXT * 4);
        if (n == 0) { snprintf(o->why, sizeof o->why, "unrecognised '%.40s', not pinned", spec); return 0; }
        int legacy = n == 1 && !strpbrk(spec, ",-");
        o->mode = legacy ? 4 : 3;
        int bad = 0;
        for (int t = 0; t < nt; t++) {
            int c = legacy ? l[0] + t : (t < n ? l[t] : -1), ok = c < 0;
            for (int i = 0; i < T.n && !ok; i++) if (T.cpu[i] == c) ok = 1;
            if (!ok && T.ok) { c = -1; bad++; }   /* not in the process mask / not online: leave that thread unpinned */
            o->cpu[t] = c;
        }
        if (legacy) snprintf(o->why, sizeof o->why, "first CPU %d (legacy)%s", l[0], bad ? ", some CPUs not available" : "");
        else snprintf(o->why, sizeof o->why, "CPU list%s", bad ? ", some CPUs not available" : "");
        /* the domain of the first CPU, for the display */
        for (int i = 0; i < T.n; i++) if (T.cpu[i] == o->cpu[0]) { o->dom = T.dom[i]; o->dom_l3_mb = T.dom_kb[T.dom[i]] / 1024.0; }
    }
    for (int t = 0; t < nt; t++) if (o->cpu[t] >= 0) {
        o->pinned++;
        int ct = -1; for (int i = 0; i < T.n; i++) if (T.cpu[i] == o->cpu[t]) ct = T.core[i];
        for (int u = 0; u < t; u++) if (o->cpu[u] >= 0) { int cu = -2; for (int i = 0; i < T.n; i++) if (T.cpu[i] == o->cpu[u]) cu = T.core[i]; if (cu == ct) o->smt_shared = 1; }
    }
    return o->pinned > 0;
}
DC_API int dc_affinity_parse_prio(const char* s) {
    if (!s) return 0;
    if (!strcmp(s, "high") || !strcmp(s, "1")) return 1;
    if (!strcmp(s, "mmcss") || !strcmp(s, "2")) return 2;
    return 0;
}
DC_API const char* dc_affinity_prio_name(int p) { return p == 1 ? "high" : p == 2 ? "mmcss" : "normal"; }

static int set_affinity_self(int cpu) {
#ifdef _WIN32
    DWORD_PTR m = 0, pm, sm;
    if (cpu < 0) { if (!GetProcessAffinityMask(GetCurrentProcess(), &pm, &sm)) return 0; m = pm; }
    else if (cpu < 8 * (int)sizeof(DWORD_PTR)) m = (DWORD_PTR)1 << cpu; else return 0;
    return SetThreadAffinityMask(GetCurrentThread(), m) != 0;
#elif defined(__linux__)
    cpu_set_t cs; CPU_ZERO(&cs);
    if (cpu < 0) { if (!proc_mask(&cs)) for (int c = 0; c < CPU_SETSIZE; c++) CPU_SET(c, &cs); } else { if (cpu >= CPU_SETSIZE) return 0; CPU_SET(cpu, &cs); }
    return pthread_setaffinity_np(pthread_self(), sizeof cs, &cs) == 0;
#else
    (void)cpu; return 0;
#endif
}
/* priority of the calling thread; *mm keeps the MMCSS registration (Windows). Returns bit 1 = ok, bit 2 = mmcss */
static int set_prio(int prio, void** mm) {
#ifdef _WIN32
    typedef HANDLE (WINAPI *avset_t)(LPCWSTR, LPDWORD); typedef BOOL (WINAPI *avrev_t)(HANDLE);
    static HMODULE av; static avset_t avset; static avrev_t avrev; static int av_tried;
    if (!av_tried) { av_tried = 1; av = LoadLibraryA("avrt.dll"); if (av) { avset = (avset_t)(void*)GetProcAddress(av, "AvSetMmThreadCharacteristicsW"); avrev = (avrev_t)(void*)GetProcAddress(av, "AvRevertMmThreadCharacteristics"); } }
    if (*mm && avrev) { avrev((HANDLE)*mm); *mm = NULL; }
    if (prio == 2 && avset) { DWORD idx = 0; HANDLE h = avset(L"Games", &idx); if (h) { *mm = (void*)h; return 2 | 4; } }
    return SetThreadPriority(GetCurrentThread(), prio >= 1 ? THREAD_PRIORITY_HIGHEST : THREAD_PRIORITY_NORMAL) ? 2 : 0;
#elif defined(__linux__)
    (void)mm;
    pid_t tid = (pid_t)syscall(SYS_gettid);
    return setpriority(PRIO_PROCESS, (id_t)tid, prio >= 1 ? -5 : 0) == 0 ? 2 : 0;
#else
    (void)mm; return prio == 0 ? 2 : 0;
#endif
}
DC_API int dc_pin_thread(int cpu, int prio) {
    static _Thread_local void* mm;
    int r = set_affinity_self(cpu) ? 1 : 0;
    if (prio >= 0) r |= set_prio(prio, &mm);
    return r;
}
int dc_pool_set_cpus(dc_pool* p, const int* cpu, int n, int prio) {
    if (!p) return 0; int ok = 1;
    for (int i = 1; i < p->n; i++) {
        int c = i < n ? cpu[i] : -1;
#ifdef _WIN32
        HANDLE h = pthread_gethandle(p->th[i]);
        DWORD_PTR m, pm, sm;
        if (c < 0 || c >= 8 * (int)sizeof(DWORD_PTR)) { m = GetProcessAffinityMask(GetCurrentProcess(), &pm, &sm) ? pm : (DWORD_PTR)-1; }
        else m = (DWORD_PTR)1 << c;
        ok &= SetThreadAffinityMask(h, m) != 0;
#elif defined(__linux__)
        cpu_set_t cs; CPU_ZERO(&cs);
        if (c < 0) { if (!proc_mask(&cs)) for (int k = 0; k < CPU_SETSIZE; k++) CPU_SET(k, &cs); } else CPU_SET(c, &cs);
        ok &= pthread_setaffinity_np(p->th[i], sizeof cs, &cs) == 0;
#else
        (void)c; ok = 0;
#endif
    }
    for (int i = 0; i < POOL_MAX; i++) atomic_store(&p->prio_res[i], -1);
    atomic_store(&p->prio_want, prio);
    return ok;
}
int dc_pool_prio_get(const dc_pool* p) {
    if (!p) return -1; int r = 6;
    for (int i = 1; i < p->n; i++) { int v = atomic_load(&((dc_pool*)p)->prio_res[i]); if (v < 0) return -1; r &= v; }
    return r;
}
