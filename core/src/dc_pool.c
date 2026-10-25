#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200112L   /* clock_gettime, pthreads under -std=c11 */
#endif
/* dc_pool.c - see dc_pool.h. Helpers wait on a generation counter: they spin (with a pause hint) for up to spin_us
   after their last job, then sleep on a condition variable, so a 1-2 kHz step loop pays no wake-up latency while an
   idle host does not burn the cores. */
#include "dc_pool.h"
#include <pthread.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <time.h>
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
    dc_task_fn fn; void* ctx; int ntasks; unsigned csr;
    struct { dc_pool* p; int id; } arg[POOL_MAX];
};
static double wall(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + 1e-9 * t.tv_nsec; }
static void* worker(void* a) {
    dc_pool* p = ((struct { dc_pool* p; int id; }*)a)->p; int id = ((struct { dc_pool* p; int id; }*)a)->id;
    unsigned seen = 0;   /* gen is 0 before the helpers exist: reading it here could miss a run dispatched before this thread started */
    for (;;) {
        double t0 = wall(); int it = 0; unsigned g;
        while ((g = atomic_load_explicit(&p->gen, memory_order_acquire)) == seen && !atomic_load_explicit(&p->quit, memory_order_relaxed)) {
            CPU_PAUSE();
            if ((++it & 255) == 0 && wall() - t0 > p->spin_s) {
                pthread_mutex_lock(&p->mu); atomic_fetch_add(&p->sleepers, 1);
                while (atomic_load(&p->gen) == seen && !atomic_load(&p->quit)) pthread_cond_wait(&p->cv, &p->mu);
                atomic_fetch_sub(&p->sleepers, 1); pthread_mutex_unlock(&p->mu);
                t0 = wall();
            }
        }
        if (atomic_load(&p->quit)) return NULL;
        seen = g;
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
    atomic_init(&p->gen, 0); atomic_init(&p->done, 0); atomic_init(&p->sleepers, 0); atomic_init(&p->quit, 0);
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
    if (atomic_load(&p->sleepers) > 0) { pthread_mutex_lock(&p->mu); pthread_cond_broadcast(&p->cv); pthread_mutex_unlock(&p->mu); }
    for (int k = 0; k < ntasks; k += p->n) fn(ctx, k);
    while (atomic_load_explicit(&p->done, memory_order_acquire) < p->n - 1) CPU_PAUSE();
}
void dc_pool_destroy(dc_pool* p) {
    if (!p) return;
    atomic_store(&p->quit, 1);
    pthread_mutex_lock(&p->mu); pthread_cond_broadcast(&p->cv); pthread_mutex_unlock(&p->mu);
    for (int i = 1; i < p->n; i++) pthread_join(p->th[i], NULL);
    pthread_mutex_destroy(&p->mu); pthread_cond_destroy(&p->cv); free(p);
}
