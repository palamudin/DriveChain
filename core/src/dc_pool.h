/* dc_pool.h - persistent worker pool for the step (internal).
   n threads in total: the calling physics thread plus n-1 helpers created once (outside the step, no heap in the
   step). dc_pool_run(fn, ctx, ntasks) runs task k on thread k % n (fixed partition: the same task always lands on the
   same thread index) and returns when all are done. Tasks must write disjoint state, so results are bit-identical
   for any n. The caller's SSE control word (FTZ/DAZ, rounding) is applied in the helpers for every run. */
#ifndef DC_POOL_H
#define DC_POOL_H
typedef void (*dc_task_fn)(void* ctx, int k);
typedef struct dc_pool dc_pool;
dc_pool* dc_pool_create(int nthreads, double spin_us);  /* NULL if nthreads < 2 or thread creation fails */
void     dc_pool_run(dc_pool* p, dc_task_fn fn, void* ctx, int ntasks);
int      dc_pool_threads(const dc_pool* p);
void     dc_pool_destroy(dc_pool* p);
void     dc_pool_prewake(dc_pool* p);                  /* wake sleeping helpers now (they spin again for spin_us): hides the
                                                          condvar wake-up latency when called shortly before the next run */
void     dc_pool_set_spin(dc_pool* p, double spin_us);
int      dc_pool_set_affinity(dc_pool* p, int first_core); /* pin helper i to core first_core + i (-1: unpin); 1 ok */
int      dc_pool_set_cpus(dc_pool* p, const int* cpu, int n, int prio); /* helper i -> cpu[i] (i >= 1, -1 unpinned); prio applied
                                                          by each helper at its next run (-1 leave) */
int      dc_pool_prio_get(const dc_pool* p);           /* AND of the helpers' priority result bits, -1 not applied yet */
#endif
