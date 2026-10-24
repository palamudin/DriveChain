#include "audio.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static float frand(audio_t* A) { A->rng = A->rng * 1664525u + 1013904223u; return ((A->rng >> 8) & 0xffff) / 32768.0f - 1.0f; }

/* RBJ biquads */
enum { BQ_LP, BQ_HP, BQ_BP };
static void bq_set(audio_t* A, int i, int type, double f, double q, double sr) {
    if (f > sr * 0.45) f = sr * 0.45; if (f < 10) f = 10;
    double w = 2 * M_PI * f / sr, cs = cos(w), al = sin(w) / (2 * q), a0 = 1 + al, b0, b1, b2;
    if (type == BQ_LP) { b0 = (1 - cs) / 2; b1 = 1 - cs; b2 = b0; }
    else if (type == BQ_HP) { b0 = (1 + cs) / 2; b1 = -(1 + cs); b2 = b0; }
    else { b0 = al; b1 = 0; b2 = -al; }
    A->bq[i][0] = (float)(b0 / a0); A->bq[i][1] = (float)(b1 / a0); A->bq[i][2] = (float)(b2 / a0);
    A->bq[i][3] = (float)(-2 * cs / a0); A->bq[i][4] = (float)((1 - al) / a0);
}
static inline float bq(audio_t* A, int i, float x) {   /* transposed direct form II */
    float* c = A->bq[i]; float* z = A->bz[i];
    float y = c[0] * x + z[0];
    z[0] = c[1] * x - c[3] * y + z[1];
    z[1] = c[2] * x - c[4] * y;
    return y;
}

void audio_init(audio_t* A, double sr, double frame_rate) {
    memset(A, 0, sizeof *A);
    A->sr = sr; A->frame_rate = frame_rate; A->rng = 12345;
    A->master = 0.8f; A->engine_gain = 1; A->tyre_gain = 1; A->road_gain = 1; A->wind_gain = 1;
    A->cur.rpm = 900;
    for (int i = 0; i < 4; i++) A->cyl_jit[i] = 1.0f + 0.12f * (i == 2 ? 1 : (i == 1 ? -0.5f : 0.3f * i));
}
void audio_push(audio_t* A, const audio_frame* f) {
    unsigned w = A->wpos;
    A->ring[w % AUDIO_RING] = *f;
    __sync_synchronize();
    A->wpos = w + 1;
}

static void lerp_frame(audio_frame* o, const audio_frame* a, const audio_frame* b, float t) {
    const float* pa = (const float*)a; const float* pb = (const float*)b; float* po = (float*)o;
    *o = *a;
    o->rpm = a->rpm + (b->rpm - a->rpm) * t;
    for (int i = 0; i < 4; i++) {
        o->fz[i] = a->fz[i] + (b->fz[i] - a->fz[i]) * t;
        o->slide[i] = a->slide[i] + (b->slide[i] - a->slide[i]) * t;
        o->sliding[i] = a->sliding[i] + (b->sliding[i] - a->sliding[i]) * t;
        o->rim[i] = a->rim[i] + (b->rim[i] - a->rim[i]) * t;
    }
    o->speed = a->speed + (b->speed - a->speed) * t;
    o->throttle = a->throttle + (b->throttle - a->throttle) * t;
    o->load = a->load + (b->load - a->load) * t;
    (void)pa; (void)pb; (void)po;
}

/* biquad slots: 0-2 exhaust formants, 3 exhaust LP, 4 intake BP, 5-12 squeal (2 per wheel), 13-16 road HP per wheel,
   17-20 road LP per wheel, 21 roar LP, 22 wind LP, 23 wind HP, 24 whine BP, 25-26 output DC */
void audio_render(audio_t* A, float* out, int frames) {
    const double sr = A->sr, step = A->frame_rate / sr;
    unsigned w = A->wpos;
    /* latency control: keep the read head 6-30 ms behind the writer */
    double avail = (double)w - A->rpos;
    if (avail > 0.030 * A->frame_rate + frames * step) A->rpos = (double)w - 0.012 * A->frame_rate;
    for (int n = 0; n < frames; n++) {
        if ((n & 31) == 0) {
            double rp = A->rpos; unsigned i0 = (unsigned)floor(rp);
            if (w >= 2 && i0 + 1 < w) {
                lerp_frame(&A->cur, &A->ring[i0 % AUDIO_RING], &A->ring[(i0 + 1) % AUDIO_RING], (float)(rp - i0));
                A->rpos += 32 * step;
            } else if (w >= 1) A->cur = A->ring[(w - 1) % AUDIO_RING];
            /* control-rate filter updates */
            const audio_frame* F = &A->cur;
            double fire = F->rpm / 60.0 * 2.0;          /* firing frequency */
            bq_set(A, 0, BQ_BP, 95 + 0.10 * fire, 1.4, sr);     /* exhaust pipe fundamental resonance */
            bq_set(A, 1, BQ_BP, 330 + 0.35 * fire, 2.2, sr);
            bq_set(A, 2, BQ_BP, 1150, 3.0, sr);
            bq_set(A, 3, BQ_LP, 1800 + 2200 * F->throttle, 0.7, sr);
            bq_set(A, 4, BQ_BP, 700 + 0.8 * fire, 1.5, sr);
            for (int i = 0; i < 4; i++) {
                float fl = 1.0f - 0.25f * fminf(F->fz[i] / 6000.0f, 1.5f);
                double f0 = (820 + 160 * (i % 2)) * fl + 90 * A->squeal_lfo[i];
                bq_set(A, 5 + 2 * i, BQ_BP, f0, 14, sr);
                bq_set(A, 6 + 2 * i, BQ_BP, 2.03 * f0, 18, sr);
                bq_set(A, 13 + i, BQ_HP, 18, 0.7, sr);
                bq_set(A, 17 + i, BQ_LP, 260, 0.7, sr);
                A->squeal_lfo[i] = 0.97f * A->squeal_lfo[i] + 0.03f * frand(A) * 3;
            }
            bq_set(A, 21, BQ_LP, 180 + 12 * F->speed, 0.6, sr);
            bq_set(A, 22, BQ_LP, 300 + 40 * F->speed, 0.5, sr);
            bq_set(A, 23, BQ_HP, 120, 0.5, sr);
            double gear_ratio_f = F->rpm / 60.0 * 23.0; /* 23-tooth gear mesh */
            bq_set(A, 24, BQ_BP, gear_ratio_f, 30, sr);
        }
        const audio_frame* F = &A->cur;
        /* ---- engine: crank-locked pulse train ---- */
        float eng = 0;
        if (F->engine_on && F->rpm > 200) {
            double dth = 2 * M_PI * F->rpm / 60.0 / sr;
            double prev = A->crank; A->crank += dth;
            /* 4 firings per 2 revs (720 deg) at 0, 180, 360, 540 deg crank */
            double cyc = fmod(A->crank, 4 * M_PI), pcyc = fmod(prev, 4 * M_PI);
            int cyl = (int)(cyc / M_PI), pcylv = (int)(pcyc / M_PI);
            if (cyl != pcylv) {
                int cut = F->limiter > 0.5f && (A->rng & 1);
                float lo = fmaxf(F->load, 0.0f);
                A->pulse_amp = cut ? 0.05f : (0.25f + 0.95f * lo + 0.25f * F->throttle) * A->cyl_jit[cyl & 3] * (1 + 0.06f * frand(A));
                A->pulse_env = 1;
                if (F->throttle < 0.05f && F->rpm > 3500 && frand(A) > 0.93f) A->crackle = 1;   /* overrun pop */
            }
            /* each pulse: fast-attack exponential blow-down, time constant scales with the firing period */
            double tau = 0.22 / fmax(F->rpm / 60.0 * 2.0, 10.0);
            A->pulse_env *= (float)exp(-1.0 / (tau * sr));
            float exc = A->pulse_env * A->pulse_amp + 0.05f * frand(A) * A->pulse_env;
            float ex = 1.2f * bq(A, 0, exc) + 0.6f * bq(A, 1, exc) + 0.25f * bq(A, 2, exc) + 0.35f * exc;
            ex = bq(A, 3, ex);
            float intake = bq(A, 4, frand(A)) * (0.03f + 0.25f * F->throttle) * fminf(F->rpm / 7000.0f, 1.2f) * (0.4f + A->pulse_env);
            A->crackle *= 0.995f;
            float pop = A->crackle > 0.05f ? A->crackle * frand(A) * 0.5f : 0;
            eng = (ex * 1.6f + intake + pop) * A->engine_gain;
            eng = tanhf(eng * 1.3f);                        /* exhaust/intake nonlinearity */
        }
        /* ---- tyres ---- */
        float tl = 0, tr = 0;
        for (int i = 0; i < 4; i++) {
            float fzn = fminf(F->fz[i] / 3000.0f, 2.5f);
            float sq = F->sliding[i] > 0.15f ? (F->sliding[i] - 0.15f) * fminf(F->slide[i] / 2.0f, 1.5f) * fzn : 0;
            if (F->surf[i] >= 2 && F->surf[i] <= 3) sq *= 0.15f;   /* grass/dirt: no squeal */
            float nz = frand(A);
            float s = (bq(A, 5 + 2 * i, nz) * 1.0f + bq(A, 6 + 2 * i, nz) * 0.35f) * sq * 2.2f * A->tyre_gain;
            /* structure-borne: high-passed vertical force (+ rim strike) */
            float fz = F->fz[i] + 3.0f * F->rim[i];
            float hp = bq(A, 13 + i, fz / 4000.0f);
            float road = bq(A, 17 + i, hp) * 0.9f * A->road_gain;
            float roar = (F->surf[i] >= 2 && F->surf[i] <= 3 ? 0.35f : 0.12f) * fminf(F->speed / 40.0f, 1.5f) * fzn * nz * 0.5f;
            float v = s + road + roar * A->road_gain * 0.4f;
            if (i == 0 || i == 2) { tl += v * 0.75f; tr += v * 0.25f; } else { tl += v * 0.25f; tr += v * 0.75f; }
        }
        float roar_f = bq(A, 21, (tl + tr) * 0.0f + frand(A)) * 0.04f * fminf(F->speed / 30.0f, 2.0f);
        float wind = bq(A, 23, bq(A, 22, frand(A))) * 0.0006f * F->speed * F->speed * A->wind_gain;
        float whine = F->gear != 0 ? bq(A, 24, frand(A)) * 0.02f * fminf(F->rpm / 6000.0f, 1.0f) : 0;
        float mono = eng * 0.55f + roar_f + wind + whine;
        float L = (mono + tl) * A->master, R = (mono + tr) * A->master;
        /* DC block + soft clip */
        A->dc[0] += 0.0005f * (L - A->dc[0]); A->dc[1] += 0.0005f * (R - A->dc[1]);
        out[2 * n] = tanhf(L - A->dc[0]); out[2 * n + 1] = tanhf(R - A->dc[1]);
    }
}

int audio_write_wav(const char* path, const float* st, int frames, int sr) {
    FILE* f = fopen(path, "wb"); if (!f) return 0;
    int data = frames * 4; int hdr[11];
    fwrite("RIFF", 1, 4, f); int v = 36 + data; fwrite(&v, 4, 1, f); fwrite("WAVEfmt ", 1, 8, f);
    v = 16; fwrite(&v, 4, 1, f); short s = 1; fwrite(&s, 2, 1, f); s = 2; fwrite(&s, 2, 1, f);
    fwrite(&sr, 4, 1, f); v = sr * 4; fwrite(&v, 4, 1, f); s = 4; fwrite(&s, 2, 1, f); s = 16; fwrite(&s, 2, 1, f);
    fwrite("data", 1, 4, f); fwrite(&data, 4, 1, f);
    for (int i = 0; i < frames * 2; i++) { float x = st[i]; x = x > 1 ? 1 : (x < -1 ? -1 : x); short q = (short)(x * 32000); fwrite(&q, 2, 1, f); }
    (void)hdr; fclose(f); return 1;
}
