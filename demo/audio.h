/* audio.h - procedural car audio. No samples: everything is synthesised from the physics state.
   engine: crank-angle-locked firing pulse train (4-cyl, 2 pulses/rev, per-cylinder amplitude & timing jitter)
           exciting exhaust/intake formant resonators, + load-dependent intake noise, + overrun crackle.
   tyres : squeal = narrow-band noise around ~1 kHz driven by brush sliding fraction x load x slide speed;
           structure-borne road noise = the actual high-passed tyre vertical-force signal (kerb ridges,
           bumps, rim strikes are heard because the patch force carries them), + rolling roar ~ speed.
   wind, gearbox whine. Feed it physics frames (1 kHz); it upsamples. Pure C, no allocation in render. */
#ifndef DEMO_AUDIO_H
#define DEMO_AUDIO_H
typedef struct {
    float rpm, throttle, load, speed, clutch, limiter;   /* load: engine torque / max torque, -1..1 */
    int gear, engine_on;
    float fz[4], slide[4], sliding[4], surf[4], rim[4];   /* N, m/s, 0..1, surface id, rim force N */
} audio_frame;
#define AUDIO_RING 512
typedef struct {
    /* SPSC ring of physics frames */
    audio_frame ring[AUDIO_RING];
    volatile unsigned wpos, rpos_i;
    double rpos;                 /* fractional read position in frames */
    double sr, frame_rate;
    float master, engine_gain, tyre_gain, road_gain, wind_gain;
    /* synth state */
    double crank;                /* rad */
    unsigned rng;
    float bq[40][5]; float bz[40][2];  /* biquads: coeffs b0 b1 b2 a1 a2 ; state */
    float pulse_env, pulse_amp, cyl_jit[4];
    int last_cyl;
    float fz_hp[4], fz_prev[4];
    float squeal_lfo[4], crackle;
    float dc[2];
    audio_frame cur;
} audio_t;
void audio_init(audio_t* A, double sample_rate, double frame_rate);
void audio_push(audio_t* A, const audio_frame* f);              /* producer (physics thread) */
void audio_render(audio_t* A, float* out_stereo, int frames);   /* consumer (audio thread) */
int  audio_write_wav(const char* path, const float* stereo, int frames, int sr);
#endif
