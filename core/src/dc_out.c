/* dc_out.c - outputs: telemetry (incl. instantaneous kingpin geometry), part transforms, linkage segments. */
#include "dc_internal.h"

static qt quat_of(const m3* R) { return mtoq(R); }

static void kingpin(const dc_car* c, int i, dc_wheel_telem* W) {
    const dc_kin* k = &c->K[i]; const dc_pose* p = &c->pose[i];
    v3 om, vv;
    if (k->steered) { om = p->Jw[1]; vv = vsub(p->Jp[1], vcross(om, p->c)); }  /* velocity of the frame origin */
    else if (k->type != DC_SUSP_STRUT && k->nl >= 4) {   /* non-steered: axis through the ball joints UBJ-LBJ */
        v3 lbj = vadd(p->c, mv(&p->R, k->lu[k->arm_a >= 0 ? k->arm_a : 0]));
        v3 ubj = vadd(p->c, mv(&p->R, k->lu[0]));
        om = vsub(ubj, lbj); vv = vcross(lbj, om);   /* screw through lbj: v = -om x lbj */
        vv = vneg(vcross(om, lbj));
    } else { W->caster = W->kpi = W->scrub = W->trail = 0; return; }
    double n2 = vdot(om, om); if (n2 < 1e-18) return;
    v3 kax = vscl(om, 1 / sqrt(n2));
    v3 p0 = vscl(vcross(om, vv), 1 / n2);
    if (kax.z < 0) kax = vneg(kax);
    W->caster = atan2(-kax.x, kax.z);
    W->kpi = (i & 1) ? atan2(kax.y, kax.z) : atan2(-kax.y, kax.z);
    /* ground point: contact point in chassis frame */
    m3 R = qtom(c->q);
    v3 pc = mtv(&R, vsub(V3(W->contact[0], W->contact[1], W->contact[2]), c->x));
    if (W->fz <= 0) { const dc_tyre_params* T = &c->P.axle[i / 2].tyre; pc = vsub(p->c, V3(0, 0, T->radius)); }
    double t = (pc.z - p0.z) / kax.z;
    v3 gp = vmad(p0, kax, t);
    W->scrub = (i & 1) ? gp.y - pc.y : pc.y - gp.y;
    W->trail = gp.x - pc.x;
}

DC_API void dc_get_telemetry(const dc_car* c, dc_telemetry* t) {
    *t = c->tel;
    for (int i = 0; i < 4; i++) kingpin(c, i, &t->w[i]);
}

static v3 sw_point(const dc_car* c) {
    return V3(-0.20 - c->P.sprung_cg[0], 0.37 - c->P.sprung_cg[1], 0.30 - c->P.sprung_cg[2]);
}

DC_API int dc_get_parts(const dc_car* c, dc_xform* out, int max) {
    if (max < DC_PART_COUNT) return 0;
    m3 R = qtom(c->q);
    dc_xform* o = &out[DC_PART_CHASSIS];
    o->pos[0] = c->x.x; o->pos[1] = c->x.y; o->pos[2] = c->x.z;
    o->quat[0] = c->q.w; o->quat[1] = c->q.x; o->quat[2] = c->q.y; o->quat[3] = c->q.z;
    for (int i = 0; i < 4; i++) {
        const dc_pose* p = &c->pose[i]; const dc_kin* k = &c->K[i];
        v3 pw = vadd(c->x, mv(&R, p->c));
        m3 Ru = mmul(&R, &p->R);
        qt qu = quat_of(&Ru);
        o = &out[DC_PART_UPRIGHT0 + i];
        o->pos[0] = pw.x; o->pos[1] = pw.y; o->pos[2] = pw.z;
        o->quat[0] = qu.w; o->quat[1] = qu.x; o->quat[2] = qu.y; o->quat[3] = qu.z;
        /* wheel = upright frame, aligned so local +y is the spindle axis, then spun by the angle */
        v3 a = k->spin_axis_u, y = V3(0, 1, 0);
        v3 ax = vcross(y, a); double sn = vlen(ax), cs = vdot(y, a);
        m3 Ra = sn > 1e-12 ? mexp(vscl(ax, atan2(sn, cs) / sn)) : m3id();
        m3 Rs = mexp(V3(0, c->ty[i].angle, 0));
        m3 t1 = mmul(&Ru, &Ra), Rw = mmul(&t1, &Rs);
        qt qw = quat_of(&Rw);
        o = &out[DC_PART_WHEEL0 + i];
        o->pos[0] = pw.x; o->pos[1] = pw.y; o->pos[2] = pw.z;
        o->quat[0] = qw.w; o->quat[1] = qw.x; o->quat[2] = qw.y; o->quat[3] = qw.z;
    }
    {
        const dc_kin* k = &c->K[0];
        v3 rp = k->rack_link >= 0 ? k->lc[k->rack_link] : V3(k->hub0.x, 0, k->hub0.z);
        rp.y = c->r;
        v3 pw = vadd(c->x, mv(&R, rp));
        o = &out[DC_PART_RACK];
        o->pos[0] = pw.x; o->pos[1] = pw.y; o->pos[2] = pw.z;
        o->quat[0] = c->q.w; o->quat[1] = c->q.x; o->quat[2] = c->q.y; o->quat[3] = c->q.z;
    }
    {
        v3 pw = vadd(c->x, mv(&R, sw_point(c)));
        m3 Rc = mexp(vscl(c->col_axis, c->th));
        m3 Rsw = mmul(&R, &Rc);
        qt q = quat_of(&Rsw);
        o = &out[DC_PART_STEERWHEEL];
        o->pos[0] = pw.x; o->pos[1] = pw.y; o->pos[2] = pw.z;
        o->quat[0] = q.w; o->quat[1] = q.x; o->quat[2] = q.y; o->quat[3] = q.z;
    }
    return DC_PART_COUNT;
}

static void put(dc_seg* s, v3 a, v3 b, int type, int corner) {
    s->a[0] = a.x; s->a[1] = a.y; s->a[2] = a.z; s->b[0] = b.x; s->b[1] = b.y; s->b[2] = b.z; s->type = type; s->corner = corner;
}

DC_API int dc_get_segments(const dc_car* c, dc_seg* out, int max) {
    m3 R = qtom(c->q); int n = 0;
    for (int i = 0; i < 4; i++) {
        const dc_pose* p = &c->pose[i]; const dc_kin* k = &c->K[i];
        for (int l = 0; l < k->nl && n < max; l++) {
            v3 Qc = k->lc[l]; if (l == k->rack_link && k->steered) Qc.y += c->r;
            v3 a = vadd(c->x, mv(&R, Qc));
            v3 b = vadd(c->x, mv(&R, vadd(p->c, mv(&p->R, k->lu[l]))));
            put(&out[n++], a, b, l == k->rack_link ? DC_SEG_TIEROD : DC_SEG_LINK, i);
        }
        if (n < max) put(&out[n++], vadd(c->x, mv(&R, p->lo)), vadd(c->x, mv(&R, k->spr_hi)), DC_SEG_SPRING, i);
        if (k->type == DC_SUSP_STRUT && n < max)
            put(&out[n++], vadd(c->x, mv(&R, k->strut_top)), vadd(c->x, mv(&R, vadd(p->c, mv(&p->R, k->strut_S)))), DC_SEG_STRUT, i);
    }
    /* rack and steering column */
    if (c->K[0].steered && c->K[0].rack_link >= 0 && n + 2 <= max) {
        v3 l = c->K[0].lc[c->K[0].rack_link], r = c->K[1].lc[c->K[1].rack_link];
        l.y += c->r; r.y += c->r;
        put(&out[n++], vadd(c->x, mv(&R, l)), vadd(c->x, mv(&R, r)), DC_SEG_TIEROD, -1);
        v3 sw = sw_point(c), pin = V3(l.x, 0.30 - c->P.sprung_cg[1], l.z);
        put(&out[n++], vadd(c->x, mv(&R, sw)), vadd(c->x, mv(&R, pin)), DC_SEG_LINK, -1);
    }
    return n;
}
