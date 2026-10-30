/* dc_lane_kern.h - the patch-ray kernel body, W rays per vector (GCC/Clang vector extensions), included by dc_lane.c
   once per width. Element-wise it performs exactly the operations of ray1() in dc_lane.c, in the same order, so every
   width gives the same bits. Defines LK_NAME(const dc_ptab*, dc_rayb*). */
#define LK_VF  LK_CAT(lk_vf, LK_W)
#define LK_VI  LK_CAT(lk_vi, LK_W)
typedef float LK_VF __attribute__((vector_size(4 * LK_W)));
typedef int   LK_VI __attribute__((vector_size(4 * LK_W)));
LK_ATTR static void LK_NAME(const dc_ptab* T, dc_rayb* R) {
    const int ni = T->ni, nj = T->nj;
    const LK_VF zero = {0}, one = zero + 1.0f, half = zero + 0.5f, two = zero + 2.0f, big = zero + 1e30f, tmax = zero + R->tmax, idw = zero + T->idw;
    const LK_VI izero = {0}, ione = izero + 1, ini1 = izero + (ni - 1), inj1 = izero + (nj - 1), ini = izero + ni;
#define SEL(m, a, b) ((LK_VF)(((m) & (LK_VI)(a)) | (~(m) & (LK_VI)(b))))
#define SELI(m, a, b) (((m) & (a)) | (~(m) & (b)))
    for (int r0 = 0; r0 < R->n; r0 += LK_W) {
        LK_VF u0, v0, z0, du, dv, dz, t;
        memcpy(&u0, R->u0 + r0, sizeof u0); memcpy(&v0, R->v0 + r0, sizeof v0); memcpy(&z0, R->z0 + r0, sizeof z0);
        memcpy(&du, R->du + r0, sizeof du); memcpy(&dv, R->dv + r0, sizeof dv); memcpy(&dz, R->dz + r0, sizeof dz);
        memcpy(&t, R->ts + r0, sizeof t);
        const LK_VI spx = du > zero, snx = du < zero, spy = dv > zero, sny = dv < zero;
        const LK_VI sxi = SELI(spx, ione, SELI(snx, -ione, izero)), syi = SELI(spy, ione, SELI(sny, -ione, izero));
        LK_VF pu = u0 + t * du, pv = v0 + t * dv;
        LK_VI i = __builtin_convertvector(pu, LK_VI), j = __builtin_convertvector(pv, LK_VI);
        i += pu < __builtin_convertvector(i, LK_VF); j += pv < __builtin_convertvector(j, LK_VF);   /* floor */
        LK_VI act = ~izero, st = izero;
        LK_VF oth = zero, ohx = zero, ohy = zero, ogr = zero;
        for (int g = 0; g < DC_LANE_GUARD; g++) {
            act &= t <= tmax;                                   /* past the tread circle: miss (st 0) */
            LK_VI oob = act & ((i < izero) | (j < izero) | (i >= ini1) | (j >= inj1));
            st = SELI(oob, -ione, st); act &= ~oob;
            int any = 0; for (int l = 0; l < LK_W; l++) any |= act[l];
            if (!any) break;
            LK_VF fi = __builtin_convertvector(i, LK_VF), fj = __builtin_convertvector(j, LK_VF);
            LK_VF tnx = SEL(spx, (fi + one - u0) / du, SEL(snx, (fi - u0) / du, big));
            LK_VF tny = SEL(spy, (fj + one - v0) / dv, SEL(sny, (fj - v0) / dv, big));
            LK_VF tm = SEL(tnx < tny, tnx, tny); LK_VF t1 = SEL(tm < tmax, tm, tmax); t1 = SEL(t1 < t, t, t1);
            LK_VI k = SELI(act, j * ini + i, izero);
            LK_VF h00, h10, h01, h11, g00, g10, g01, g11;
            for (int l = 0; l < LK_W; l++) {
                const int q = k[l];
                h00[l] = T->h[q]; h10[l] = T->h[q + 1]; h01[l] = T->h[q + ni]; h11[l] = T->h[q + ni + 1];
                g00[l] = T->g[q]; g10[l] = T->g[q + 1]; g01[l] = T->g[q + ni]; g11[l] = T->g[q + ni + 1];
            }
            LK_VF A = h00, Bc = h10 - h00, Cc = h01 - h00, Dc = h11 - h10 - h01 + h00;
            LK_VF uu = u0 - fi, vv = v0 - fj;
            LK_VF c0 = z0 - (A + Bc * uu + Cc * vv + Dc * uu * vv);
            LK_VF c1 = dz - (Bc * du + Cc * dv + Dc * (uu * dv + vv * du));
            LK_VF c2 = (zero - Dc) * du * dv;
            LK_VF fa = c0 + t * (c1 + t * c2), fb = c0 + t1 * (c1 + t1 * c2);
            LK_VI h1 = fa <= zero, h2 = ~h1 & (fb <= zero);
            LK_VF tv = (zero - c1) / (two * c2), fv = c0 + tv * (c1 + tv * c2);
            LK_VI h3 = ~h1 & ~h2 & (c2 > zero) & (tv > t) & (tv < t1) & (fv <= zero);
            LK_VF tb = SEL(h3, tv, t1), ftb = SEL(h3, fv, fb);
            LK_VI hb = (h2 | h3) & act;
            LK_VF lo = t, hi = tb, r = t + (tb - t) * fa / (fa - ftb);
            for (int it = 0; it < 3; it++) {
                LK_VF fr = c0 + r * (c1 + r * c2), dfr = c1 + two * c2 * r;
                LK_VI pos = fr > zero, neg = fr < zero, zr = ~pos & ~neg;
                lo = SEL(pos | zr, r, lo); hi = SEL(neg | zr, r, hi);
                LK_VF mid = half * (lo + hi);
                LK_VF rn = SEL(dfr != zero, r - fr / dfr, mid);
                r = SEL((rn > lo) & (rn < hi), rn, mid);
            }
            r = SEL(h1, t, r);
            LK_VI hit = (h1 & act) | hb;
            LK_VF u = uu + r * du, v = vv + r * dv;
            oth = SEL(hit, r, oth);
            ohx = SEL(hit, (Bc + Dc * v) * idw, ohx);
            ohy = SEL(hit, (Cc + Dc * u) * idw, ohy);
            ogr = SEL(hit, (one - v) * (g00 + u * (g10 - g00)) + v * (g01 + u * (g11 - g01)), ogr);
            st = SELI(hit, ione, st);
            act &= ~hit;
            act &= ~(t1 >= tmax);                               /* walked to the end of the ray: miss */
            t = SEL(act, t1, t);
            LK_VI stx = act & (tnx <= tny), sty = act & ~(tnx <= tny);
            i += SELI(stx, sxi, izero); j += SELI(sty, syi, izero);
            j += SELI(act & (tnx == tny) & (sxi != izero) & (syi != izero), syi, izero);   /* exact corner crossing */
        }
        st = SELI(act, -ione, st);                              /* guard ran out: hand back */
        memcpy(R->th + r0, &oth, sizeof oth); memcpy(R->hx + r0, &ohx, sizeof ohx); memcpy(R->hy + r0, &ohy, sizeof ohy);
        memcpy(R->gr + r0, &ogr, sizeof ogr); memcpy(R->st + r0, &st, sizeof st);
    }
#undef SEL
#undef SELI
}
#undef LK_VF
#undef LK_VI
