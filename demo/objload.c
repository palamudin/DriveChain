#include "objload.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define MAXM 96
typedef struct { char name[64]; Color kd; char tex[256]; float* buf; int n, cap; } omat;   /* buf: 8 floats per vertex */
static void push(omat* m, const float* v8) {
    if (m->n == m->cap) { m->cap = m->cap ? m->cap * 2 : 3072; m->buf = (float*)realloc(m->buf, sizeof(float) * 8 * m->cap); }
    memcpy(m->buf + 8 * m->n++, v8, 8 * sizeof(float));
}
static void dirof(const char* p, char* d, int n) { snprintf(d, n, "%s", p); char* s = strrchr(d, '/'); char* b = strrchr(d, '\\'); if (b > s) s = b; if (s) s[1] = 0; else d[0] = 0; }
int obj_load(const char* path, Model* out) {
    FILE* f = fopen(path, "r"); if (!f) return 0;
    char dir[512]; dirof(path, dir, sizeof dir);
    omat* M = (omat*)calloc(MAXM, sizeof(omat)); int nm = 1; strcpy(M[0].name, "default"); M[0].kd = WHITE;
    int nv = 0, nt = 0, nn = 0, cv = 4096, ct = 4096, cn = 4096;
    float *V = malloc(sizeof(float) * 3 * cv), *T = malloc(sizeof(float) * 2 * ct), *N = malloc(sizeof(float) * 3 * cn);
    int cur = 0; char line[1024];
    while (fgets(line, sizeof line, f)) {
        if (line[0] == 'v' && line[1] == ' ') { if (nv == cv) V = realloc(V, sizeof(float) * 3 * (cv *= 2)); sscanf(line + 2, "%f %f %f", &V[3 * nv], &V[3 * nv + 1], &V[3 * nv + 2]); nv++; }
        else if (line[0] == 'v' && line[1] == 't') { if (nt == ct) T = realloc(T, sizeof(float) * 2 * (ct *= 2)); T[2 * nt] = T[2 * nt + 1] = 0; sscanf(line + 3, "%f %f", &T[2 * nt], &T[2 * nt + 1]); nt++; }
        else if (line[0] == 'v' && line[1] == 'n') { if (nn == cn) N = realloc(N, sizeof(float) * 3 * (cn *= 2)); sscanf(line + 3, "%f %f %f", &N[3 * nn], &N[3 * nn + 1], &N[3 * nn + 2]); nn++; }
        else if (!strncmp(line, "mtllib", 6)) {
            char mp[600], nmb[256]; if (sscanf(line + 7, "%255s", nmb) != 1) continue; snprintf(mp, sizeof mp, "%s%s", dir, nmb);
            FILE* g = fopen(mp, "r"); if (!g) continue; char l2[512]; omat* m = NULL;
            while (fgets(l2, sizeof l2, g)) {
                if (!strncmp(l2, "newmtl", 6) && nm < MAXM) { m = &M[nm++]; sscanf(l2 + 7, "%63s", m->name); m->kd = WHITE; }
                else if (m && !strncmp(l2, "Kd", 2)) { float r, gg, b; if (sscanf(l2 + 3, "%f %f %f", &r, &gg, &b) == 3) m->kd = (Color){(unsigned char)(r * 255), (unsigned char)(gg * 255), (unsigned char)(b * 255), 255}; }
                else if (m && !strncmp(l2, "map_Kd", 6)) { char t[256]; if (sscanf(l2 + 7, "%255s", t) == 1) snprintf(m->tex, sizeof m->tex, "%s%s", dir, t); }
            }
            fclose(g);
        } else if (!strncmp(line, "usemtl", 6)) {
            char nmb[64]; sscanf(line + 7, "%63s", nmb); cur = 0; for (int i = 1; i < nm; i++) if (!strcmp(M[i].name, nmb)) cur = i;
        } else if (line[0] == 'f' && line[1] == ' ') {
            int vi[16], ti[16], ni[16], k = 0; char* p = line + 2;
            while (*p && k < 16) {
                while (*p == ' ' || *p == '\t') p++; if (!*p || *p == '\n' || *p == '\r') break;
                int a = 0, b = 0, c = 0; char* e;
                a = strtol(p, &e, 10); p = e;
                if (*p == '/') { p++; if (*p != '/') { b = strtol(p, &e, 10); p = e; } if (*p == '/') { p++; c = strtol(p, &e, 10); p = e; } }
                vi[k] = a < 0 ? nv + a : a - 1; ti[k] = b < 0 ? nt + b : b - 1; ni[k] = c < 0 ? nn + c : c - 1; k++;
                while (*p && *p != ' ' && *p != '\t') p++;
            }
            for (int t = 1; t + 1 < k; t++) {
                int idx[3] = {0, t, t + 1};
                float fn[3] = {0, 0, 1};
                { float* A = &V[3 * vi[0]]; float* B = &V[3 * vi[t]]; float* C = &V[3 * vi[t + 1]];
                  float u[3] = {B[0] - A[0], B[1] - A[1], B[2] - A[2]}, w[3] = {C[0] - A[0], C[1] - A[1], C[2] - A[2]};
                  fn[0] = u[1] * w[2] - u[2] * w[1]; fn[1] = u[2] * w[0] - u[0] * w[2]; fn[2] = u[0] * w[1] - u[1] * w[0]; }
                for (int q = 0; q < 3; q++) {
                    int j = idx[q]; float v8[8];
                    memcpy(v8, &V[3 * vi[j]], 12);
                    if (ti[j] >= 0 && ti[j] < nt) { v8[3] = T[2 * ti[j]]; v8[4] = 1.0f - T[2 * ti[j] + 1]; } else v8[3] = v8[4] = 0;
                    if (ni[j] >= 0 && ni[j] < nn) memcpy(v8 + 5, &N[3 * ni[j]], 12); else memcpy(v8 + 5, fn, 12);
                    push(&M[cur], v8);
                }
            }
        }
    }
    fclose(f); free(V); free(T); free(N);
    int used = 0; for (int i = 0; i < nm; i++) if (M[i].n) used++;
    Model mdl = {0}; mdl.transform = (Matrix){1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    mdl.meshCount = used; mdl.materialCount = used;
    mdl.meshes = (Mesh*)MemAlloc(sizeof(Mesh) * (used ? used : 1)); mdl.materials = (Material*)MemAlloc(sizeof(Material) * (used ? used : 1));
    mdl.meshMaterial = (int*)MemAlloc(sizeof(int) * (used ? used : 1));
    int k = 0;
    for (int i = 0; i < nm; i++) {
        omat* m = &M[i]; if (!m->n) continue;
        Mesh me = {0}; me.vertexCount = m->n; me.triangleCount = m->n / 3;
        me.vertices = MemAlloc(sizeof(float) * 3 * m->n); me.texcoords = MemAlloc(sizeof(float) * 2 * m->n); me.normals = MemAlloc(sizeof(float) * 3 * m->n);
        me.colors = MemAlloc(4 * m->n);
        for (int v = 0; v < m->n; v++) {
            float* s = m->buf + 8 * v;
            /* physics frame (x fwd, y left, z up) -> raylib render frame (x, z, -y); main.c's transforms expect render-frame mesh data */
            me.vertices[3 * v] = s[0]; me.vertices[3 * v + 1] = s[1]; me.vertices[3 * v + 2] = s[2];
            me.texcoords[2 * v] = s[3]; me.texcoords[2 * v + 1] = s[4];
            me.normals[3 * v] = s[5]; me.normals[3 * v + 1] = s[6]; me.normals[3 * v + 2] = s[7];
            me.colors[4 * v] = me.colors[4 * v + 1] = me.colors[4 * v + 2] = me.colors[4 * v + 3] = 255;
        }
        UploadMesh(&me, false);
        Material mt = LoadMaterialDefault(); mt.maps[MATERIAL_MAP_DIFFUSE].color = m->kd;
        if (m->tex[0] && FileExists(m->tex)) { Texture2D tx = LoadTexture(m->tex); GenTextureMipmaps(&tx); SetTextureFilter(tx, TEXTURE_FILTER_TRILINEAR); SetTextureWrap(tx, TEXTURE_WRAP_REPEAT); mt.maps[MATERIAL_MAP_DIFFUSE].texture = tx; }
        mdl.meshes[k] = me; mdl.materials[k] = mt; mdl.meshMaterial[k] = k; k++;
        free(m->buf);
    }
    free(M);
    *out = mdl; return used > 0;
}
