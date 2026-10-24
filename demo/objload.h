/* objload.h - small OBJ/MTL loader (triangles, v/vt/vn, usemtl, Kd + map_Kd) -> raylib Model, one mesh per material.
   Replaces raylib's LoadOBJ, which stalls on multi-material files. Vertices are kept in the file's frame. */
#ifndef DEMO_OBJLOAD_H
#define DEMO_OBJLOAD_H
#include "raylib.h"
int obj_load(const char* path, Model* out);
#endif
