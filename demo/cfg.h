/* cfg.h - minimal "key = value" config reader for the demo (sections are flattened as "section.key"). */
#ifndef DEMO_CFG_H
#define DEMO_CFG_H
typedef struct { int n; char key[256][64]; char val[256][192]; } cfg_t;
int cfg_load(cfg_t* c, const char* path);                 /* 1 ok */
void cfg_set(cfg_t* c, const char* key, const char* val); /* add/override (command line) */
const char* cfg_str(const cfg_t* c, const char* key, const char* def);
double cfg_num(const cfg_t* c, const char* key, double def);
int cfg_int(const cfg_t* c, const char* key, int def);
#endif
