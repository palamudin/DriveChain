/* net.h - tiny non-blocking, single-client, line-oriented TCP server (POSIX sockets / Winsock) for the
   JSON-lines driver protocol. Binds to 127.0.0.1 by default: the control port is a local test hook, not a
   network service. */
#ifndef DEMO_NET_H
#define DEMO_NET_H
typedef struct { long long lfd, cfd; char buf[1 << 16]; int blen; int nclients; } net_srv;
int  net_listen(net_srv* N, const char* bind_addr, int port);   /* 1 ok */
int  net_accept(net_srv* N);                                     /* non-blocking; 1 if a client is (now) connected */
int  net_read_line(net_srv* N, char* line, int max);             /* non-blocking; 1 = a full line was read */
int  net_read_line_blocking(net_srv* N, char* line, int max);    /* 1 = line, 0 = client gone */
int  net_send(net_srv* N, const char* s);                        /* sends s as-is (caller adds '\n') */
void net_close(net_srv* N);
#endif
