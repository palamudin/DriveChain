/* net.c - see net.h */
#include "net.h"
#include <string.h>
#include <stdio.h>
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET sock_t;
#define CLOSESOCK closesocket
static void set_nb(sock_t s, int nb) { u_long m = nb; ioctlsocket(s, FIONBIO, &m); }
static int would_block(void) { return WSAGetLastError() == WSAEWOULDBLOCK; }
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
typedef int sock_t;
#define INVALID_SOCKET (-1)
#define CLOSESOCK close
static void set_nb(sock_t s, int nb) { int f = fcntl(s, F_GETFL, 0); fcntl(s, F_SETFL, nb ? (f | O_NONBLOCK) : (f & ~O_NONBLOCK)); }
static int would_block(void) { return errno == EAGAIN || errno == EWOULDBLOCK; }
#endif

int net_listen(net_srv* N, const char* bind_addr, int port) {
    memset(N, 0, sizeof *N); N->lfd = N->cfd = -1;
#ifdef _WIN32
    WSADATA w; if (WSAStartup(MAKEWORD(2, 2), &w)) return 0;
#endif
    sock_t s = socket(AF_INET, SOCK_STREAM, 0); if (s == INVALID_SOCKET) return 0;
    int one = 1; setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char*)&one, sizeof one);
    struct sockaddr_in a; memset(&a, 0, sizeof a); a.sin_family = AF_INET; a.sin_port = htons((unsigned short)port);
    a.sin_addr.s_addr = inet_addr(bind_addr && *bind_addr ? bind_addr : "127.0.0.1");
    if (bind(s, (struct sockaddr*)&a, sizeof a) || listen(s, 1)) { CLOSESOCK(s); return 0; }
    set_nb(s, 1); N->lfd = (long long)s; return 1;
}
int net_accept(net_srv* N) {
    if (N->cfd >= 0) return 1;
    if (N->lfd < 0) return 0;
    sock_t c = accept((sock_t)N->lfd, NULL, NULL);
    if (c == INVALID_SOCKET) return 0;
    int one = 1; setsockopt(c, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof one);
    set_nb(c, 1); N->cfd = (long long)c; N->blen = 0; N->nclients++; return 1;
}
static void drop(net_srv* N) { if (N->cfd >= 0) CLOSESOCK((sock_t)N->cfd); N->cfd = -1; N->blen = 0; }
static int take_line(net_srv* N, char* line, int max) {
    for (int i = 0; i < N->blen; i++) if (N->buf[i] == '\n') {
        int n = i < max - 1 ? i : max - 1; memcpy(line, N->buf, n); line[n] = 0;
        if (n > 0 && line[n - 1] == '\r') line[n - 1] = 0;
        memmove(N->buf, N->buf + i + 1, N->blen - i - 1); N->blen -= i + 1; return 1;
    }
    if (N->blen >= (int)sizeof N->buf - 1) N->blen = 0;   /* overlong garbage: drop it */
    return 0;
}
int net_read_line(net_srv* N, char* line, int max) {
    if (!net_accept(N)) return 0;
    if (take_line(N, line, max)) return 1;
    int r = (int)recv((sock_t)N->cfd, N->buf + N->blen, (int)sizeof N->buf - 1 - N->blen, 0);
    if (r == 0 || (r < 0 && !would_block())) { drop(N); return 0; }
    if (r > 0) N->blen += r;
    return take_line(N, line, max);
}
int net_read_line_blocking(net_srv* N, char* line, int max) {
    for (;;) {
        if (N->cfd < 0) { set_nb((sock_t)N->lfd, 0); int ok = net_accept(N); set_nb((sock_t)N->lfd, 1); if (!ok) return 0; }
        if (take_line(N, line, max)) return 1;
        set_nb((sock_t)N->cfd, 0);
        int r = (int)recv((sock_t)N->cfd, N->buf + N->blen, (int)sizeof N->buf - 1 - N->blen, 0);
        if (N->cfd >= 0) set_nb((sock_t)N->cfd, 1);
        if (r <= 0) { drop(N); return 0; }
        N->blen += r;
    }
}
int net_send(net_srv* N, const char* s) {
    if (N->cfd < 0) return 0;
    int len = (int)strlen(s), off = 0, spins = 0;
    while (off < len) {
        int r = (int)send((sock_t)N->cfd, s + off, len - off, 0);
        if (r > 0) { off += r; continue; }
        if (r < 0 && would_block() && ++spins < 100000) continue;
        drop(N); return 0;
    }
    return 1;
}
void net_close(net_srv* N) {
    drop(N); if (N->lfd >= 0) CLOSESOCK((sock_t)N->lfd); N->lfd = -1;
#ifdef _WIN32
    WSACleanup();
#endif
}
