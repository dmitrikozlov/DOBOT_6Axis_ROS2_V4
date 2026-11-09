/* LD_PRELOAD shim for fjt_cycle_fallback.py: connections to port 65511 go to
 * 127.0.0.1:16511 instead, where the test runs a proxy it can switch off.
 *
 *   gcc -shared -fPIC -o redirect_65511.so redirect_65511.c -ldl
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <dlfcn.h>
#include <netinet/in.h>
#include <sys/socket.h>

static int (*real)(int, const struct sockaddr *, socklen_t);

/* At load, not in connect(): a library constructor calling connect() holds the
 * loader lock that dlsym() needs. */
__attribute__((constructor)) static void init(void)
{
    real = (int (*)(int, const struct sockaddr *, socklen_t))dlsym(RTLD_NEXT, "connect");
}

int connect(int fd, const struct sockaddr *addr, socklen_t len)
{
    if (!real)      /* called from a constructor that ran before init() */
        init();
    if (addr && addr->sa_family == AF_INET && len >= sizeof(struct sockaddr_in) &&
        ntohs(((const struct sockaddr_in *)addr)->sin_port) == 65511)
    {
        struct sockaddr_in to = *(const struct sockaddr_in *)addr;
        to.sin_port = htons(16511);
        to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        return real(fd, (const struct sockaddr *)&to, sizeof(to));
    }
    return real(fd, addr, len);
}
