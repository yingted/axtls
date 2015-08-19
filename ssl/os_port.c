/*
 * Copyright (c) 2007, Cameron Rich
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, 
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice, 
 *   this list of conditions and the following disclaimer in the documentation 
 *   and/or other materials provided with the distribution.
 * * Neither the name of the axTLS project nor the names of its contributors 
 *   may be used to endorse or promote products derived from this software 
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file os_port.c
 *
 * OS specific functions.
 */
#include "os_port.h"
#include <time.h>
#include <stdlib.h>
#include <errno.h>
#include <stdarg.h>
#if defined(CONFIG_PLATFORM_ESP8266)
#include <setjmp.h>
#endif

#ifdef WIN32
/**
 * gettimeofday() not in Win32 
 */
EXP_FUNC void STDCALL gettimeofday(struct timeval* t, void* timezone)
{       
#if defined(_WIN32_WCE)
    t->tv_sec = time(NULL);
    t->tv_usec = 0;                         /* 1sec precision only */ 
#else
    struct _timeb timebuffer;
    _ftime(&timebuffer);
    t->tv_sec = (long)timebuffer.time;
    t->tv_usec = 1000 * timebuffer.millitm; /* 1ms precision */
#endif
}

/**
 * strcasecmp() not in Win32
 */
EXP_FUNC int STDCALL strcasecmp(const char *s1, const char *s2)
{
    while (tolower(*s1) == tolower(*s2++))
    {
        if (*s1++ == '\0')
        {
            return 0;
        }
    }

    return *(unsigned char *)s1 - *(unsigned char *)(s2 - 1);
}


EXP_FUNC int STDCALL getdomainname(char *buf, int buf_size)
{
    HKEY hKey;
    unsigned long datatype;
    unsigned long bufferlength = buf_size;

    if (RegOpenKeyEx(HKEY_LOCAL_MACHINE,
            TEXT("SYSTEM\\CurrentControlSet\\Services\\Tcpip\\Parameters"),
                        0, KEY_QUERY_VALUE, &hKey) != ERROR_SUCCESS)
        return -1;

    RegQueryValueEx(hKey, "Domain", NULL, &datatype, buf, &bufferlength);
    RegCloseKey(hKey);
    return 0; 
}
#endif

#if !defined(CONFIG_PLATFORM_ESP8266)
#undef malloc
#undef realloc
#undef calloc

static const char * out_of_mem_str = "out of memory";
static const char * file_open_str = "Could not open file \"%s\"";
#endif

/* 
 * Some functions that call display some error trace and then call abort().
 * This just makes life much easier on embedded systems, since we're 
 * suffering major trauma...
 */
EXP_FUNC void * STDCALL ax_malloc(size_t s)
{
    void *x;

    if ((x = malloc(s)) == NULL)
        exit_now(out_of_mem_str);

    return x;
}

EXP_FUNC void * STDCALL ax_realloc(void *y, size_t s)
{
    void *x;

    if ((x = realloc(y, s)) == NULL)
        exit_now(out_of_mem_str);

    return x;
}

EXP_FUNC void * STDCALL ax_calloc(size_t n, size_t s)
{
    void *x;

    if ((x = calloc(n, s)) == NULL)
        exit_now(out_of_mem_str);

    return x;
}

#if defined(CONFIG_PLATFORM_ESP8266)
void os_port_exit_now() {
    system_restart();
    for (;;);
}

time_t os_port_time(time_t *time) {
    time_t ret = system_get_time() / 1000000;
    if (time != NULL) {
        *time = ret;
    }
    return ret;
}

int gettimeofday(struct timeval *__restrict tp, struct timezone *tz) {
    if (tp != NULL) {
        uint32_t usecs = system_get_time(), sec = usecs / 1000000;
        tp->tv_sec = CONFIG_BUILD_UNIX_TIME + sec;
        tp->tv_usec = usecs - sec * 1000000;
    }
    return 0;
}

// Only supports one blocking call at once.
static jmp_buf os_port_main_env, os_port_worker_env;
bool os_port_is_blocked = false;
#ifndef NDEBUG
static bool os_port_is_worker = false;
#endif
__attribute__((returns_twice))
void os_port_blocking_call(void (fn*)(void *), void *arg) {
    static char stack[1024];
    assert(!os_port_is_blocked);
    assert(!os_port_is_worker);
    if (!setjmp(os_port_main_env)) {
        os_port_is_blocked = false;
#ifndef NDEBUG
        os_port_is_worker = true;
#endif
        __asm__ __volatile__("mov a1, %0"::"r"(stack + sizeof(stack)));
        (*fn)(arg);
    } else {
        assert(!os_port_is_blocked);
#ifndef NDEBUG
        os_port_is_worker = false;
#endif
    }
}

void os_port_blocking_yield() {
    assert(!os_port_is_blocked);
    assert(os_port_is_worker);
    if (!setjmp(os_port_worker_env)) {
        os_port_is_blocked = true;
#ifndef NDEBUG
        os_port_is_worker = false;
#endif
        longjmp(os_port_main_env, 1);
    }
}

void os_port_blocking_resume() {
    assert(os_port_is_blocked);
    assert(!os_port_is_worker);
    if (!setjmp(os_port_main_env)) {
        os_port_is_blocked = false;
#ifndef NDEBUG
        os_port_is_worker = true;
#endif
        longjmp(os_port_worker_env, 1);
    }
}

ssize_t os_port_socket_read(int fd, void *buf, size_t len) {
    tcp_pcb *tpcb = (tcp_pcb *)fd;
    for (;;) {
        ...
    }
}

ssize_t os_port_socket_write(int fd, volatile void *buf, volatile size_t len) {
    volatile tcp_pcb *tpcb = (tcp_pcb *)fd;
    for (;;) {
        err_t rc = tcp_write(tpcb, buf, len, 0);
        switch (rc) {
            case ERR_OK:
                return len;
            case ERR_MEM:
                os_port_blocking_yield();
                continue;
            default:
                errno = rc;
                return -1;
        }
    }
}

int os_port_select(int nfds, fd_set *__restrict readfds, fd_set *__restrict writefds, fd_set *__restrict errorfds, struct timeval *__restrict timeout) {
    exit_now("select not implemented"); // unreachable
}

#else
EXP_FUNC int STDCALL ax_open(const char *pathname, int flags)
{
    int x;

    if ((x = open(pathname, flags)) < 0)
        exit_now(file_open_str, pathname);

    return x;
}

/**
 * This is a call which will deliberately exit an application, but will
 * display some information before dying.
 */
void exit_now(const char *format, ...)
{
    va_list argp;

    va_start(argp, format);
    vfprintf(stderr, format, argp);
    va_end(argp);
    abort();
}

#endif
