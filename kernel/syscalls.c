/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright 2025 Michael Kaa */

/* Includes */
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

#include "drv_face.h"
#include "uart.h"

char*  __env[1] = {0};
char** environ  = __env;

extern const drv_face_t dev_uart1;

#define PRINTF_BUF_SIZE (8192U)
static uint8_t  printf_buf[PRINTF_BUF_SIZE];
static volatile uint32_t printf_head = 0;
static volatile uint32_t printf_tail = 0;

void printf_flush(void) {
    if (printf_tail == printf_head) return;
    if (!dev_uart1.ioctl(UART_TX_READY, NULL)) return;

    uint32_t head = printf_head;
    uint32_t tail = printf_tail;
    uint32_t chunk = (head > tail) ? (head - tail) : (PRINTF_BUF_SIZE - tail);

    int sent = dev_uart1.write(&printf_buf[tail], chunk);
    if (sent > 0)
        printf_tail = (tail + (uint32_t)sent) % PRINTF_BUF_SIZE;
}

int printf_flush_pending(void) {
    return printf_tail != printf_head;
}

static void printf_drain(void) {
    while (printf_flush_pending()) printf_flush();
}

int _write(int file, char* ptr, int len)
{
    if (file == STDOUT_FILENO || file == STDERR_FILENO)
    {
        for (int i = 0; i < len; i++) {
            if (((printf_head + 1) % PRINTF_BUF_SIZE) == printf_tail)
                printf_drain(); // buffer full - wait
            printf_buf[printf_head] = (uint8_t)ptr[i];
            printf_head = (printf_head + 1) % PRINTF_BUF_SIZE;
        }
        return len;
    }
    errno = EIO;
    return -1;
}


int __io_getchar(void)
{
    uint8_t ch = 0;
    dev_uart1.read(&ch, 1);
    return ch;
}


int _read(int file, char* ptr, int len)
{
    if (file == STDIN_FILENO)
    {
        for (int DataIdx = 0; DataIdx < len; DataIdx++)
        {
            *ptr++ = (char)__io_getchar();
        }
        return len;
    }
    errno = EIO;
    return -1;
}



/* Functions */
void initialise_monitor_handles()
{
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

void _exit(int status)
{
    _kill(status, -1);
    while (1)
    {
    } /* Make sure we hang here */
}


int _close(int file)
{
    (void)file;
    return -1;
}


int _fstat(int file, struct stat* st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

int _open(char* path, int flags, ...)
{
    (void)path;
    (void)flags;
    /* Pretend like we always fail */
    return -1;
}

int _wait(int* status)
{
    (void)status;
    errno = ECHILD;
    return -1;
}

int _unlink(char* name)
{
    (void)name;
    errno = ENOENT;
    return -1;
}

// clock_t _times(struct tms* buf)
// {
//     (void)buf;
//     return (clock_t)(-1);
// }

int _stat(const char* file, struct stat* st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _link(char* old, char* new)
{
    (void)old;
    (void)new;
    errno = EMLINK;
    return -1;
}

int _fork(void)
{
    errno = EAGAIN;
    return -1;
}

int _execve(char* name, char** argv, char** env)
{
    (void)name;
    (void)argv;
    (void)env;
    errno = ENOMEM;
    return -1;
}
