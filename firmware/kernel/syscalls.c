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

#include "dev_interface.h"

char*  __env[1] = {0};
char** environ  = __env;

extern const interface_t dev_uart1;

int _write(int file, char* ptr, int len)
{
    if (file == STDOUT_FILENO || file == STDERR_FILENO)
    {
        return dev_uart1.write(ptr, (size_t)len);
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

clock_t _times(struct tms* buf)
{
    (void)buf;
    return (clock_t)(-1);
}

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
