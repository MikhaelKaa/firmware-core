
#include <string.h> // strcmp
#include <stdio.h>  // printf
#include <stdint.h>
#include "term_gxf.h"
#include "ucmd.h"
#include "microrl.h"
#include "microrl.h"

#include "memory_man.h"
#include "ucmd_time.h"
#include "ucmd_led.h"
#include "ucmd_w25q.h"
#include "usb_cdc.h"

int ucmd_parse(command_t cmd_list[], int argc, const char **argv)
{
  if (!argv) return 0;        // return 0 for empty commands
  if (!cmd_list) return UCMD_CMD_NOT_FOUND;   // obviously not found, no list
  int retval = 0;

  if (argc) {
    command_t *c = NULL;
    for (command_t *p = cmd_list; p->cmd; p++)
      if (strcmp(p->cmd, &argv[0][0]) == 0) c = p;
    if (c) retval = c->fn(argc, (char**)argv);
    else retval = UCMD_CMD_NOT_FOUND;
  }

  return retval;
}


#include "stm32f407xx.h"
int ucmd_mcu_reset(int argc, char **argv) {
  (void)(argc);
  (void)(argv);
  
  NVIC_SystemReset();
}

extern const drv_face_t dev_usb_cdc;
int ucmd_usb(int argc, char **argv) {
  (void)(argc);
  (void)(argv);
  
  usb_cdc_debug_stats_t stats;
  dev_usb_cdc.ioctl(USB_CDC_GET_DEBUG_STATS, &stats);
  printf("USB Debug Stats:\r\n");
  printf("  RESET:  %lu\r\n", stats.reset_count);
  printf("  SETUP:  %lu\r\n", stats.setup_count);
  printf("  RXFLVL: %lu\r\n", stats.rxflvl_count);
  printf("  ADDR_SET: %lu\r\n", stats.address_set);
  printf("  EP0_IN_XFRC: %lu\r\n", stats.ep0_in_xfrc);
  printf("  MULTI_PKT: %lu\r\n", stats.multi_packet);
  printf("  Last SETUP: %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
         stats.last_setup[0], stats.last_setup[1], stats.last_setup[2], stats.last_setup[3],
         stats.last_setup[4], stats.last_setup[5], stats.last_setup[6], stats.last_setup[7]);
  return 0;
}

extern command_t cmd_list[];
// Пример cmd_list.
command_t cmd_list[] = {
  {
    .cmd  = "help",
    .help = "print available commands with their help text",
    .fn   = print_help_cb,
  },
  {
    .cmd  = "reset",
    .help = "reset mcu",
    .fn   = ucmd_mcu_reset,
  }, 
  {
    .cmd  = "mem",
    .help = "memory man, use mem help",
    .fn   = ucmd_mem,
  },
  {
    .cmd  = "time",
    .help = "rtc time. to set type time hh mm ss",
    .fn   = ucmd_time,
  },
  {
    .cmd  = "led",
    .help = "led PA1 ctrl",
    .fn   = ucmd_led,
  },
  {
    .cmd  = "w25q",
    .help = "w25q ctrl",
    .fn   = ucmd_w25q,
  },
  {
    .cmd  = "usb",
    .help = "usb debug stats",
    .fn   = ucmd_usb,
  },
  {0}, // null list terminator DON'T FORGET THIS!
};

int ucmd_execute(int argc, char **argv) {
  int ret = 0;
  ret = ucmd_parse(cmd_list, argc, (const char **)argv);
  if(ret == UCMD_CMD_NOT_FOUND){
    static uint8_t gssu = 0;
    if(gssu++ < 6)  {
      printf("unknown command");
      for(int i = 0; i < argc; i++) {
        printf(" %s", (char*)&argv[i][0]);
      }
      printf(", try help\r\n");
    } else {
      gssu = 0;
      set_display_atrib(F_RED);
      printf("GO SLEEP, STUPID USER!\r\n");
      resetcolor();
    }
  }
  return ret; 
}

int print_help_cb(int argc, char *argv[])
{
  (void)(argc);
  (void)(argv);
  command_t *p = cmd_list;
  while (p->cmd) {
    printf("%s \t%s\r\n", p->cmd, p->help);
    p++;
  }
  return 0;
}

void ucmd_default_print(const char * str) {
  printf("%s", str);
}
  
void default_sigint(void) {
  printf("default_sigint\r\n");
}

static volatile uint8_t ucmd_default_rx;
static microrl_t default_rl;

void ucmd_default_init(void) {
  setvbuf(stdin, NULL, _IONBF, 0);
  ucmd_default_rx = 0;
  // microrl_t * prl = &default_rl;
  // call init with ptr to microrl instance and print callback
  microrl_init(&default_rl, ucmd_default_print);
  // set callback for execute
  microrl_set_execute_callback(&default_rl, (int (*)(int, const char * const*))ucmd_execute);
  // set callback for completion (optionally)
  // microrl_set_complete_callback (prl, complet);
  // set callback for ctrl+c handling (optionally)
  microrl_set_sigint_callback(&default_rl, default_sigint);
  // microrl_insert_char(prl, '\r');
  microrl_insert_char(&default_rl, '\n');
  microrl_insert_char(&default_rl, '\n');  
}

void ucmd_default_proc(void) {
  // ucmd_default_rx = 0;
  scanf("%c", &ucmd_default_rx);
  microrl_insert_char(&default_rl, ucmd_default_rx);
}


void ucmd_set_sigint(void (*sigintf)(void)) {
  microrl_set_sigint_callback(&default_rl, sigintf);
}

