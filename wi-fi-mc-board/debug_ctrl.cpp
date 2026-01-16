#include "debug_ctrl.h"

enum LOG_LEVEL g_APP_LOG_LEVEL = LOG_INFO;

void switch_dbg_print_lvl(enum LOG_LEVEL lvl)
{
    g_APP_LOG_LEVEL = lvl;
}
