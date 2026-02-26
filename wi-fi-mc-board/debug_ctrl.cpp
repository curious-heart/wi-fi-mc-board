#include "debug_ctrl.h"

LOG_LEVEL g_APP_LOG_LEVEL = LOG_ERROR;

void switch_dbg_print_lvl(LOG_LEVEL lvl)
{
    g_APP_LOG_LEVEL = lvl;
}
