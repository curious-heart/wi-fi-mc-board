#ifndef __DEBUG_CTRL_H__
#define __DEBUG_CTRL_H__

typedef enum
{
    LOG_DISABLE = -1,   //关闭log
    LOG_DEBUG = 0,//调试
    LOG_INFO,   //信息
    LOG_WARN,   //警告
    LOG_ERROR,   //错误
}LOG_LEVEL ;

extern LOG_LEVEL g_APP_LOG_LEVEL;

/* do not use these macros. use DBG_PRINT and DBG_PRINTLN below.*/
#define DIY_LOG(level, output_func, ...) \
{\
    if((g_APP_LOG_LEVEL >= LOG_DEBUG) && (level) >= g_APP_LOG_LEVEL)\
    {\
        output_func(__VA_ARGS__);\
    }\
}

#define DBG_PRINT(level, ...) DIY_LOG(level, g_dbg_serial.print, __VA_ARGS__)
#define DBG_PRINTLN(level, ...) DIY_LOG(level, g_dbg_serial.println, __VA_ARGS__)

void switch_dbg_print_lvl(LOG_LEVEL lvl);

#endif
