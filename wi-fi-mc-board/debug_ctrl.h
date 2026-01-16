#ifndef __DEBUG_CTRL_H__
#define __DEBUG_CTRL_H__

enum LOG_LEVEL {
    LOG_DISABLE = -1,   //关闭log
    LOG_DEBUG = 0,//调试
    LOG_INFO,   //信息
    LOG_WARN,   //警告
    LOG_ERROR,   //错误
};

extern enum LOG_LEVEL g_APP_LOG_LEVEL;

/* do not use these macros. use DBG_PRINT and DBG_PRINTLN below.*/
#define DIY_LOG(level, output_func, s) \
{\
    if((g_APP_LOG_LEVEL >= LOG_DEBUG) && (level) >= g_APP_LOG_LEVEL)\
    {\
        output_func(s);\
    }\
}

#define DIY_LOG_FMT(level, output_func, s, fmt) \
{\
    if((g_APP_LOG_LEVEL >= LOG_DEBUG) && (level) >= g_APP_LOG_LEVEL)\
    {\
        output_func(s, fmt);\
    }\
}

#define DBG_PRINT(level, s) DIY_LOG(level, g_dbg_serial.print, s)
#define DBG_PRINT_FMT(level, s, fmt) DIY_LOG_FMT(level, g_dbg_serial.print, s, fmt)
#define DBG_PRINTLN(level, s) DIY_LOG(level, g_dbg_serial.println, s)
#define DBG_PRINTLN_FMT(level, s, fmt) DIY_LOG_FMT(level, g_dbg_serial.println, s, fmt)

void switch_dbg_print_lvl(enum LOG_LEVEL lvl);

#endif
