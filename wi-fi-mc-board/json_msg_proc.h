#ifndef __JSON_MSG_PROC_H__
#define __JSON_MSG_PROC_H__

#include <Arduino.h>
#include <ArduinoJson.h>

#define MAX_JSON_MSG_LEN 1024

typedef void (*json_str_msg_dispatcher_t)(const char * msg);
void json_msg_handler_dispatcher(const char * msg);
const char* json_msg_recv_proc(Stream &sport, json_str_msg_dispatcher_t dispatcher = json_msg_handler_dispatcher, size_t * msg_len_ptr = nullptr);

typedef void (*json_doc_hdlr)(JsonDocument& doc);

#endif
