#include "json_keys.h"

#undef JSON_KEY

#define JSON_KEY(var_name, val) const char* var_name = val;

JSON_KEY_LIST;
