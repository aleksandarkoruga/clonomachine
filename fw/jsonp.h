/* Minimal JSON reader for small config files (fw/midimap.c): walks objects and arrays in
 * place, numbers as Q16 fixed point, no allocation. */
#ifndef JSONP_H
#define JSONP_H

#include <stdint.h>

struct json {
    const char* p;
    const char* end;
    int err; /* set on the first syntax error; every call then returns 0 */
};

void json_init(struct json* j, const char* buf, int len);
/* Object members: call with *first = 1 after '{' was ... not consumed yet; returns 1 and the
   key when a member follows (its ':' consumed, the value is next), 0 at the closing '}'. */
int json_member(struct json* j, int* first, char* key, int n);
/* Array elements: returns 1 when an element follows, 0 at the closing ']'. */
int json_element(struct json* j, int* first);
int json_string(struct json* j, char* out, int n);
int json_q16(struct json* j, int32_t* v);
int json_int(struct json* j, int* v);
int json_bool(struct json* j, int* v);
int json_skip(struct json* j); /* any value */
/* offset of the error (for messages) */
int json_pos(const struct json* j, const char* buf);

#endif
