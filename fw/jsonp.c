#include "jsonp.h"

void json_init(struct json* j, const char* buf, int len) {
    j->p = buf;
    j->end = buf + len;
    j->err = 0;
}

static void ws(struct json* j) {
    while (j->p < j->end && (*j->p == ' ' || *j->p == '\t' || *j->p == '\n' || *j->p == '\r'))
        ++j->p;
}

static int fail(struct json* j) {
    j->err = 1;
    return 0;
}

static int peek(struct json* j) {
    ws(j);
    return j->p < j->end ? *j->p : 0;
}

static int take(struct json* j, char c) {
    if (j->err || peek(j) != c)
        return fail(j);
    ++j->p;
    return 1;
}

int json_string(struct json* j, char* out, int n) {
    if (!take(j, '"'))
        return 0;
    int k = 0;
    while (j->p < j->end && *j->p != '"') {
        char c = *j->p++;
        if (c == '\\' && j->p < j->end) {
            c = *j->p++;
            if (c == 'n')
                c = '\n';
            else if (c == 't')
                c = '\t';
            else if (c == 'u') { /* \uXXXX: keep a '?' */
                j->p = j->p + 4 <= j->end ? j->p + 4 : j->end;
                c = '?';
            }
        }
        if (out && k < n - 1)
            out[k++] = c;
    }
    if (out && n)
        out[k] = 0;
    if (j->p >= j->end)
        return fail(j);
    ++j->p;
    return 1;
}

int json_q16(struct json* j, int32_t* v) {
    if (j->err)
        return 0;
    ws(j);
    int neg = 0, digits = 0;
    int64_t ip = 0, fp = 0, fd = 1;
    if (j->p < j->end && (*j->p == '-' || *j->p == '+'))
        neg = *j->p++ == '-';
    while (j->p < j->end && *j->p >= '0' && *j->p <= '9') {
        ip = ip * 10 + (*j->p++ - '0');
        ++digits;
    }
    if (j->p < j->end && *j->p == '.') {
        ++j->p;
        while (j->p < j->end && *j->p >= '0' && *j->p <= '9') {
            if (fd < 100000000) {
                fp = fp * 10 + (*j->p - '0');
                fd *= 10;
            }
            ++j->p;
            ++digits;
        }
    }
    if (!digits)
        return fail(j);
    int64_t q = (ip << 16) + ((fp << 16) + fd / 2) / fd;
    if (q > 0x7FFFFFFF)
        q = 0x7FFFFFFF;
    *v = (int32_t)(neg ? -q : q);
    return 1;
}

int json_int(struct json* j, int* v) {
    int32_t q;
    if (!json_q16(j, &q))
        return 0;
    *v = (q + (q >= 0 ? 32768 : -32768)) / 65536;
    return 1;
}

static int word(struct json* j, const char* w) {
    ws(j);
    const char* p = j->p;
    for (; *w; ++w, ++p)
        if (p >= j->end || *p != *w)
            return 0;
    j->p = p;
    return 1;
}

int json_bool(struct json* j, int* v) {
    if (j->err)
        return 0;
    if (word(j, "true"))
        *v = 1;
    else if (word(j, "false"))
        *v = 0;
    else {
        int32_t q;
        if (!json_q16(j, &q))
            return 0;
        *v = q != 0;
    }
    return 1;
}

int json_member(struct json* j, int* first, char* key, int n) {
    if (j->err)
        return 0;
    if (*first) {
        if (!take(j, '{'))
            return 0;
        *first = 0;
        if (peek(j) == '}') {
            ++j->p;
            return 0;
        }
    } else {
        const int c = peek(j);
        if (c == '}') {
            ++j->p;
            return 0;
        }
        if (!take(j, ','))
            return 0;
    }
    return json_string(j, key, n) && take(j, ':');
}

int json_element(struct json* j, int* first) {
    if (j->err)
        return 0;
    if (*first) {
        if (!take(j, '['))
            return 0;
        *first = 0;
        if (peek(j) == ']') {
            ++j->p;
            return 0;
        }
        return 1;
    }
    const int c = peek(j);
    if (c == ']') {
        ++j->p;
        return 0;
    }
    return take(j, ',');
}

int json_skip(struct json* j) {
    const int c = peek(j);
    if (j->err)
        return 0;
    if (c == '"')
        return json_string(j, 0, 0);
    if (c == '{') {
        int first = 1;
        while (json_member(j, &first, 0, 0))
            if (!json_skip(j))
                return 0;
        return !j->err;
    }
    if (c == '[') {
        int first = 1;
        while (json_element(j, &first))
            if (!json_skip(j))
                return 0;
        return !j->err;
    }
    if (word(j, "true") || word(j, "false") || word(j, "null"))
        return 1;
    int32_t q;
    return json_q16(j, &q);
}

int json_pos(const struct json* j, const char* buf) { return (int)(j->p - buf); }
