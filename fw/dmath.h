/* exp / log in double without libm (the soft core has no FPU; used where tables are built,
 * not per sample). */
#ifndef DMATH_H
#define DMATH_H

static inline double d_exp(double x) {
    /* e^x = 2^k * e^r, |r| <= ln2/2 */
    const double ln2 = 0.69314718055994530942;
    int k = (int)(x / ln2 + (x >= 0 ? 0.5 : -0.5));
    double r = x - k * ln2, term = 1, sum = 1;
    for (int i = 1; i < 18; ++i) {
        term *= r / i;
        sum += term;
    }
    while (k > 0) { sum *= 2; --k; }
    while (k < 0) { sum *= 0.5; ++k; }
    return sum;
}
static inline double d_log(double x) {
    /* ln x = 2 atanh((x-1)/(x+1)) after scaling into [0.5, 2) */
    const double ln2 = 0.69314718055994530942;
    int k = 0;
    while (x >= 2) { x *= 0.5; ++k; }
    while (x < 0.5) { x *= 2; --k; }
    const double y = (x - 1) / (x + 1), y2 = y * y;
    double t = y, sum = 0;
    for (int i = 1; i < 60; i += 2) {
        sum += t / i;
        t *= y2;
    }
    return 2 * sum + k * ln2;
}

#endif
