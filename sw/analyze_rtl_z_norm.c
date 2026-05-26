#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "inner.h"

#define LOGN 9
#define N (1u << LOGN)
#define HN (N >> 1)

static uint64_t
fpr_bits(fpr x)
{
    uint64_t u;
    memcpy(&u, &x, sizeof u);
    return u;
}

static fpr
fpr_from_bits(uint64_t u)
{
    fpr x;
    memcpy(&x, &u, sizeof x);
    return x;
}

static int
read_word(FILE *f, uint64_t *re, uint64_t *im)
{
    char line[256];
    if (fgets(line, sizeof line, f) == NULL) {
        return 0;
    }
    if (strlen(line) < 64) {
        return 0;
    }
    char im_s[17], re_s[17];
    memcpy(im_s, line + 32, 16);
    memcpy(re_s, line + 48, 16);
    im_s[16] = 0;
    re_s[16] = 0;
    *im = (uint64_t)strtoull(im_s, NULL, 16);
    *re = (uint64_t)strtoull(re_s, NULL, 16);
    return 1;
}

static void
load_fpr_poly(const char *name, fpr *x)
{
    FILE *f = fopen(name, "r");
    if (f == NULL) {
        perror(name);
        exit(1);
    }
    for (size_t u = 0; u < HN; u ++) {
        uint64_t re, im;
        if (!read_word(f, &re, &im)) {
            fprintf(stderr, "short read %s at %u\n", name, (unsigned)u);
            exit(1);
        }
        x[u] = fpr_from_bits(re);
        x[u + HN] = fpr_from_bits(im);
    }
    fclose(f);
}

static void
load_hm(const char *name, uint16_t *hm)
{
    FILE *f = fopen(name, "r");
    if (f == NULL) {
        perror(name);
        exit(1);
    }
    for (size_t w = 0; w < 32; w ++) {
        char line[256];
        if (fgets(line, sizeof line, f) == NULL || strlen(line) < 64) {
            fprintf(stderr, "short hm read at word %u\n", (unsigned)w);
            exit(1);
        }
        for (size_t lane = 0; lane < 16; lane ++) {
            char s[5];
            size_t pos = 64 - 4 * (lane + 1);
            memcpy(s, line + pos, 4);
            s[4] = 0;
            hm[w * 16 + lane] = (uint16_t)strtoul(s, NULL, 16);
        }
    }
    fclose(f);
}

static int64_t
center_modq(int64_t x)
{
    x %= 12289;
    if (x < 0) {
        x += 12289;
    }
    if (x > 6144) {
        x -= 12289;
    }
    return x;
}

int
main(void)
{
    fpr *z0 = calloc(N, sizeof *z0);
    fpr *z1 = calloc(N, sizeof *z1);
    fpr *b00 = calloc(N, sizeof *b00);
    fpr *b01 = calloc(N, sizeof *b01);
    fpr *b10 = calloc(N, sizeof *b10);
    fpr *b11 = calloc(N, sizeof *b11);
    fpr *s1f = calloc(N, sizeof *s1f);
    fpr *s2f = calloc(N, sizeof *s2f);
    fpr *tmp = calloc(N, sizeof *tmp);
    uint16_t hm[N];
    int16_t s1[N], s2[N];
    uint64_t norm = 0;
    uint64_t s1_norm = 0;
    uint64_t s2_norm = 0;

    if (!z0 || !z1 || !b00 || !b01 || !b10 || !b11 || !s1f || !s2f || !tmp) {
        fprintf(stderr, "calloc failed\n");
        return 1;
    }

    load_fpr_poly("fs_z0_rtl.hex", z0);
    load_fpr_poly("fs_z1_rtl.hex", z1);
    load_fpr_poly("b00.hex", b00);
    load_fpr_poly("b01.hex", b01);
    load_fpr_poly("b10.hex", b10);
    load_fpr_poly("b11.hex", b11);
    load_hm("hm.hex", hm);

    memcpy(s1f, z0, N * sizeof *z0);
    Zf(poly_mul_fft)(s1f, b00, LOGN);
    memcpy(tmp, z1, N * sizeof *z1);
    Zf(poly_mul_fft)(tmp, b10, LOGN);
    Zf(poly_add)(s1f, tmp, LOGN);

    memcpy(s2f, z0, N * sizeof *z0);
    Zf(poly_mul_fft)(s2f, b01, LOGN);
    memcpy(tmp, z1, N * sizeof *z1);
    Zf(poly_mul_fft)(tmp, b11, LOGN);
    Zf(poly_add)(s2f, tmp, LOGN);

    Zf(iFFT)(s1f, LOGN);
    Zf(iFFT)(s2f, LOGN);

    for (size_t u = 0; u < N; u ++) {
        int32_t r1 = (int32_t)fpr_rint(s1f[u]);
        int32_t r2 = (int32_t)fpr_rint(s2f[u]);
        int32_t a = (int32_t)hm[u] - r1;
        int32_t b = -r2;
        s1[u] = (int16_t)a;
        s2[u] = (int16_t)b;
        s1_norm += (uint64_t)((int64_t)a * (int64_t)a);
        s2_norm += (uint64_t)((int64_t)b * (int64_t)b);
    }
    norm = s1_norm + s2_norm;

    printf("C_ANALYZE_RTL_Z s1_norm=%llu s2_norm=%llu norm=%llu bound=34034726 accept=%s\n",
        (unsigned long long)s1_norm,
        (unsigned long long)s2_norm,
        (unsigned long long)norm,
        norm <= 34034726ULL ? "YES" : "NO");
    for (size_t u = 0; u < 8; u ++) {
        printf("  coeff[%u] hm=%u s1=%d s2=%d s1f=%016llx s2f=%016llx\n",
            (unsigned)u, hm[u], s1[u], s2[u],
            (unsigned long long)fpr_bits(s1f[u]),
            (unsigned long long)fpr_bits(s2f[u]));
    }

    (void)center_modq;
    free(z0); free(z1); free(b00); free(b01); free(b10); free(b11);
    free(s1f); free(s2f); free(tmp);
    return 0;
}
