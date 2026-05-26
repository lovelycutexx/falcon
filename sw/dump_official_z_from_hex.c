#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "inner.h"
#include "sign.c"

#define LOGN 9
#define N (1u << LOGN)
#define HN (N >> 1)

static uint64_t
fpr_bits_local(fpr x)
{
    uint64_t u;
    memcpy(&u, &x, sizeof u);
    return u;
}

static fpr
fpr_from_bits_local(uint64_t u)
{
    fpr x;
    memcpy(&x, &u, sizeof x);
    return x;
}

static int
read_word(FILE *f, uint64_t *re, uint64_t *im)
{
    char line[256];
    char re_s[17], im_s[17];
    if (fgets(line, sizeof line, f) == NULL || strlen(line) < 64) {
        return 0;
    }
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
        x[u] = fpr_from_bits_local(re);
        x[u + HN] = fpr_from_bits_local(im);
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

static void
write_fft_poly_rtl_hex(const char *name, const fpr *x)
{
    FILE *f = fopen(name, "w");
    if (f == NULL) {
        perror(name);
        exit(1);
    }
    for (size_t u = 0; u < HN; u ++) {
        fprintf(f, "%016llx%016llx%016llx%016llx\n",
            0ULL, 0ULL,
            (unsigned long long)fpr_bits_local(x[u + HN]),
            (unsigned long long)fpr_bits_local(x[u]));
    }
    for (size_t u = HN; u < N; u ++) {
        size_t m = N - 1 - u;
        fprintf(f, "%016llx%016llx%016llx%016llx\n",
            0ULL, 0ULL,
            (unsigned long long)fpr_bits_local(fpr_neg(x[m + HN])),
            (unsigned long long)fpr_bits_local(x[m]));
    }
    fclose(f);
}

static void
analyze_norm(const fpr *z0, const fpr *z1, const fpr *b00,
             const fpr *b01, const fpr *b10, const fpr *b11,
             const uint16_t *hm)
{
    fpr *s1f = calloc(N, sizeof *s1f);
    fpr *s2f = calloc(N, sizeof *s2f);
    fpr *tmp = calloc(N, sizeof *tmp);
    uint64_t s1_norm = 0, s2_norm = 0;
    if (!s1f || !s2f || !tmp) {
        fprintf(stderr, "calloc failed\n");
        exit(1);
    }
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
    write_fft_poly_rtl_hex("sw_s2_fft_official.hex", s2f);
    Zf(iFFT)(s1f, LOGN);
    Zf(iFFT)(s2f, LOGN);
    write_fft_poly_rtl_hex("sw_s2_time_official.hex", s2f);
    for (size_t u = 0; u < N; u ++) {
        int64_t s1 = (int64_t)hm[u] - (int64_t)fpr_rint(s1f[u]);
        int64_t s2 = -(int64_t)fpr_rint(s2f[u]);
        s1_norm += (uint64_t)(s1 * s1);
        s2_norm += (uint64_t)(s2 * s2);
    }
    {
        FILE *f = fopen("sw_s2_i16_official.hex", "w");
        if (f == NULL) {
            perror("sw_s2_i16_official.hex");
            exit(1);
        }
        for (size_t w = 0; w < 32; w ++) {
            for (int lane = 15; lane >= 0; lane --) {
                size_t u = w * 16 + (size_t)lane;
                int16_t s2 = (int16_t)(-(int64_t)fpr_rint(s2f[u]));
                fprintf(f, "%04x", (uint16_t)s2);
            }
            fprintf(f, "\n");
        }
        fclose(f);
    }
    printf("OFFICIAL_Z_NORM s1=%llu s2=%llu total=%llu bound=34034726 accept=%s\n",
        (unsigned long long)s1_norm,
        (unsigned long long)s2_norm,
        (unsigned long long)(s1_norm + s2_norm),
        (s1_norm + s2_norm) <= 34034726ULL ? "YES" : "NO");
    free(s1f);
    free(s2f);
    free(tmp);
}

static unsigned sampler_trace_count;

static int
sampler_trace(void *ctx, fpr mu, fpr isigma)
{
    int z = Zf(sampler)(ctx, mu, isigma);
    if (sampler_trace_count < 64) {
        printf("SW_SAMPLE[%u] mu=%016llx si=%016llx z=%d diff=%.6f\n",
            sampler_trace_count,
            (unsigned long long)fpr_bits_local(mu),
            (unsigned long long)fpr_bits_local(isigma),
            z,
            (double)z - (double)mu);
    }
    sampler_trace_count ++;
    return z;
}

int
main(void)
{
    const size_t treesize = ffLDL_treesize(LOGN);
    fpr *expanded = calloc(4 * N + treesize, sizeof *expanded);
    fpr *t0 = calloc(N * 6, sizeof *t0);
    fpr *t1 = t0 + N;
    fpr *z0 = t1 + N;
    fpr *z1 = z0 + N;
    fpr *tmp = z1 + N;
    uint16_t hm[N];
    inner_shake256_context rng_sc;
    sampler_context spc;
    FILE *f;

    if (!expanded || !t0) {
        fprintf(stderr, "calloc failed\n");
        return 1;
    }
    f = fopen("expanded_key.bin", "rb");
    if (f == NULL) {
        perror("expanded_key.bin");
        return 1;
    }
    if (fread(expanded, sizeof *expanded, 4 * N + treesize, f) != 4 * N + treesize) {
        fprintf(stderr, "short read expanded_key.bin\n");
        return 1;
    }
    fclose(f);

    load_fpr_poly("t0_target.hex", t0);
    load_fpr_poly("t1_target.hex", t1);
    load_hm("hm.hex", hm);

    inner_shake256_init(&rng_sc);
    {
        uint8_t rng_seed[48] = {
            0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF,
            0x10,0x32,0x54,0x76,0x98,0xBA,0xDC,0xFE,
            0x55,0xAA,0x33,0xCC,0x77,0x88,0x99,0x00,
            0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,
            0xDE,0xAD,0xBE,0xEF,0xCA,0xFE,0xBA,0xBE,
            0xFE,0xED,0xFA,0xCE,0x12,0x34,0x56,0x78
        };
        inner_shake256_inject(&rng_sc, rng_seed, sizeof rng_seed);
    }
    inner_shake256_flip(&rng_sc);
    Zf(prng_init)(&spc.p, &rng_sc);
    spc.sigma_min = fpr_sigma_min[LOGN];

    ffSampling_fft(sampler_trace, &spc, z0, z1, expanded + 4 * N,
                   t0, t1, LOGN, tmp);

    write_fft_poly_rtl_hex("sw_z0_official.hex", z0);
    write_fft_poly_rtl_hex("sw_z1_official.hex", z1);
    printf("OFFICIAL_Z first z0=%016llx z1=%016llx\n",
        (unsigned long long)fpr_bits_local(z0[0]),
        (unsigned long long)fpr_bits_local(z1[0]));
    printf("OFFICIAL_SAMPLE_COUNT %u\n", sampler_trace_count);
    analyze_norm(z0, z1, expanded, expanded + N, expanded + 2 * N,
                 expanded + 3 * N, hm);

    free(expanded);
    free(t0);
    return 0;
}
