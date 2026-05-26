#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "inner.h"

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

static void
write_word(FILE *f, uint64_t re, uint64_t im)
{
    fprintf(f, "00000000000000000000000000000000%016llx%016llx\n",
        (unsigned long long)im, (unsigned long long)re);
}

static int
read_word(FILE *f, uint64_t *re, uint64_t *im)
{
    char line[256];
    if (fgets(line, sizeof line, f) == NULL) {
        return 0;
    }
    if (strlen(line) < 32) {
        return 0;
    }
    char re_s[17], im_s[17];
    memcpy(im_s, line + 32, 16);
    memcpy(re_s, line + 48, 16);
    im_s[16] = 0;
    re_s[16] = 0;
    *im = (uint64_t)strtoull(im_s, NULL, 16);
    *re = (uint64_t)strtoull(re_s, NULL, 16);
    return 1;
}

static void
load_complex_words(const char *name, fpr *x, unsigned logn)
{
    size_t n = (size_t)1 << logn;
    size_t hn = n >> 1;
    FILE *f = fopen(name, "r");
    if (f == NULL) {
        perror(name);
        exit(1);
    }
    for (size_t u = 0; u < hn; u ++) {
        uint64_t re, im;
        if (!read_word(f, &re, &im)) {
            fprintf(stderr, "short input: %s at word %u\n", name, (unsigned)u);
            exit(1);
        }
        x[u] = fpr_from_bits(re);
        x[u + hn] = fpr_from_bits(im);
    }
    fclose(f);
}

static void
write_complex_words(const char *name, const fpr *x, unsigned logn)
{
    size_t n = (size_t)1 << logn;
    size_t hn = n >> 1;
    FILE *f = fopen(name, "w");
    if (f == NULL) {
        perror(name);
        exit(1);
    }
    for (size_t u = 0; u < hn; u ++) {
        write_word(f, fpr_bits(x[u]), fpr_bits(x[u + hn]));
    }
    fclose(f);
}

static void
write_root_merge_full_words(const char *name, const fpr *x)
{
    FILE *f = fopen(name, "w");
    if (f == NULL) {
        perror(name);
        exit(1);
    }
    for (size_t u = 0; u < 256; u ++) {
        write_word(f, fpr_bits(x[u]), fpr_bits(x[u + 256]));
    }
    for (size_t u = 256; u < 512; u ++) {
        size_t v = 511 - u;
        write_word(f, fpr_bits(x[v]), fpr_bits(fpr_neg(x[v + 256])));
    }
    fclose(f);
}

static void
write_split_pair_words(const char *name, const fpr *f0, const fpr *f1,
    unsigned logn)
{
    size_t n = (size_t)1 << logn;
    size_t qn = n >> 2;
    FILE *f = fopen(name, "w");
    if (f == NULL) {
        perror(name);
        exit(1);
    }
    for (size_t u = 0; u < qn; u ++) {
        write_word(f, fpr_bits(f0[u]), fpr_bits(f0[u + qn]));
    }
    for (size_t u = 0; u < qn; u ++) {
        write_word(f, fpr_bits(f1[u]), fpr_bits(f1[u + qn]));
    }
    fclose(f);
}

static void
make_adjust_vectors(void)
{
    enum { LOGN = 8 };
    size_t n = (size_t)1 << LOGN;
    size_t hn = n >> 1;
    fpr *t0 = calloc(n, sizeof *t0);
    fpr *t1 = calloc(n, sizeof *t1);
    fpr *z1 = calloc(n, sizeof *z1);
    fpr *l10 = calloc(n, sizeof *l10);
    fpr *tmp = calloc(n, sizeof *tmp);
    if (t0 == NULL || t1 == NULL || z1 == NULL || l10 == NULL || tmp == NULL) {
        fprintf(stderr, "calloc failed\n");
        exit(1);
    }

    for (size_t u = 0; u < hn; u ++) {
        double a = (double)((int)(u % 17) - 8);
        double b = (double)((int)((u * 3) % 19) - 9);
        double c = (double)((int)((u * 5) % 23) - 11);
        double d = (double)((int)((u * 7) % 29) - 14);
        t0[u]      = fpr_of((int64_t)(a * 13.0));
        t0[u + hn] = fpr_of((int64_t)(b * 11.0));
        t1[u]      = fpr_of((int64_t)(c * 7.0));
        t1[u + hn] = fpr_of((int64_t)(d * 5.0));
        z1[u]      = fpr_of((int64_t)(c * 7.0 - ((int)(u & 3) - 1)));
        z1[u + hn] = fpr_of((int64_t)(d * 5.0 + ((int)((u + 1) & 3) - 1)));
        l10[u]      = fpr_of((u & 1) ? -2 : 3);
        l10[u + hn] = fpr_of((u & 2) ? 1 : -1);
    }

    memcpy(tmp, t1, n * sizeof *tmp);
    Zf(poly_sub)(tmp, z1, LOGN);
    Zf(poly_mul_fft)(tmp, l10, LOGN);
    Zf(poly_add)(tmp, t0, LOGN);

    write_complex_words("ffexu_adjust_t0.hex", t0, LOGN);
    write_complex_words("ffexu_adjust_t1.hex", t1, LOGN);
    write_complex_words("ffexu_adjust_z1.hex", z1, LOGN);
    write_complex_words("ffexu_adjust_l10.hex", l10, LOGN);
    write_complex_words("ffexu_adjust_exp.hex", tmp, LOGN);

    free(t0);
    free(t1);
    free(z1);
    free(l10);
    free(tmp);
}

int
main(void)
{
    enum { LOGN = 9 };
    size_t n = (size_t)1 << LOGN;
    fpr *root = calloc(n, sizeof *root);
    fpr *f0 = calloc(n >> 1, sizeof *f0);
    fpr *f1 = calloc(n >> 1, sizeof *f1);
    fpr *merged = calloc(n, sizeof *merged);
    if (root == NULL || f0 == NULL || f1 == NULL || merged == NULL) {
        fprintf(stderr, "calloc failed\n");
        return 1;
    }

    load_complex_words("t1_target.hex", root, LOGN);
    write_complex_words("ffexu_split_in.hex", root, LOGN);

    Zf(poly_split_fft)(f0, f1, root, LOGN);
    write_split_pair_words("ffexu_split_exp.hex", f0, f1, LOGN);
    write_split_pair_words("ffexu_merge_in.hex", f0, f1, LOGN);

    Zf(poly_merge_fft)(merged, f0, f1, LOGN);
    write_root_merge_full_words("ffexu_merge_exp.hex", merged);

    make_adjust_vectors();

    free(root);
    free(f0);
    free(f1);
    free(merged);
    return 0;
}
