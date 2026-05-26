#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "inner.h"

#define LOGN 9
#define N (1u << LOGN)
#define HN (N >> 1)
#define LEVELS LOGN
#define FLAT_WORDS (LEVELS * HN)

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
load_complex_words(const char *name, fpr *x)
{
    FILE *f = fopen(name, "r");
    if (f == NULL) {
        perror(name);
        exit(1);
    }
    for (size_t u = 0; u < HN; u ++) {
        uint64_t re, im;
        if (!read_word(f, &re, &im)) {
            fprintf(stderr, "short read from %s at %u\n", name, (unsigned)u);
            exit(1);
        }
        x[u] = fpr_from_bits(re);
        x[u + HN] = fpr_from_bits(im);
    }
    fclose(f);
}

static void
store_node(fpr *flat, unsigned level, unsigned index, const fpr *node)
{
    size_t words = HN >> level;
    size_t base = (size_t)level * HN + (size_t)index * words;
    for (size_t u = 0; u < words; u ++) {
        flat[base + u] = node[u];
        flat[FLAT_WORDS + base + u] = node[u + words];
    }
}

static void
build_nodes_rec(fpr *flat, const fpr *node, unsigned level, unsigned index)
{
    unsigned logn = LOGN - level;
    size_t n = (size_t)1 << logn;
    if (level >= LEVELS) {
        return;
    }
    store_node(flat, level, index, node);
    if (level + 1 < LEVELS) {
        fpr *f0 = calloc(n >> 1, sizeof *f0);
        fpr *f1 = calloc(n >> 1, sizeof *f1);
        fpr *tmp = calloc(n, sizeof *tmp);
        if (f0 == NULL || f1 == NULL || tmp == NULL) {
            fprintf(stderr, "calloc failed\n");
            exit(1);
        }
        memcpy(tmp, node, n * sizeof *tmp);
        Zf(poly_split_fft)(f0, f1, tmp, logn);
        build_nodes_rec(flat, f0, level + 1, index << 1);
        build_nodes_rec(flat, f1, level + 1, (index << 1) + 1);
        free(f0);
        free(f1);
        free(tmp);
    }
}

static void
write_nodes(const char *name, const fpr *flat)
{
    FILE *f = fopen(name, "w");
    if (f == NULL) {
        perror(name);
        exit(1);
    }
    for (size_t u = 0; u < FLAT_WORDS; u ++) {
        write_word(f, fpr_bits(flat[u]), fpr_bits(flat[FLAT_WORDS + u]));
    }
    fclose(f);
}

int
main(void)
{
    fpr *t0 = calloc(N, sizeof *t0);
    fpr *t1 = calloc(N, sizeof *t1);
    fpr *nodes0 = calloc(FLAT_WORDS * 2, sizeof *nodes0);
    fpr *nodes1 = calloc(FLAT_WORDS * 2, sizeof *nodes1);
    if (t0 == NULL || t1 == NULL || nodes0 == NULL || nodes1 == NULL) {
        fprintf(stderr, "calloc failed\n");
        return 1;
    }

    load_complex_words("t0_target.hex", t0);
    load_complex_words("t1_target.hex", t1);
    build_nodes_rec(nodes0, t0, 0, 0);
    build_nodes_rec(nodes1, t1, 0, 0);
    write_nodes("ffid_t0_nodes.hex", nodes0);
    write_nodes("ffid_t1_nodes.hex", nodes1);

    free(t0);
    free(t1);
    free(nodes0);
    free(nodes1);
    return 0;
}
