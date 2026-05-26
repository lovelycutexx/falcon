#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "inner.h"

#define LOGN 9
#define N (1u << LOGN)
#define Q 12289

static void
load_packed_u16(const char *name, uint16_t *x)
{
    FILE *f = fopen(name, "r");
    char line[256];
    if (f == NULL) {
        perror(name);
        exit(1);
    }
    for (size_t w = 0; w < 32; w ++) {
        if (fgets(line, sizeof line, f) == NULL || strlen(line) < 64) {
            fprintf(stderr, "short read from %s at word %u\n", name, (unsigned)w);
            exit(1);
        }
        for (size_t lane = 0; lane < 16; lane ++) {
            char s[5];
            size_t pos = 64 - 4 * (lane + 1);
            memcpy(s, line + pos, 4);
            s[4] = 0;
            x[w * 16 + lane] = (uint16_t)strtoul(s, NULL, 16);
        }
    }
    fclose(f);
}

static void
load_packed_i16(const char *name, int16_t *x)
{
    uint16_t tmp[N];
    load_packed_u16(name, tmp);
    for (size_t u = 0; u < N; u ++) {
        x[u] = (int16_t)tmp[u];
    }
}

static int16_t
center_modq(uint16_t x)
{
    int32_t w = (int32_t)(x % Q);
    if (w > (Q >> 1)) {
        w -= Q;
    }
    return (int16_t)w;
}

int
main(void)
{
    uint16_t hm[N];
    uint16_t h_ntt[N];
    uint16_t s1_modq[N];
    int16_t s2[N];
    uint8_t tmp[4 * N + 1024];
    int ok;
    int rel_bad = 0;
    uint64_t s1_norm = 0;
    uint64_t s2_norm = 0;

    load_packed_u16("hm.hex", hm);
    load_packed_u16("h_ntt.hex", h_ntt);
    Zf(to_ntt_monty)(h_ntt, LOGN);
    load_packed_u16("rtl_s1_modq.hex", s1_modq);
    load_packed_i16("rtl_s2_i16.hex", s2);

    memset(tmp, 0, sizeof tmp);
    ok = Zf(verify_raw)(hm, s2, h_ntt, LOGN, tmp);

    for (size_t u = 0; u < N; u ++) {
        int16_t recovered_neg_s1 = ((int16_t *)tmp)[u];
        int16_t rtl_s1 = center_modq(s1_modq[u]);
        int32_t want_s1 = -(int32_t)recovered_neg_s1;
        int32_t s2v = s2[u];
        s1_norm += (uint64_t)((int64_t)rtl_s1 * (int64_t)rtl_s1);
        s2_norm += (uint64_t)((int64_t)s2v * (int64_t)s2v);
        if (want_s1 != rtl_s1) {
            if (rel_bad < 8) {
                printf("REL_MISMATCH[%u] rtl_s1=%d recovered_s1=%d raw_modq=%u s2=%d\n",
                    (unsigned)u, rtl_s1, want_s1, s1_modq[u], s2[u]);
            }
            rel_bad ++;
        }
    }

    printf("RTL_SIGNATURE_VERIFY verify_raw=%s relation_bad=%d norm_s1=%llu norm_s2=%llu total=%llu bound=34034726\n",
        ok ? "PASS" : "FAIL",
        rel_bad,
        (unsigned long long)s1_norm,
        (unsigned long long)s2_norm,
        (unsigned long long)(s1_norm + s2_norm));

    return (ok && rel_bad == 0) ? 0 : 1;
}
