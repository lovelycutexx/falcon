#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "inner.h"

static uint64_t fpr_bits(fpr x) {
    uint64_t u; memcpy(&u, &x, sizeof(u)); return u;
}

static const int8_t ntru_f_512[] = {
    4, -4, 0, -6, 6, -6, 2, 1, -8, 0, -2, 0, -1, -1, -4, 8, -5, 3,
    -2, 2, 0, -5, -2, -1, 3, -4, -5, -1, 8, 1, 1, 7, 5, 1, 6, 2, -1,
    -13, 1, -4, 9, -4, -2, 4, -4, 0, -1, -1, -3, 2, 1, 1, 1, 3, -3,
    2, -1, -1, -5, 9, 4, -7, -3, -8, -3, -2, -3, -6, -6, -3, -2, -2,
    2, 1, -10, -2, -2, 4, 2, 0, -2, -2, 4, -3, 5, 2, -2, 3, 8, 1, 8,
    -3, -4, 2, 7, -5, -4, -2, -2, -3, 5, -5, 0, -3, -5, 3, -6, -2,
    3, 0, 3, 1, 2, -2, 1, 6, -1, -7, 0, -5, 3, -5, 9, 0, 1, 5, -4,
    0, 5, -1, 4, 3, 5, -6, 2, 0, -7, 1, 0, 0, 2, 4, 1, -7, -3, 4, 4,
    -2, -7, -5, 6, 3, 2, -5, 6, -1, -1, -4, 1, 2, 1, 2, -10, -9, -9,
    -1, 3, -2, -2, -6, 1, -2, -4, -1, 2, 3, 8, 2, 1, -1, 8, 0, 7, 3,
    1, 5, 0, -7, 1, -6, -4, 4, 2, 0, 0, -3, 2, 0, -3, 0, 7, -1, -1,
    -7, 2, 5, 3, 0, 1, 6, -2, -1, 2, 0, -1, -3, -6, -5, -5, -4, 0,
    1, 7, 1, -3, 2, 2, -5, 0, -4, 3, -4, 5, 3, 4, 7, -2, 15, -3, 1,
    1, 4, 5, -9, -3, 4, 2, -4, -4, -3, -1, -4, 3, -1, 1, -8, -4, -1,
    0, -3, 1, -1, 3, 3, 3, -3, -6, -7, 7, 0, -6, 2, -1, 4, 7, 1, 4,
    0, 1, 6, -1, -2, -2, 5, 0, 6, -3, -2, -5, 3, -1, 0, 5, -2, 8,
    -5, -4, 1, -3, 8, 2, -4, 1, 6, 0, 0, -1, 0, -4, -5, -2, 3, -2,
    5, 1, 4, 5, -4, 4, -1, 4, -5, -2, 1, 3, -5, 1, 2, -2, 0, -5, 1,
    8, -3, -4, 3, -2, -3, -4, 4, 3, -2, 6, -3, -2, 4, 0, -2, 0, -5,
    1, -9, 5, 6, -2, -6, 1, 5, -1, -7, 1, 2, 5, 2, 0, -1, 0, -2, -4,
    -1, -8, 5, -5, 9, -4, -4, 2, -5, -1, 0, 1, 4, 3, 1, -2, -7, -8,
    -4, -4, 4, 3, -1, 4, -1, -1, 1, 0, 6, 1, 0, -6, -2, 0, -3, 0,
    -1, -1, 0, 3, -5, -2, -5, 6, 2, -4, -3, 4, -8, 1, -1, 4, -3, 5,
    -2, 8, 7, -1, -3, -3, -2, 0, -4, 4, 0, -6, -4, -2, 5, 8, -3, 3,
    -1, 0, -5, -5, 0, 2, -5, -2, -3, 1, 6, 3, 1, -3, 4, -3, 0, -7,
    -1, -3, 1, -5, 1, -4, -2, 2, 4, 0, 1, 5, 2, 2, -3, -5, -8, 4,
    -2, -3, 2, 2, 0, 8, -5, 2, -7, 0, 3, -1, 0, 4, -3, 1, -2, -4,
    -6, -5, 0, -4, 1, -3, 9, 1, -3, -2, -3, 5, -1, -4, -7, 1, 1
};
static const int8_t ntru_g_512[] = {
    -6, -2, 4, -8, -4, 2, 3, 4, 1, -1, 3, 0, 2, 3, -3, 1, -7, -5, 3,
    -3, -1, 3, -3, 8, -6, -6, 0, 6, 4, 7, 3, 5, 0, -5, -3, -5, 7, 3,
    -1, -4, 3, 4, -1, 1, 3, -3, -4, -4, 4, -5, -1, 3, 7, -2, -4, -2,
    -3, -1, -2, -1, -2, -6, -7, -3, -6, -3, -6, 4, -1, -5, 1, 4, -4,
    3, -1, -6, 6, -2, 2, -6, 5, -7, 8, -3, 0, -2, 0, 7, 1, 3, 6, 4,
    -5, 2, 2, 2, 4, -4, -5, -4, -3, 4, -7, 7, -6, -2, -7, 1, -2, -2,
    -3, 1, 3, 7, 0, -1, -5, 4, -8, -8, 0, 3, 6, -3, 2, 6, -1, 1, -5,
    -4, 2, -3, 8, -2, 2, 3, 0, 1, 6, 4, 4, -4, -1, -3, -2, -5, 3, 9,
    0, 4, -1, 1, -4, 0, 3, 0, -2, 8, 0, 1, 0, -1, 1, 9, -1, -4, -1,
    3, 5, -2, -2, 1, -1, 1, -1, 0, 1, 0, -6, -2, 0, 7, -3, -4, -1,
    -6, -2, 5, -2, 0, 4, -3, -5, 0, 1, -1, -3, 5, 5, -4, -4, -5, -1,
    9, -1, -5, -7, -1, -2, 4, 2, 5, -4, -1, -5, 8, -3, -6, -2, 1,
    -2, 1, 1, 4, -4, -1, 4, 1, 1, 0, -5, 1, 7, 2, -3, 3, -2, -4, 1,
    -6, -1, -3, 7, 6, 0, -2, 2, -6, -4, -3, 2, -7, 7, 0, -11, -1, 3,
    4, 0, 6, -8, -4, -1, 1, 0, -3, 7, 0, 0, -2, -1, -4, 0, -1, -3,
    7, -6, -2, -2, -1, 0, -2, 8, -6, 4, 4, 6, -2, -1, 0, -13, 1, 2,
    0, 5, -7, 3, -2, -6, -3, -4, 4, -1, 1, 3, -6, 1, -5, -8, 2, -11,
    -1, 2, -2, 0, 0, 1, 1, -4, -5, 0, 1, 0, 1, -6, -2, 2, 0, 7, 1,
    -1, 1, -2, 1, -3, 1, 2, 1, -7, -2, 2, -1, 4, 1, -2, -2, 0, 4,
    -3, -6, 2, 3, 1, 1, -4, 6, -2, -4, -3, 0, 4, -5, 0, 1, 8, 2, 2,
    -1, 1, -2, -4, -1, 4, 4, -1, 7, 2, -1, -3, -8, 3, 1, 1, 0, -1,
    1, -7, -8, 2, 1, -2, 1, 0, 4, 1, 1, -2, -1, -5, 3, -4, -1, -1,
    -8, 2, -4, 3, 2, -5, 0, 1, 5, 2, -5, -2, 3, 7, 5, 6, 5, -2, 1,
    3, -7, 7, -3, -8, -2, 2, 3, 3, 5, -2, -4, -1, 7, -2, 7, -3, -2,
    0, 3, 5, 0, 0, 4, 8, -1, -5, 3, -2, -2, -5, -5, -2, 2, 5, -8,
    -1, -2, -4, 6, 0, 6, -5, -1, -5, -6, 9, 5, -2, 4, -1, -8, -2,
    -2, 1, -8, -5, 6, -1, 0, 5, -6, -3, -3, -2, -6, -2, 0, -1, -3,
    7, -3, -1, 3, 6, 3, -2, -4, 2, 1, -1, 11, 3, 4, -1, -6, 1, 2, 3, 3
};
static const int8_t ntru_F_512[] = {
    -3, -27, 4, 18, 39, 7, 20, -13, 33, -29, 3, 38, 30, 26, -6, 24,
    -26, 16, 24, -48, -18, -21, 3, -14, -2, 6, -9, 42, 22, 21, 33,
    -27, -14, -14, -56, -68, -2, -33, 6, -38, -43, 21, 13, 6, 2,
    -69, -10, -30, -27, 23, -1, 41, -21, 11, -20, 15, 39, 5, 41, 15,
    -28, -34, 9, -11, 9, -1, -8, 61, 8, 13, -23, 2, 7, -23, -21,
    -54, -11, -9, -19, 40, 37, -2, -16, 19, -16, 2, -78, -35, -19,
    11, 17, -46, -16, 25, 0, 22, 13, -15, -33, 13, -15, -34, 33,
    -13, 38, 39, 37, -29, 40, 7, 63, 35, 15, 21, -24, 16, -6, 30,
    12, 18, 61, 17, -11, -15, 11, 0, -15, -2, -14, -26, -1, -42,
    -10, -52, 64, 45, 22, 6, -22, 32, -50, -16, -12, -16, -8, 34,
    -17, -18, 7, 19, 37, 41, -5, -22, -12, -7, -17, -27, -17, 4, 36,
    0, 22, -4, -50, 24, 30, 5, 1, -50, 43, 0, 0, -6, -9, 34, 0, 14,
    -27, 17, 35, -30, -13, 3, -23, -46, 17, -34, 30, 24, 47, 31, -7,
    11, 10, 16, 30, 27, -4, 11, -4, -14, -28, 49, 0, 27, -5, -10,
    53, -50, -13, -15, 13, -10, -26, 2, -3, 88, 22, -27, 40, -23, 3,
    -42, 2, -27, -12, 35, 26, -33, 38, -42, -5, 17, -24, 6, -10, 13,
    -10, -30, -35, -17, 25, 49, -29, 48, 19, 37, 48, -25, -31, -41,
    -15, -1, 19, -17, -7, -16, 2, 5, 12, 0, -15, -19, -6, -32, -4,
    -56, 14, -6, -7, 17, 24, -1, 17, -35, 5, 3, -64, -15, 4, 0, -31,
    4, -10, -18, 55, 13, -13, 23, -30, -11, -29, -21, 15, -18, 30,
    39, 16, -27, 31, 4, 31, 39, -49, 11, -25, 37, -42, -72, 28, -57,
    13, 34, 6, 10, -17, -3, -19, -43, -1, -32, 9, -11, 9, 11, -23,
    6, 28, -34, -12, -42, -7, 42, -18, -2, 22, -30, -4, -42, 10, 54,
    -16, 19, -23, -4, 18, -58, 26, -3, -38, 20, 38, 23, 20, 0, 10,
    49, 47, 18, 27, -11, -10, -14, 0, 6, 6, -18, -6, 14, -38, -16,
    12, -17, 17, -21, -52, -3, -53, 9, 9, -4, 44, 9, -4, 17, 2, 10,
    -28, -13, 28, -12, 11, -33, -2, 33, 0, -51, 2, -33, 20, -47, 23,
    42, 2, 52, -18, -17, 35, 6, 27, 3, 11, 24, -8, 0, -35, -44, -22,
    -49, 61, 3, -15, -2, -14, 46, -24, -10, -24, -24, -21, -10, -51,
    -3, 31, 20, 1, -44, 18, 9, 38, 26, -17, -8, 2, 33, 24, -8, -9,
    -20, 32, 54, 47, -11, 40, 3, -58, 13, 17, 29, -21, 27, 4, -31,
    14, 14, 17, 19, -29, 19, -86, -29, -15, -35, 18, 53, -10, 9, 13,
    -38, 9, -4, 80, 0, 6, 1, 15, -14, 0, -5, 45, 26, 50, 28, 21, 1,
    -8, -6, 12, 32, 5, -21, -1, 54, 14, 22, 27, 6, 8, -18, 33, -5
};

int main() {
    unsigned logn = 9;
    size_t n = 512, hn = 256, qn = 128;

    size_t treesize = ((size_t)(logn + 1)) << logn;
    size_t ek_fpr = 4*n + treesize;
    fpr *ek = calloc(ek_fpr, sizeof(fpr));
    uint8_t *tmp = calloc((72u << logn) + 7, 1);
    uint8_t *tmp_cp = calloc((4u << logn), 1);

    int8_t G[512];
    Zf(complete_private)(G, ntru_f_512, ntru_g_512, ntru_F_512, logn, tmp_cp);
    Zf(expand_privkey)(ek, ntru_f_512, ntru_g_512, ntru_F_512, G, logn, tmp);

    fpr *t0 = calloc(n, sizeof(fpr));
    const char *msg = "FALCON_SIGN_TEST_MSG_V1.0______";
    inner_shake256_context sc;
    inner_shake256_init(&sc);
    inner_shake256_inject(&sc, (const uint8_t*)msg, 32);
    inner_shake256_flip(&sc);
    uint16_t hm[512];
    Zf(hash_to_point_vartime)(&sc, hm, logn);
    for (size_t u = 0; u < n; u++) t0[u] = fpr_of(hm[u]);
    Zf(FFT)(t0, logn);

    const fpr *b01 = ek + n;
    const fpr *b11 = ek + 3*n;
    fpr *target_t0 = calloc(n, sizeof(fpr));
    fpr *target_t1 = calloc(n, sizeof(fpr));
    memcpy(target_t0, t0, n * sizeof(fpr));
    memcpy(target_t1, t0, n * sizeof(fpr));
    Zf(poly_mul_fft)(target_t1, b01, logn);
    Zf(poly_mulconst)(target_t1, fpr_neg(fpr_inverse_of_q), logn);
    Zf(poly_mul_fft)(target_t0, b11, logn);
    Zf(poly_mulconst)(target_t0, fpr_inverse_of_q, logn);

    printf("=== SPLIT Golden Test - Root (L=0), idx=0 ===\n\n");

    fpr a_re = target_t0[0], a_im = target_t0[hn];
    fpr b_re = target_t0[1], b_im = target_t0[hn+1];

    printf("Input a (t0[0]): re=%016llx im=%016llx\n",
        (unsigned long long)fpr_bits(a_re), (unsigned long long)fpr_bits(a_im));
    printf("Input b (t0[1]): re=%016llx im=%016llx\n",
        (unsigned long long)fpr_bits(b_re), (unsigned long long)fpr_bits(b_im));

    fpr gm_re = fpr_gm_tab[(256 << 1) + 0];
    fpr gm_im = fpr_gm_tab[(256 << 1) + 1];
    printf("\nGM twiddle gm_tab[256]: re=%016llx im=%016llx\n",
        (unsigned long long)fpr_bits(gm_re), (unsigned long long)fpr_bits(gm_im));

    // === Golden SPLIT ===
    fpr *t0_copy = calloc(n, sizeof(fpr));
    memcpy(t0_copy, target_t0, n * sizeof(fpr));
    fpr *f0_g = calloc(n/2, sizeof(fpr));
    fpr *f1_g = calloc(n/2, sizeof(fpr));
    Zf(poly_split_fft)(f0_g, f1_g, t0_copy, logn);

    printf("\n--- Golden (poly_split_fft) ---\n");
    printf("f0[0]: re=%016llx im=%016llx\n",
        (unsigned long long)fpr_bits(f0_g[0]), (unsigned long long)fpr_bits(f0_g[qn]));
    printf("f1[0]: re=%016llx im=%016llx\n",
        (unsigned long long)fpr_bits(f1_g[0]), (unsigned long long)fpr_bits(f1_g[qn]));

    // === Manual computation ===
    fpr sum_re = fpr_add(a_re, b_re);
    fpr sum_im = fpr_add(a_im, b_im);
    fpr f0m_re = fpr_half(sum_re);
    fpr f0m_im = fpr_half(sum_im);

    fpr diff_re = fpr_sub(a_re, b_re);
    fpr diff_im = fpr_sub(a_im, b_im);
    // RTL: rot = diff * conj(gm)
    // rot_re = diff_re*gm_re + diff_im*gm_im  (conj: im -> -gm_im, so -diff_im*(-gm_im)?)
    // Actually: conj(gm) = (gm_re, -gm_im)
    // diff * conj(gm) = (diff_re + i*diff_im) * (gm_re - i*gm_im)
    // = (diff_re*gm_re + diff_im*gm_im) + i*(diff_im*gm_re - diff_re*gm_im)
    fpr rot_re = fpr_add(fpr_mul(diff_re, gm_re), fpr_mul(diff_im, gm_im));
    fpr rot_im = fpr_sub(fpr_mul(diff_im, gm_re), fpr_mul(diff_re, gm_im));
    fpr f1m_re = fpr_half(rot_re);
    fpr f1m_im = fpr_half(rot_im);

    printf("\n--- Manual (a+b)/2, (a-b)*conj(gm)/2 ---\n");
    printf("f0: re=%016llx im=%016llx\n",
        (unsigned long long)fpr_bits(f0m_re), (unsigned long long)fpr_bits(f0m_im));
    printf("f1: re=%016llx im=%016llx\n",
        (unsigned long long)fpr_bits(f1m_re), (unsigned long long)fpr_bits(f1m_im));

    // Check match
    printf("\n--- Match check ---\n");
    printf("f0: %s\n", (fpr_bits(f0m_re)==fpr_bits(f0_g[0]) && fpr_bits(f0m_im)==fpr_bits(f0_g[qn])) ? "MATCH" : "MISMATCH");
    printf("f1: %s\n", (fpr_bits(f1m_re)==fpr_bits(f1_g[0]) && fpr_bits(f1m_im)==fpr_bits(f1_g[qn])) ? "MATCH" : "MISMATCH");

    // === RTL input/output hex files ===
    FILE *fin = fopen("split_test_input.hex", "w");
    fprintf(fin, "%016llx%016llx%016llx%016llx\n",
        0ULL, 0ULL,
        (unsigned long long)fpr_bits(a_im), (unsigned long long)fpr_bits(a_re));
    fprintf(fin, "%016llx%016llx%016llx%016llx\n",
        0ULL, 0ULL,
        (unsigned long long)fpr_bits(b_im), (unsigned long long)fpr_bits(b_re));
    for (int i = 2; i < 512; i++)
        fprintf(fin, "0000000000000000000000000000000000000000000000000000000000000000\n");
    fclose(fin);

    FILE *fexp = fopen("split_test_expected.hex", "w");
    // f0 at word 0
    fprintf(fexp, "%016llx%016llx%016llx%016llx\n",
        0ULL, 0ULL,
        (unsigned long long)fpr_bits(f0m_im), (unsigned long long)fpr_bits(f0m_re));
    // f1 at word 128 (pair_limit for root)
    for (int i = 1; i < 128; i++)
        fprintf(fexp, "0000000000000000000000000000000000000000000000000000000000000000\n");
    fprintf(fexp, "%016llx%016llx%016llx%016llx\n",
        0ULL, 0ULL,
        (unsigned long long)fpr_bits(f1m_im), (unsigned long long)fpr_bits(f1m_re));
    for (int i = 129; i < 512; i++)
        fprintf(fexp, "0000000000000000000000000000000000000000000000000000000000000000\n");
    fclose(fexp);

    printf("\nFiles: split_test_input.hex, split_test_expected.hex\n");

    free(ek); free(tmp); free(tmp_cp); free(t0);
    free(target_t0); free(target_t1);
    free(t0_copy); free(f0_g); free(f1_g);
    return 0;
}
