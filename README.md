# 可重构 PQC 公共 RTL 模块说明

本目录包含一版面向 `CTRU / NEV / Falcon / HAWK` 的公共硬件模块实现。当前代码的目标不是一次性实现完整算法，而是先抽取这些算法在硬件实现中的可复用资源，形成“共享主干 + 可切换尾部 + 专用旁路”的可重构 RTL 基础。

## 设计定位

这些模块用于支撑后续完整算法实现中的公共数据通路：

- `CTRU / NEV` 可复用 KEM 路线中的采样、存储、多项式乘法、模约减和解码接口。
- `Falcon / HAWK` 可复用签名验证路线中的多项式算术、范数检查和公开关系检查接口。
- `Falcon signer` 这类高度专用路径后续应作为旁路接入，而不是强行并入共享主干。

当前实现优先保证：

- 纯 Verilog-2001 兼容
- 模块接口清晰
- 参数可配置
- 可仿真验证
- 便于后续替换为高性能 NTT / Barrett / Montgomery / 专用采样器

## 目录结构

```text
SRC/
  README.md
  rtl/
    common/
      pqc_addr_gen.v
      pqc_e8_radius_check.v
      pqc_mod_add_sub.v
      pqc_mod_mul_reduce.v
      pqc_norm_check.v
      pqc_ntt_reconfig_wrapper.v
      pqc_poly_mul_schoolbook.v
      pqc_vector_vote_decode.v
    falcon/
      falcon_fft_addr_gen.v
      falcon_fft_complex_bfly.v
      falcon_fft_exu.v
  tb/
    tb_pqc_common_modules.v
    tb_falcon_fft_complex_bfly.v
```

## 共享主干模块

### `pqc_mod_add_sub.v`

模加 / 模减单元，用于系数域公共算术。

功能：

- 输入两个模 `q` 系数 `a, b`
- 根据 `sub` 选择执行加法或减法
- 输出仍保持在 `[0, q)` 范围内

可复用位置：

- `CTRU / NEV` 的多项式系数加减
- `Falcon / HAWK` 验证侧的公开算术
- 压缩、重构、中心化之前的基础运算

### `pqc_mod_mul_reduce.v`

模乘与约减单元。

当前实现使用 `% MODULUS` 做紧凑参考实现，适合架构验证与功能仿真。后续若面向 FPGA / ASIC 高性能实现，建议替换为：

- Barrett reduction
- Montgomery reduction
- 针对固定模数的专用约减器

可复用位置：

- 多项式乘法内部的系数乘法
- NTT 点值域乘法
- 签名验证中的公开多项式乘法

### `pqc_poly_mul_schoolbook.v`

顺序 schoolbook 多项式乘法器，是当前共享算术主干的基线版本。

支持两类环：

- `RING_MODE = 0`：
  `Z_q[x] / (x^N + 1)`

- `RING_MODE = 1`：
  `Z_q[x] / (x^N - x^(N/2) + 1)`

设计用途：

- 作为不启用 NTT / pNTT 时的 fallback 数据通路
- 作为后续 NTT 版本的功能参考模型
- 支撑小参数仿真和早期集成

注意：

- 当前实现偏正确性基线，不是最终高吞吐实现。
- 大参数场景下应替换或并联为 NTT / pNTT 数据通路。

### `pqc_addr_gen.v`

地址发生器，用于统一存储访问和变换域遍历。

支持模式：

- `mode = 0`：直接顺序地址
- `mode = 1`：NTT butterfly pair 地址
- `mode = 2`：partial NTT / segmented 地址

设计用途：

- 统一 BRAM / FIFO / 多项式存储访问
- 为 full NTT、partial NTT 和直接卷积提供同一类地址接口
- 为后续 twiddle ROM、NTT butterfly、pNTT split/merge 模块提供调度基础

注意：

- 当前模块只产生地址，不包含 twiddle factor 查表。
- NTT root 表和 stage-specific 参数应在外部模块接入。

### `pqc_ntt_reconfig_wrapper.v`

面向后续高性能 NTT / pNTT 核心的顶层兼容 wrapper。

设计定位：

- 对外使用 `256-bit` 内存数据口。
- 通过 `cfg_precision` 支持 `8 / 16 / 32 / 64 / 128 / 256 bit` 系数粒度。
- 通过 `cfg_precision = 7` 支持自动模式：`KEM` 默认 `16 lanes x 16 bit`，`DSA` 默认 `8 lanes x 32 bit`。
- 通过 `cfg_transform_mode` 支持 `direct / full NTT / partial NTT` 遍历。
- 输出 `core_in_lane_count`、`core_in_coeff_width`、`core_in_lane_mask`，供后级 butterfly / twiddle / reduction core 使用。

该模块本身不实现 butterfly、twiddle ROM 或模约减。它的作用是把统一内存接口、地址调度和 lane 格式适配成一个稳定的 NTT core 接口，便于后续替换不同算法和不同精度的 NTT 数据通路。

## 算法参数与可重构复用关系

### 多项式乘法是否使用 NTT

当前 `pqc_poly_mul_schoolbook.v` 是正确性基线，不是最终高性能 NTT 乘法器。面向完整实现时，不同算法应按环和模数选择 full NTT、partial NTT 或专用旁路：

- `CTRU / CNTR`：应使用 NTT。其环为 `Z_q[x] / (x^n - x^(n/2) + 1)`，`q = 3457`，`n = 512 / 768 / 1024`。其中 `n = 512` 和 `n = 1024` 可以采用 radix-2 NTT 思路，`n = 768` 需要 mixed-radix 或统一 NTT 调度。因此硬件上应复用 `pqc_addr_gen.v` 的 NTT 地址调度，再接 `NTT butterfly + twiddle ROM + 模乘约减`。
- `NEV / NEV'`：不适合直接在 `Z_q[x] / (x^n + 1)` 上做 full NTT。原因是 `q = 769` 较小，对 `n = 512 / 1024` 不能直接满足完整 negacyclic NTT 所需的根条件。实现上应使用 partial NTT / split-then-NTT，把大多项式拆成较小子多项式后做变换域乘法。因此硬件上应使用 `pqc_addr_gen.v` 的 `mode = 2` 作为 pNTT / segmented 调度入口。
- `Falcon verify`：可使用 NTT。Falcon 使用 `q = 12289`，环为 `Z_q[x] / (x^n + 1)`，`n = 512 / 1024`。验证过程中的 `s1 = c - s2 * h mod q` 适合用 NTT/INTT 加速。注意 Falcon 签名侧不是简单 NTT 主干，主要还需要 FFT、LDL tree 和离散高斯采样旁路。
- `HAWK`：算法主体不是以一个公开小模数 `q` 上的 NTT 环来定义的，而是在整数环 `R_n = Z[x] / (x^n + 1)` 上做签名和验证。工程实现中，为了加速若干多项式乘法，可使用辅助 NTT 模数 `p = 18433`，规范也提到 `p = 12289` 和 `p = 18433` 都适合所有指定 HAWK 参数集；参考和优化实现使用 `p = 18433`。因此硬件复用时应把 HAWK 的 NTT 看成“实现辅助模数”，不能把它误写成算法公钥模数。

结论：多项式乘法主干应该设计成可切换的 `schoolbook fallback / full NTT / partial NTT / 专用旁路`。当前 RTL 已经提供了 schoolbook baseline 和地址调度原语；下一步真正高性能实现需要补 `NTT butterfly`、`twiddle ROM`、`INTT scaling`、`pNTT split/merge` 和固定模数约减器。

### 参数集合

| 算法 | 参数集 | 环 / 结构 | 主模数或辅助模数 | 维度 `n` | 安全定位 / 备注 |
| --- | --- | --- | --- | --- | --- |
| `CTRU` | `CTRU-512` | `Z_q[x] / (x^n - x^(n/2) + 1)` | `q = 3457`，`q2 = 2^10` | `512` | KEM，约 NIST-I；`(Psi1, Psi2) = (B3, B3)`，`PK/CT = 768/640 bytes`，安全估计 `(118,107)`，失败率 `2^-143` |
| `CTRU` | `CTRU-768` | `Z_q[x] / (x^n - x^(n/2) + 1)` | `q = 3457`，`q2 = 2^10` | `768` | KEM，约 NIST-III；`(Psi1, Psi2) = (B2, B2)`，`PK/CT = 1152/960 bytes`，安全估计 `(181,164)`，失败率 `2^-184`，实现常用 mixed-radix NTT |
| `CTRU` | `CTRU-1024` | `Z_q[x] / (x^n - x^(n/2) + 1)` | `q = 3457`，`q2 = 2^11` | `1024` | KEM，约 NIST-V；`(Psi1, Psi2) = (B2, B2)`，`PK/CT = 1536/1408 bytes`，安全估计 `(255,231)`，失败率 `2^-195` |
| `CNTR` | `CNTR-512` | `Z_q[x] / (x^n - x^(n/2) + 1)` | `q = 3457`，`q2 = 2^10` | `512` | CTRU 的简化 / RLWR 路线；`(Psi1, Psi2) = (B5, B5)`，`PK/CT = 768/640 bytes`，安全估计 `(127,115)`，失败率 `2^-170` |
| `CNTR` | `CNTR-768` | `Z_q[x] / (x^n - x^(n/2) + 1)` | `q = 3457`，`q2 = 2^10` | `768` | 论文重点实现参数；`(Psi1, Psi2) = (B3, B3)`，`PK/CT = 1152/960 bytes`，安全估计 `(191,173)`，失败率 `2^-230` |
| `CNTR` | `CNTR-1024` | `Z_q[x] / (x^n - x^(n/2) + 1)` | `q = 3457`，`q2 = 2^10` | `1024` | 约 NIST-V；`(Psi1, Psi2) = (B2, B2)`，`PK/CT = 1536/1280 bytes`，安全估计 `(253,230)`，失败率 `2^-291` |
| `NEV` | `NEV-512` | `Z_q[x] / (x^n + 1)` | `q = 769` | `512` | KEM；`(chi_f, chi_g) = (B1, B1)`，`(chi_r, chi_e) = (B1, T1/6)`，`PK/CT = 615/615 bytes`，失败率 `2^-138`，安全估计 `(145,141)` |
| `NEV'` | `NEV'-512` | `Z_q[x] / (x^n + 1)` | `q = 769` | `512` | KEM；`(chi_f, chi_g) = (B1, B1)`，`(chi_r, chi_e) = (B1, B1)`，`PK/CT = 615/615 bytes`，失败率 `2^-200`，安全估计 `(145,145)` |
| `NEV` | `NEV-1024` | `Z_q[x] / (x^n + 1)` | `q = 769` | `1024` | KEM；`(chi_f, chi_g) = (B1, B1)`，`(chi_r, chi_e) = (B1, T1/6)`，`PK/CT = 1229/1229 bytes`，失败率 `2^-152`，安全估计 `(292,281)` |
| `NEV'` | `NEV'-1024` | `Z_q[x] / (x^n + 1)` | `q = 769` | `1024` | KEM；`(chi_f, chi_g) = (B1, B1)`，`(chi_r, chi_e) = (B1, B1)`，`PK/CT = 1229/1229 bytes`，失败率 `2^-200`，安全估计 `(292,292)` |
| `Falcon` | `Falcon-512` | `Z_q[x] / (x^n + 1)` | `q = 12289` | `512` | 签名，NIST-I；`sigma = 165.736617183`，`sigma_min = 1.277833697`，`sigma_max = 1.8205`，公钥 `897 bytes`，签名 `666 bytes`，验证平方范数界约 `34034726` |
| `Falcon` | `Falcon-1024` | `Z_q[x] / (x^n + 1)` | `q = 12289` | `1024` | 签名，NIST-V；`sigma = 168.388571447`，`sigma_min = 1.298280334`，`sigma_max = 1.8205`，公钥 `1793 bytes`，签名 `1280 bytes`，验证平方范数界约 `70265242` |
| `HAWK` | `HAWK-256` | `Z[x] / (x^n + 1)` | 实现辅助 `p = 18433`，可选 `p = 12289` | `256` | Challenge 参数；`eta = 2`，`sigma_sign = 1.010`，`sigma_verify = 1.042`，`saltlenbits = 112`，私钥/公钥/签名 `96/450/249 bytes` |
| `HAWK` | `HAWK-512` | `Z[x] / (x^n + 1)` | 实现辅助 `p = 18433`，可选 `p = 12289` | `512` | 签名，NIST-I；`eta = 4`，`sigma_sign = 1.278`，`sigma_verify = 1.425`，`saltlenbits = 192`，私钥/公钥/签名 `184/1024/555 bytes` |
| `HAWK` | `HAWK-1024` | `Z[x] / (x^n + 1)` | 实现辅助 `p = 18433`，可选 `p = 12289` | `1024` | 签名，NIST-V；`eta = 8`，`sigma_sign = 1.299`，`sigma_verify = 1.571`，`saltlenbits = 320`，私钥/公钥/签名 `360/2440/1221 bytes` |

以上参数来自 `DOC/` 中 CTRU、NEV、Falcon、HAWK 论文 / 规范文本；其中 `q2` 是 CTRU/CNTR 的压缩 / 舍入模数，随参数集可取 `2^10` 或 `2^11`，HAWK 的 `p = 18433` 是工程实现辅助 NTT 模数，不是算法定义中的公钥模数。

### RTL 参数映射

| 算法路径 | 多项式乘法配置 | 模算术配置 | 地址调度配置 | 尾部模块 |
| --- | --- | --- | --- | --- |
| `CTRU-512 / CNTR-512` | `N = 512`，`RING_MODE = 1`，当前可用 `pqc_poly_mul_schoolbook.v` 做 baseline，最终应换 full NTT | `MODULUS = 3457`，`COEFF_WIDTH >= 12` | `pqc_addr_gen.v`：`mode = 1`；NTT stage 按 `512` 点配置 | `pqc_e8_radius_check.v` 接入可扩展 E8 decode tail |
| `CTRU-768 / CNTR-768` | `N = 768`，`RING_MODE = 1`，最终应使用 mixed-radix NTT 或统一 NTT | `MODULUS = 3457`，`COEFF_WIDTH >= 12` | `mode = 1`，但需要支持 radix-2 + radix-3 或统一 NTT 的分段调度 | `pqc_e8_radius_check.v`，解码阈值和半径由 CTRU/CNTR 参数决定 |
| `CTRU-1024 / CNTR-1024` | `N = 1024`，`RING_MODE = 1`，最终应使用 full NTT | `MODULUS = 3457`，`COEFF_WIDTH >= 12` | `mode = 1`；NTT stage 按 `1024` 点配置 | `pqc_e8_radius_check.v` |
| `NEV-512 / NEV'-512` | `N = 512`，`RING_MODE = 0`，最终应使用 partial NTT，不建议直接 full NTT | `MODULUS = 769`，`COEFF_WIDTH >= 10` | `mode = 2`；`SEG_SIZE` 按 pNTT 拆分方案配置 | `pqc_vector_vote_decode.v`：`BIT_COUNT = 256`，`REPEAT_COUNT = n / 256 = 2` |
| `NEV-1024 / NEV'-1024` | `N = 1024`，`RING_MODE = 0`，最终应使用 partial NTT | `MODULUS = 769`，`COEFF_WIDTH >= 10` | `mode = 2`；`SEG_SIZE` 按 pNTT 拆分方案配置 | `pqc_vector_vote_decode.v`：`BIT_COUNT = 256`，`REPEAT_COUNT = n / 256 = 4` |
| `Falcon-512 verify` | `N = 512`，`RING_MODE = 0`，验证乘法 `s2 * h` 适合 full NTT | `MODULUS = 12289`，`COEFF_WIDTH >= 14` | `mode = 1`；NTT stage 按 `512` 点配置 | `pqc_norm_check.v`：`COEFF_COUNT = 2N`，`BOUND_SQ ≈ 34034726` |
| `Falcon-1024 verify` | `N = 1024`，`RING_MODE = 0`，验证乘法 `s2 * h` 适合 full NTT | `MODULUS = 12289`，`COEFF_WIDTH >= 14` | `mode = 1`；NTT stage 按 `1024` 点配置 | `pqc_norm_check.v`：`COEFF_COUNT = 2N`，`BOUND_SQ ≈ 70265242` |
| `Falcon signer` | 不建议并入通用 NTT 主干 | `q = 12289` 仍会出现，但核心是 FFT/LDL/采样 | 专用 FFT 存储调度 | 需要专用 Gaussian sampler、FFT/LDL tree、拒绝采样控制 |
| `HAWK-256 verify/sign 辅助乘法` | `N = 256`，`RING_MODE = 0`，如用 NTT 加速则使用辅助模数 | `MODULUS = 18433` 时 `COEFF_WIDTH >= 15`；若选 `12289` 则 `>= 14` | `mode = 1`；NTT stage 按 `256` 点配置 | `pqc_norm_check.v` 只能做 sum-of-squares 原语，完整验证需 Q-范数模块 |
| `HAWK-512 verify/sign 辅助乘法` | `N = 512`，`RING_MODE = 0`，辅助 NTT | `MODULUS = 18433`，`COEFF_WIDTH >= 15` | `mode = 1`；NTT stage 按 `512` 点配置 | 需要扩展 `Q`-范数检查和 `RebuildS0` 相关公开计算 |
| `HAWK-1024 verify/sign 辅助乘法` | `N = 1024`，`RING_MODE = 0`，辅助 NTT | `MODULUS = 18433`，`COEFF_WIDTH >= 15` | `mode = 1`；NTT stage 按 `1024` 点配置 | 需要扩展 `Q`-范数检查和编码边界检查 |

实现时还需要注意以下边界：

- `CTRU / CNTR` 的公钥计算、封装侧公开乘法主要在 `q = 3457` 上工作，可走 NTT 主干；但解封装中的 `c * f mod q2` 若直接落在 `q2 = 2^10 / 2^11` 这种二次幂模数上，不能直接使用同一套 NTT 根表。高性能方案可选择 multi-moduli lifting，例如先提升到 `Q = q * q'` 再约回 `q2`；早期 RTL baseline 可以先走 `pqc_poly_mul_schoolbook.v`。
- `NEV / NEV'` 的消息复制结构由 `v = 1 - x^(n/k)` 触发，其中明文长度固定为 `256 bit`，`k = n / 256`。因此 `n = 512` 时硬件投票复制数为 `2`，`n = 1024` 时复制数为 `4`。
- `Falcon verify` 的 NTT 乘法只覆盖验证侧公开关系，不能替代签名侧的 FFT/LDL/采样路径。若顶层 wrapper 同时支持 verify 和 sign，应把 signer 作为专用旁路挂接，而不是强制复用 `pqc_poly_mul_schoolbook.v`。
- `HAWK` 使用 `p = 18433` 时，`COEFF_WIDTH` 至少取 `15`；软件规范中的参考实现用 Montgomery 表示，硬件可在 `pqc_mod_mul_reduce.v` 后续替换为 Montgomery 或 Barrett 固定模数约减器。

### 按算法过程复用硬件模块

`CTRU / CNTR` 的复用过程：

1. `KeyGen`：生成小系数多项式 `f, g`，计算 `h = g / f`。硬件复用 `pqc_mod_add_sub.v`、`pqc_mod_mul_reduce.v`、多项式乘法 / 求逆数据通路和 NTT 地址调度。
2. `Encaps / Encrypt`：计算公开多项式乘法与加噪或舍入，例如 `h * r + e` 或 CNTR 的舍入路径。硬件复用 NTT 主干、模约减和系数压缩接口。
3. `Decaps / Decrypt`：计算 `c * f`，再进入可扩展 E8 解码路径。硬件复用多项式乘法主干，尾部切换到 `pqc_e8_radius_check.v` 以及后续完整 E8 nearest-codeword decode。

`NEV / NEV'` 的复用过程：

1. `KeyGen`：在 `Z_769[x] / (x^n + 1)` 上生成 `f, g` 并计算公钥关系。由于不能直接 full NTT，硬件应走 pNTT / segmented 乘法调度。
2. `Encaps / Encrypt`：计算 `h * r + e + encode(m)` 类公开乘法和消息嵌入。硬件复用 `MODULUS = 769` 的模加、模乘、partial NTT 地址调度。
3. `Decaps / Decrypt`：先做 `f * c` 或相应恢复计算，再进入复制消息投票判决。硬件尾部切换到 `pqc_vector_vote_decode.v`，其中 `REPEAT_COUNT = 2` 对应 `n = 512`，`REPEAT_COUNT = 4` 对应 `n = 1024`。

`Falcon` 的复用过程：

1. `Verify`：从签名中得到 `s2`，计算挑战 `c`，再计算 `s1 = c - s2 * h mod q`。其中 `s2 * h` 复用 `q = 12289` 的 NTT 乘法主干。
2. `Verify`：对 `(s1, s2)` 做平方范数界检查。硬件尾部切换到 `pqc_norm_check.v`，`COEFF_COUNT = 2N`，`BOUND_SQ` 按 Falcon-512 或 Falcon-1024 参数配置。
3. `Sign / KeyGen`：不建议复用当前公共乘法主干作为核心路径，应设计独立 FFT/LDL/Gaussian sampler 旁路，只在模加、存储、哈希接口和部分公开算术上复用。

`HAWK` 的复用过程：

1. `KeyGen / Sign`：主要工作在整数环 `Z[x] / (x^n + 1)` 和格基结构上，采样器、编码、重建逻辑较专用。若内部多项式乘法用 NTT 加速，可把 `p = 18433` 作为辅助实现模数接入通用 NTT 主干。
2. `Verify`：解码公钥和签名，重建 `s0`，计算公开二次型 / `Q`-范数关系。普通 `pqc_norm_check.v` 只能作为平方和子模块，完整 HAWK 还需要新增 `Q-norm engine`，支持交叉项、公开矩阵系数和 64-bit 中间累加。
3. `Encoding / boundary check`：签名长度、Golomb-Rice 压缩边界、salt 长度等不属于当前算术主干，应作为控制与编码模块单独接入。

## 可切换尾部模块

### `pqc_e8_radius_check.v`

CTRU 路线使用的 E8 风格半径检查单元。

功能：

- 输入 8 个有符号坐标
- 计算平方范数
- 与 `RADIUS_SQ` 比较
- 输出是否落在允许半径内

注意：

- 当前模块是 E8 解码尾部中的“半径接受检查”原语。
- 它不是完整 E8 nearest-codeword 解码器。
- 后续完整 CTRU 解码需要在此基础上接入候选点选择和码字恢复逻辑。

### `pqc_vector_vote_decode.v`

NEV 路线使用的向量聚合判决单元。

功能：

- 每个消息 bit 对应 `REPEAT_COUNT` 个复制系数
- 每个系数与 `THRESHOLD` 比较
- 达到 `MAJORITY_COUNT` 时恢复为 1，否则恢复为 0

可配置参数：

- `BIT_COUNT`
- `REPEAT_COUNT`
- `COEFF_WIDTH`
- `THRESHOLD`
- `MAJORITY_COUNT`

用途：

- 支撑 NEV / NEV' 的复制消息恢复机制
- 后续可替换为更复杂的软判决或加权判决逻辑

### `pqc_norm_check.v`

签名路线使用的平方范数检查单元。

功能：

- 输入一组有符号系数
- 计算平方范数
- 与 `BOUND_SQ` 比较
- 输出是否接受

可复用位置：

- `Falcon verify` 的签名向量长度界检查
- `HAWK verify` 中部分范数 / 边界检查逻辑

注意：

- HAWK 的完整 `Q`-范数检查还需要引入公开矩阵或二次型交叉项。
- 当前模块提供的是最基础的 sum-of-squares 检查原语。

## 测试平台

### `tb/tb_pqc_common_modules.v`

自检 testbench，覆盖以下功能：

- 模加
- 模减
- 模乘
- 向量投票判决
- E8 半径检查
- 范数检查
- 小参数 negacyclic 多项式乘法

运行命令：

```text
iverilog -g2001 -o SRC/tb/tb_pqc_common_modules.vvp \
  SRC/tb/tb_pqc_common_modules.v \
  SRC/rtl/common/pqc_mod_add_sub.v \
  SRC/rtl/common/pqc_mod_mul_reduce.v \
  SRC/rtl/common/pqc_poly_mul_schoolbook.v \
  SRC/rtl/common/pqc_e8_radius_check.v \
  SRC/rtl/common/pqc_vector_vote_decode.v \
  SRC/rtl/common/pqc_norm_check.v

vvp SRC/tb/tb_pqc_common_modules.vvp
```

期望输出：

```text
TB_PASS pqc_common_modules
```

## 已验证情况

当前版本已做过以下检查：

- `check_pure_verilog.py` 纯 Verilog 检查通过
- `iverilog -g2001` 编译通过
- `vvp` 功能仿真通过
- `pqc_addr_gen.v` 已做独立 Verilog-2001 编译检查

## 后续扩展建议

建议后续按以下方向继续扩展：

1. 将 `pqc_mod_mul_reduce.v` 中的 `%` 约减替换为 Barrett 或 Montgomery 约减。

2. 在 `pqc_addr_gen.v` 后接入：
   - NTT butterfly
   - twiddle ROM
   - inverse NTT 缩放逻辑
   - partial NTT split/merge 控制

3. 在 `pqc_e8_radius_check.v` 基础上实现完整 CTRU E8 解码。

4. 在 `pqc_norm_check.v` 基础上扩展 HAWK `Q`-范数检查。

5. 为 Falcon signer 单独增加：
   - FFT / LDL tree 数据通路
   - 离散高斯采样器
   - 拒绝采样控制逻辑

6. 增加顶层可重构 wrapper，通过模式选择信号连接：
   - KEM decode tail
   - signature verify tail
   - signer 专用旁路

## 当前边界

当前 `SRC` 中的代码不是完整的 CTRU / NEV / Falcon / HAWK 实现。它是公共硬件资源的第一版 RTL 基础，用于后续完整算法集成与可重构平台设计。
