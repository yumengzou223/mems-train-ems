# MEMS 多目标帕累托交接（2026-08-26）

## 一、本轮做了什么

承接 8/25 交接“下一步 1：多目标帕累托（TLC vs FC 退化/更换 vs 电池退化/更换 vs H2）”，
用降阶 Python 模型跑出 4 目标帕累托前沿。

### 校准模型修正（把 mrfo_calibrated.py 升级到最终口径）
- A_res：10500 → **7500**
- 功率平衡补 DC/DC 92%：`P_bat = P_total / 0.92 - P_fc`
- 电池电网电费改净放电：`E_grid = max(0, SOC_start - SOC_end) * C_kWh / 0.975`
- 保留：牵引效率 0.70、再生效率 0.40、齿轮箱 0.96、FC 732/1367/stoch1.05、
  电池 1178.61Ah/1500V、Wang 退化公式、FC 四指标退化、维护费 CAPEX×2%。

### 基线校验（论文 EMS1，x=[0.30,0.70,1.0,200kW/820.3kW,0.90]）
| 指标 | Python 降阶 | 最终 MATLAB | 论文 EMS1 |
|---|---:|---:|---:|
| TLC | 9.46M | 9.32M | 9.19M |
| H2 kg | 106.62 | 106.00 | 105.8 |
| FC 更换 | 1.95 | 1.95 | 2.5 |
| 电池更换 | 7.32 | 7.54 | 1.57(口径矛盾) |
| SOC end | 0.522 | 0.555 | ~0.40 |

偏差 ~1.5%，来自降阶模型（dt=0.5 采样、无 RC 电池动态/无闭环动力学）。
对帕累托相对权衡足够可信。

## 二、多目标优化设置

- 决策变量（5 维，同单目标 MRFO）：
  SOC_low∈[0.20,0.35]、SOC_high∈[0.60,0.85]、FC_nom_scale∈[0.85,1.10]、
  FC_idle_scale∈[0.15,0.35]、SOC_regen∈[0.80,0.95]。
- 目标（均最小化）：TLC、N_rep_FC、N_rep_BAT、H2。
- 算法：MO-MRFO —— MRFO 链式/螺旋觅食 + 翻腾觅食生成候选，NSGA-II
  非支配排序 + 拥挤距离做环境选择。pop=40、gen=30、seed=0。

## 三、关键结果

40 个非支配解（`pareto_front.csv`）。两端极值：

| 端 | TLC | FC 更换 | 电池更换 | H2 kg | 主要参数 |
|---|---:|---:|---:|---:|---|
| 低 TLC/低 H2 端 | 8.78M | 1.47 | 7.78 | 87.6 | FC_nom=697kW, SOC_high=0.60 |
| 高 H2 端 | 9.89M | 3.61 | 6.93 | 115.6 | FC_nom=902kW, SOC_high=0.60 |

基线 EMS1（9.46M / 1.95 / 7.32 / 106.6）被帕累托前沿支配。

### 结论
1. 论文 EMS1 非最优：降 FC 标称功率、早切 idle 能同时省氢、降 TLC，
   代价是电池循环略增（换 7.3→7.8）。
2. 主导权衡轴是 **TLC/H2/FC 更换 vs 电池更换**；主导决策变量是 **FC_nom**，
   SOC_high 次之，SOC_low/FC_idle/SOC_regen 影响较小。
3. 4 目标在 4 维下互相非支配点很多（维度诅咒），实际主要结构近似一条
   2 目标前沿（H2/TLC 强正相关，与电池更换弱冲突）。

## 四、产出文件（工作区）
- `mrfo_multiobjective.py` —— 校准多目标优化脚本（numba 加速）
- `pareto_front.csv` —— 40 个非支配解（决策变量 + 4 目标）
- `pareto_scatter.png` —— 4×4 目标两两投影散点矩阵
- `pareto_parallel.png` —— 归一化平行坐标图

## 五、可选下一步
1. 把 2–3 个帕累托代表点回填 Simulink 做最终验证（当前 Python 偏差 ~1.5%）。
2. 若想论文更“干净”，可把 4 目标降为 3 目标或 2 目标（TLC vs 电池退化），
   前沿会更聚焦；或用 ɛ-约束法生成更均匀的帕累托集。
3. 把 FC_nom 的上下界再放宽（<0.85 或 >1.10）确认最优端未贴边界。
4. 可选修 `run_sim_with_degradation.m` 里 `save('F:\MEMS\sim_results.mat')` 的
   `\M` 转义警告。

## 六、Simulink 回填验证（2026-08-26 追加）

把帕累托前沿的 3 个代表点回填到真实 Simulink 模型，验证 Python 降阶代理的准确性。

### 方法
- 把 `build_simulink_model_v5.m` 由脚本改成函数 `build_simulink_model_v5(EMS_OVERRIDE)`，
  支持覆盖 SOC_low / SOC_high / FC_nom / FC_idle / SOC_regen（默认仍是论文 EMS1）。
- 新增 `verify_pareto.m`：对每个点重建+仿真 Simulink，按与 Python 完全一致的口径
  重算 TLC/FC 更换/电池更换，输出对照表。

### 结果（Python 代理 vs Simulink）
| 点 | H2_py | H2_sim | TLC_py | TLC_sim | FC_py | FC_sim | Bat_py | Bat_sim |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Low TLC | 89.61 | 88.90 | 8.86M | 8.70M | 1.46 | 1.49 | 7.72 | 7.73 |
| Mid | 95.90 | 95.50 | 9.09M | 8.94M | 1.47 | 1.51 | 7.54 | 7.55 |
| High H2 | 115.58 | 111.62 | 9.68M | 9.42M | 1.62 | 1.69 | 6.92 | 6.98 |

### 结论
- 代理模型可信：电池更换偏差 <0.5%，H2/TLC 偏差约 1–3.5%（代理略偏高，高 FC_nom 端更明显，
  因 Simulink 闭环动力学与功率限幅在高功率端差异更大）。
- 帕累托排序与主导权衡结构在 Simulink 中成立：Low TLC 点 TLC/H2 最低、High H2 点最高。
- 后续若需要论文级精度，可用 Simulink 只在最终选定的 2–3 个候选上重算即可，
  不必整条前沿都走 Simulink。
