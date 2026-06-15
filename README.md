# 面向大飞机装配的 AGV 移载协同控制系统

本仓库对应《网络系统与控制》课程大作业。  
实现内容覆盖：离散建模、MSS/LMI 控制器求解、单AGV与12AGV在丢包+固定时延下的仿真分析，以及报告生成。

---

## 目录结构与文件功能

## 顶层文件

- `main.tex`  
  最终报告（课程提交主文档）。
- `agv_mss_solution.mat`  
  控制器求解结果缓存（含 `K_delay_state`、模型参数等）。

## `src/`（核心算法函数）

- `build_single_agv_discrete.m`  
  单AGV连续模型离散化（得到 `Ad, Bd`）。
- `build_stacked_multiagv_model.m`  
  多AGV块对角堆叠模型。
- `build_augmented_packet_delay_model.m`  
  构建“丢包+固定时延+输入保持”的增广模型。
- `solve_mss_lmi_yalmip.m`  
  MSS-LMI 求解（YALMIP接口）。
- `verify_mss_fixedK_yalmip.m`  
  固定K下MSS可行性验证。
- `simulate_single_agv_network.m`  
  单AGV网络化闭环时域仿真。
- `compute_performance_metrics.m`  
  指标计算（RMSE、decay ratio、settle time等）。
- `setup_solver_paths.m`  
  求解器路径配置。

## `scripts/`（可执行流程脚本）

### 依赖安装
- `install_yalmip_and_solvers.m`：安装 YALMIP/SDPT3/SeDuMi。
- `install_sedumi_release.m`：安装 SeDuMi release 预编译版本。

### 理论与控制器求解
- `run_agv_mss_demo.m`：联合LMI求解主流程（可能数值不稳）。
- `run_agv_mss_fixedK_search.m`：固定结构K搜索 + MSS验证（当前主用）。

### 单AGV（Matlab脚本）
- `run_step6_single_scan.m`：基线 + 小范围 `p,d` 扫描。
- `run_step7_export_network_scan.m`：导出CSV与热力图。
- `run_step8_single_scan_wide.m`：宽范围 `p,d` 扫描。

### 12AGV（Matlab脚本）
- `run_step6_multi12_demo.m`：12AGV并行基线。
- `run_step9_multi12_coop_demo.m`：协同+安全项版本。
- `run_step10_compare_multi12_step6_step9.m`：Step6 vs Step9 对比。
- `run_step10b_compare_matched_on_step9_scenario.m`：同场景公平对比。
- `run_step11_tune_coop_safety.m`：协同参数与安全阈值联合调参。

### Simulink流程
- `init_simulink_12agv_workspace.m`：Simulink工作区变量一键初始化。
- `run_simulink_single_scan.m`：单AGV Simulink 单参数扫描。
- `run_simulink_pd_scan.m`：单AGV Simulink `p-d` 扫描。
- `run_simulink_12agv_check.m`：12AGV Simulink 快速验收。
- `run_simulink_12agv_pd_scan.m`：12AGV Simulink `p-d` 扫描与曲线导出。

## `slimulink/`（模型）

- `single_agv_baseline.slx`：单AGV网络化闭环模型。
- `x12_agv.slx`：12AGV并行扩展模型。

## `data/`（结果表）

包含单AGV/12AGV的扫描统计表与对比结果表，例如：
- `single_scan_metrics.csv`
- `single_scan_wide_metrics.csv`
- `simulink_pd_scan_summary.csv`
- `simulink_12agv_pd_scan_summary.csv`
- `compare_matched_step9_scenario_summary.csv`

## `pic/` 与 `pic/simulink_pic/`（图像）

报告中使用的全部曲线图与热力图（单AGV、12AGV、公平对比、Simulink扫描等）。

---

## 完整执行流程（按报告章节）

以下流程与报告结构一致：  
**理论问题解答 → 普通算法部分 → 单AGV仿真 → 12AGV仿真**

## 0. 环境准备

在 Matlab 中进入项目根目录：

```matlab
cd('d:\大三下\网络\大作业3')
```

首次运行可安装依赖：

```matlab
run('scripts/install_sedumi_release.m')
run('scripts/install_yalmip_and_solvers.m')
```

---

## 1. 理论问题解答（模型 + MSS + LMI）

### 1.1 求控制器

优先执行：

```matlab
run('scripts/run_agv_mss_fixedK_search.m')
```

输出：
- `agv_mss_solution.mat`
- 可行 `K_delay_state`

> 若希望尝试联合求解，可额外运行 `run_agv_mss_demo.m`。

---

## 2. 普通算法部分（脚本化流程与参数扫描）

### 2.1 单AGV脚本扫描

```matlab
run('scripts/run_step6_single_scan.m')
run('scripts/run_step7_export_network_scan.m')
run('scripts/run_step8_single_scan_wide.m')
```

输出：
- `data/single_scan_metrics.csv`
- `data/single_scan_wide_metrics.csv`
- 对应热力图（`pic/`）

### 2.2 12AGV脚本流程（协同与安全）

```matlab
run('scripts/run_step6_multi12_demo.m')
run('scripts/run_step9_multi12_coop_demo.m')
run('scripts/run_step10b_compare_matched_on_step9_scenario.m')
run('scripts/run_step11_tune_coop_safety.m')
```

输出：
- 公平对比结果 `data/compare_matched_step9_scenario_summary.csv`
- 调参结果 `data/step11_tune_coop_safety_results.csv`

---

## 3. 单AGV仿真（Simulink）

### 3.1 初始化变量

```matlab
run('scripts/init_simulink_12agv_workspace.m')
```

### 3.2 运行扫描

```matlab
run('scripts/run_simulink_single_scan.m')
run('scripts/run_simulink_pd_scan.m')
```

输出：
- `data/simulink_single_scan_summary.csv`
- `data/simulink_pd_scan_summary.csv`
- 图像保存在 `pic/simulink_pic/`

---

## 4. 12AGV仿真（Simulink）

### 4.1 快速验收

```matlab
run('scripts/run_simulink_12agv_check.m')
```

检查：
- `x_all` 维度应为 `N x 48`
- `ua_all` 维度应为 `N x 24`
- `gamma_all` 维度应为 `N x 12`

### 4.2 `p-d` 扫描

```matlab
run('scripts/run_simulink_12agv_pd_scan.m')
```

输出：
- `data/simulink_12agv_pd_scan_summary.csv`
- 状态/控制时域曲线与热力图（`pic/simulink_pic/`）

---

## 报告生成

- 最终报告：`main.tex`
- 阶段报告：`pic/pre_simulink_report.tex`

推荐编译方式：XeLaTeX（支持中文）。

---

## 结果对照建议（提交前自检）

- 理论部分：是否给出明确 `K` 数值、MSS可行性与LMI说明。
- 单AGV：是否有 `p-d` 变化结论 + 状态/控制时域曲线。
- 12AGV：是否有并行基线、协同对比、安全性结论、`p-d` 扫描。
- 数据与图片：`data/*.csv` 与 `pic/*.png` 是否与 `main.tex` 引用一致。
