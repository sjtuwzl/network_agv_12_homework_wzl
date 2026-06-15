# Simulink协同控制修改指南

## 方法一：使用MATLAB脚本自动添加（推荐）

### 步骤1：运行脚本
```matlab
run('scripts/add_coop_to_simulink.m')
```

这会自动在每个AGV子系统中添加：
- `coop_repulsion` MATLAB Function块
- 参数常量块（kc_pos, kc_vel, d_safe, k_rep）
- `u_saturation` 饱和块
- `sum_coop` 求和块

### 步骤2：手动完成连线

打开Simulink模型 `x12_agv.slx`，对每个AGV子系统：

#### 2.1 添加输入端口
双击进入 `agv_1` 子系统，添加2个Inport块：
- `nb_left_state` (4维，接收左邻居状态)
- `nb_right_state` (4维，接收右邻居状态)

#### 2.2 连接协同计算
在子系统内：
1. 从 `x_out` 分出一路信号，提取位置 `p_i = x(1:2)` 和速度 `v_i = x(3:4)`
2. 将 `nb_left_state` 和 `nb_right_state` 也提取位置/速度
3. 连接到 `coop_repulsion` 函数的输入
4. 函数输出 `[u_coop, u_rep]` 连接到 `sum_coop`
5. `sum_coop` 输出连接到 `u_saturation`
6. `u_saturation` 输出连接到 `plant` 的输入

#### 2.3 在根层级连线
回到根层级，对每个AGV：
- AGV1的 `x_out` → AGV2和AGV4的 `nb_left_state` / `nb_right_state`
- AGV2的 `x_out` → AGV1和AGV3的输入
- ...（按环形拓扑连接）

### 步骤3：测试
```matlab
run('scripts/init_simulink_12agv_workspace.m')
out = sim('x12_agv', 'StopTime', '400');
```

---

## 方法二：手动在Simulink中添加

### 步骤1：打开模型
```matlab
open_system('slimulink/x12_agv.slx')
```

### 步骤2：修改单个AGV子系统
1. 双击 `agv_1` 进入子系统
2. 添加MATLAB Function块，代码见下方
3. 添加参数常量块
4. 添加Sum块和Saturation块
5. 添加Inport块接收邻居状态

### MATLAB Function代码
```matlab
function [u_coop, u_rep] = coop_repulsion(p_i, v_i, p_j1, v_j1, p_j2, v_j2, ...
    p_ref_i, p_ref_j1, p_ref_j2, kc_pos, kc_vel, d_safe, k_rep)

% 协同项
e_rel_p1 = (p_i - p_j1) - (p_ref_i - p_ref_j1);
e_rel_v1 = (v_i - v_j1);
e_rel_p2 = (p_i - p_j2) - (p_ref_i - p_ref_j2);
e_rel_v2 = (v_i - v_j2);
u_coop = -kc_pos * (e_rel_p1 + e_rel_p2) - kc_vel * (e_rel_v1 + e_rel_v2);

% 斥力项（简化：只对邻居）
u_rep = zeros(2,1);
for j = 1:2
    if j == 1
        dp = p_i - p_j1;
    else
        dp = p_i - p_j2;
    end
    dij = norm(dp);
    if dij < d_safe && dij > 1e-6
        dir = dp / dij;
        u_rep = u_rep + k_rep * (d_safe - dij) * dir;
    end
end
end
```

### 步骤3：复制到其他AGV
修改完 `agv_1` 后，可以复制子系统结构到其他11个AGV，只需修改邻居连接。

### 步骤4：根层级连线
在根层级，用Mux/Demux块实现环形拓扑连接：
- AGV1 ↔ AGV2 ↔ AGV3 ↔ AGV4 ↔ AGV1（组1）
- AGV5 ↔ AGV6 ↔ AGV7 ↔ AGV8 ↔ AGV5（组2）
- AGV9 ↔ AGV10 ↔ AGV11 ↔ AGV12 ↔ AGV9（组3）

---

## 方法三：保持现状（最快）

如果时间紧张，可以：
1. 脚本仿真已完成协同验证（含延迟对比）
2. Simulink验证了网络通道效应
3. 报告里说明两者互补

**这个方案不需要改Simulink模型。**

---

## 建议

**如果你时间充裕**：用方法一（脚本自动添加）+ 手动连线

**如果你时间紧张**：用方法三（保持现状），报告里写清楚即可

你想用哪个方法？
