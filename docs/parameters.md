# 参数与判定规则

所有参数集中在 `config.go: DefaultConfig()`，可整体替换。下表解释含义、
默认值以及触发后的行为。

## 扫描质量门控（`validateScan`）

| 参数 | 默认 | 含义 | 失败行为 |
|---|---|---|---|
| `MinPoints` | 25 | 最少有效点数 | `too_few_points` 拒绝 |
| `MaxRange`/`MinRange` | 20 / 0.05 | 有效距离区间 | 越界点丢弃 |
| `BadPointFraction` | 0.25 | NaN/Inf/越界点占比上限 | `bad_scan` 拒绝 |
| `MinUniqueRays` | 20 | 不同方位角桶数量（防突发噪声/镜面反射） | `bad_scan` 拒绝 |

故障注入类型（synth）：`few_points`、`nan`、`burst`（同方位突发点）、
`huge`（超量程）。它们会产生**可区分**的判定结果。

## 里程计

| 参数 | 默认 | 含义 |
|---|---|---|
| `OdomInvalidSample` 判据 | `Valid=false` | 单帧坏里程计 -> 信扫描（降级） |
| 无里程计 | `HasOdom=false` | 纯扫描定位（降级） |
| `RestartJumpXY/Theta` | 1.0 m / 1.0 rad | odom 增量跳变，判定中途重启 |
| `RestartXY/Theta`（检测后放宽） | 1.0 / 1.0 | 重锚定搜索范围 |

重启处理：检测到 odom 系跳变后，记录 `odom_restart_reanchored`，用扫描
在更大窗口内重新定位并重置 odom 基线，避免把整圈运动误当单帧跳变。

## 扫描匹配与融合

## 跨会话全局定位与地图合并

| 参数 | 默认 | 含义 |
|---|---|---|
| `LocalizeCoarseStepXY/Theta` | 0.5 m / 0.196 rad | 全图粗搜索的位置 / 朝向栅格 |
| `LocalizeCoarseScanPts` | 90 | 粗搜索降采样点数（计算有界） |
| `LocalizeHintRadiusXY/Theta` | 2.0 m / 0.6 rad | 给粗略线索时的搜索半径 |
| `LocalizeRefineWinXY/Theta` | 0.8 m / 0.35 rad | 每个粗候选的局部精配准窗口 |
| `LocalizeKFRefRadius` | 6.0 m | 参与局部参考的旧关键帧半径 |
| `LocalizeMinScore` | 0.55 | 唯一解最低内点分数 |
| `LocalizeMaxResidual` | 0.20 m | 唯一解最大点面 RMS |
| `LocalizeScoreMargin` | 0.15 | 与空间分离的次优候选最小分数差 |
| `LocalizeEdgeSigmaXY/Th` | 0.08 m / 0.05 rad | 锚定 / 跨会话边信息 |
| `LocalizeMaxAnchors` | 2 | 会话起点锚定边数量上限 |
| `LocalizeCrossSessionRadius` | 1.5 m | 跨会话重访的世界系近邻半径 |
| `LocalizeCrossMinScore` | 0.70 | 跨会话重访只接受强对齐 |
| `MapOccPersist` | 2.0 | 超过该 log-odds 的占据格不被自由射线擦除 |

判定：`unique` = 最佳分数/残差达标且与次优空间分离候选差距足够；
`ambiguous` = 存在另一个位姿显著不同、分数与残差都达标且分数差小于
margin 的候选；其余为 `failed`（见 `docs/multi-session.md`）。

| 参数 | 默认 | 含义 |
|---|---|---|
| `FieldResolution/Inflate` | 0.05 / 0.30 | 似然场分辨率、高斯影响半径 |
| `MatchWinXY/Theta` | 0.30 m / 0.25 rad | 粗搜索窗口（预测位姿附近） |
| `CoarseStep*` | 0.05..0.1 | 粗网格步长 |
| `RefineIter` | 8 | ICP 迭代次数 |
| `MinEigRatio` | 1e-3 | 信息矩阵特征值比，低于即退化 |
| `SigmaMatchXY/Theta` | 0.015 / 0.01 | 扫描 σ（逆方差权重） |
| `SigmaOdomXY/Theta` | 0.15 / 0.10 | 里程计 σ |
| `DegradedMatchWeight*` | 0.2 / 0.2 | 退化方向扫描权重（其余信 odom） |
| `MaxMatchTransErr/RotErr` | 0.60 / 0.45 | 匹配 vs 预测偏差上限 |
| `MaxDeltaConflict` | 0.25 | 单帧 scan 增量 vs odom 增量冲突上限 |

**为什么不会悄悄输出错误位姿**：每帧先过门控，再过“匹配-预测偏差”和
“增量一致性”两道检查；几何退化时逐方向退化（只信 odom 的可信轴）；
任何拒绝/降级都带独立原因码进入 `FrameDecision` 与日志。

## 关键帧

| 参数 | 默认 | 含义 |
|---|---|---|
| `KeyframeMinDist/MinTheta` | 0.3 m / 0.15 rad | 新建关键帧的位移/转角阈值 |
| `KeyframeMaxGap` | 2.0 m | 触发关键帧的兜底间隔 |
| `MaxScanPointsKF` | 360 | 每关键帧扫描点数上限（内存有界） |
| `RevisitSuppressionXY/Theta` | 0.40 m / 0.30 rad | 新关键帧与已有非前驱节点重合时复用该节点（重复扫同区不膨胀） |
| `refRadius`（代码常量） | 4 m | 局部匹配参考范围 |

关键帧保存稠密栅格化表面（命中单元去重），扫描等距降采样，内存随时间
线性且有上限，不会无界增长。

## 诊断日志与持久化

| 参数 | 默认 | 含义 |
|---|---|---|
| `PersistLogs` | false | 决策/回环/优化日志是否写入状态文件（恢复续跑不需要；关闭后重复存档大小稳定） |
| `MaxDecisionLog/MaxLoopLog/MaxOptLog` | 500 / 300 / 200 | 内存（及开启持久化后的盘上）日志环形上限 |

## 回环

| 参数 | 默认 | 含义 |
|---|---|---|
| `LoopMinFrames` | 15 | 与候选的最小关键帧间隔 |
| `LoopCheckEvery` | 2 | 回环尝试的关键帧节奏 |
| `LoopOdomRadius` | 1.2 m | odom 帧空间近邻半径（抗世界漂移） |
| `LoopMinScore` | 0.45 | 接受的最低 ICP 分数 |
| `LoopStrongScore` | 0.75 | 全权回环阈值 |
| `LoopOdomMaxDiffXY/Th` | 2.0 m / 0.3 rad | scan/odom 局部一致性门限 |
| `LoopSigmaXY/Th` | 0.10 / 0.08 | 回环边信息矩阵 |

判定结果三类、原因码互不相同：`loop_strong`（接受）、
`loop_weak_align`（降权）、`loop_match_score_low` /
`loop_contradicts_odom`（拒绝）。

## 地图

| 参数 | 默认 | 含义 |
|---|---|---|
| `MapResolution` | 0.05 m | 栅格分辨率 |
| `MapLogOddHit/Free` | +0.55 / −0.30 | 命中/穿越更新 |
| `MapLogOddMax` | 4.0 | 对数几率截断 |
| `MapMaxRayStep` | 1000 | 单射线步进上限（有界耗时） |

## 优化器

| 参数 | 默认 | 含义 |
|---|---|---|
| `OptMaxIter` | 30 | GN 外迭代上限 |
| `OptConvergeDelta` | 1e-4 | 收敛阈值（最大修正量） |
| `OptHuberDelta` | 4.0 | 回环边 Huber 核 δ |

收敛条件、LM 阻尼阶梯、只接受代价下降步等见 README“优化器”一节。
