# lidar-slam-pose-graph-go

本地可复现的二维激光 SLAM：扫描匹配与轮式里程计融合、位姿图与回环修正、
栅格地图与状态持久化。**无任何外部服务依赖**——输入来自本地文件回放或内置的
确定性数据生成器（激光/里程计噪声、坏帧、回环均可注入）。

## 快速开始

```bash
# 端到端 demo：生成一圈闭合轨迹 -> 跑 SLAM -> 输出轨迹/地图
go run ./cmd/slam-demo -seed 42 -out ./out
# 产物:
#   out/map.pgm         占据栅格地图 (可直接用图像查看器打开)
#   out/trajectory.csv  逐帧位姿 frame,time,x,y,theta
#   out/traj_*.csv      真值/里程计对比

# 全部测试（确定性、含竞态检测，见下）
go test ./...
CGO_ENABLED=1 go test -race -run TestConcurrentMapQueries .

# 性能基准
go test -bench=. -benchtime 1x .
```

## 坐标与单位约定

详见 [`docs/coordinates.md`](docs/coordinates.md)。要点：

- 位置：**米（m）**，二维 `(x, y)`，世界系为右手系，x 前、y 左。
- 角度：**弧度（rad）**，范围 `[-π, π]`，从 x 轴逆时针为正。
- 位姿：`world <- base` 的刚体变换 `T=(x, y, θ)`；复合 `p.Compose(q)`
  表示先做 q 再做 p；`Between(a,b)` 给出 a→b 的相对变换。
- 第一条有效激光的位姿锚定为世界原点 `(0,0,0)`。
- 时间：源时钟秒（单调浮点），但定位不依赖绝对时间。

## 处理流程

```
Frame (激光 Scan + 轮式 Odom)
  │
  ├─ 扫描质量门控 validateScan
  │     点太少 / NaN-Inf / 超量程坏点占比 / 射线方向聚集 -> 拒绝(原因可区分)
  │
  ├─ 里程计门控
  │     缺帧 / Valid=false      -> 降级(信扫描)
  │     增量跳变 > RestartJump*  -> 判定中途重启，扫描重锚定(原因 restart)
  │
  ├─ 扫描匹配 matchScan（在预测位姿附近）
  │     粗搜索: 栅格命中重叠计数（无 exp、确定性网格）
  │     精配准: 点-线 ICP（PCA 预计算法向，Huber 鲁棒权重）
  │     退化诊断: 3x3 信息矩阵特征值比 + 逐方向 weak 轴标记
  │
  ├─ 融合 fuseDelta：逆方差权重混合扫描增量与里程计增量
  │     弱几何方向 -> 该方向退化为信里程计（降级，原因 degenerate）
  │     匹配-预测偏差过大 -> 拒绝匹配、保留里程计（原因 match_far_*）
  │     两帧增量冲突过大 -> 拒绝匹配（原因 match_far_from_prediction）
  │
  ├─ 关键帧 + 局部稠密表面（栅格化、有上限、有界内存）
  └─ 回环检测 tryLoopClosure（固定节奏）
        odom 帧空间近邻 -> 局部系全角度扫描对齐
        接受 / 降权(soft) / 拒绝，原因分别记录

Finalize:
  Gauss-Newton 位姿图优化（Huber 鲁棒核 + LM 信赖域行搜索 + 迭代上限）
  -> 非关键帧轨迹按最近关键帧校正重定位
  -> 地图从优化后关键帧一次性重建（保证地图与轨迹对齐）
```

## 回环与矛盾约束的判定规则

| 判定 | 条件 | 处理 | 记录原因 |
|---|---|---|---|
| accepted | 扫描对齐分数 ≥ `LoopStrongScore`，且 scan/odom 局部一致 | 全权边 | `loop_strong` |
| soft | 分数在 `[LoopMinScore, LoopStrongScore)` | 按分数比例降权 + 优化器 Huber | `loop_weak_align` |
| rejected | 分数过低 | 不加边 | `loop_match_score_low` |
| rejected | scan 相对运动与 odom 局部链矛盾超阈 | 不加边 | `loop_contradicts_odom` |

即使错误约束进入优化器，LM 信赖域 + “只接受代价下降步”也保证位姿不会被
带飞（`TestBadLoopConstraintHandled`）。所有判定都写入 `LoopDecision` 日志，
与坏帧判定原因码相互区分。

## 优化器的收敛条件与可复现性

- 收敛：最大位姿修正 `< OptConvergeDelta`（默认 1e-4 m/rad）。
- 迭代上限：`OptMaxIter`（默认 30）；LM 阻尼阶梯 1e-4…10。
- 固定节点 0（先做 Schur 式消元，保证法方程对称正定）。
- 稠密高斯消元（关键帧数量有界，单次数十节点规模为亚毫秒～毫秒级）。
- 固定的节点/边迭代顺序、无随机数，因此**同一张图重复优化位姿逐位相同**。

## 确定性

数据生成器用带种子的 PRNG；匹配网格、种子排序（平局按位姿字典序）、
优化遍历顺序全部确定。同一份输入重复回放，逐帧轨迹完全一致
（`TestDeterministicReplay`）。

## 持久化与“中断恢复”

`p.Save(path)` 先写临时文件再原子 rename，且拒绝写入含 NaN/Inf 的状态；
`LoadState(path, logger)` 读回后可继续处理。保存-在中段-加载-续跑与一次跑完
结果一致（`TestSaveResumeEquivalent`），不会留下半更新的地图或轨迹。
关键帧的未导出字段 `surfaceBase` 通过自定义 gob 编解码一并保存。

## 跨会话建图（换位置重新开机）

机器搬到新位置、里程计和时钟都归零后再扫第二段时，用
`p.BeginSessionLocalize(firstFrame, hint)` 在旧图里做全局定位：起点落在
旧图覆盖内即可自动求出旧世界系位姿并锚定，之后整段在旧世界系跑。定位
返回**可区分**的三种结论——`unique`（含对齐分数、残差 RMS、与次佳解的
分差）/ `ambiguous`（多个位置都像）/ `failed`（对不上）；后两者**不改动
任何旧地图、轨迹和位姿图**，可带粗略 hint 重试。重叠区通过跨会话回环边
和图压缩合并成一条轨迹、一张无重影地图；已占据单元不会因重复扫过被擦
除；反复“加载→扫→保存”不会膨胀。状态文件 v1 自动迁移、损坏/未来版本
明确拒绝。详见 [`docs/cross-session.md`](docs/cross-session.md)，测试见
`crosssession_test.go`。

## 并发

所有可变状态由 `Pipeline.mu`（RWMutex）保护：采集/优化/地图写入持写锁，
地图查询 `MapSnapshot()` 持读锁并返回**不可变拷贝**，因此并发读不会看到
中间状态。竞态测试见 `TestConcurrentMapQueries`（`-race` 下通过）。

## 参数

所有阈值集中在 `config.go` 的 `DefaultConfig()`，含义逐项注释，包括：
扫描质量门限、融合 σ、退化权重、匹配/预测偏差门限、回环半径与分数、
关键帧间隔、地图分辨率/对数几率、优化收敛与迭代上限。

## 代码结构

| 文件 | 职责 |
|---|---|
| `types.go` | 位姿 SE2、帧/数据源、原因码 |
| `config.go` | 全部可调参数与默认值 |
| `frame.go` | 扫描质量门控、融合 |
| `field.go` | EDT 似然场（粗评分） |
| `matcher.go` | 粗搜索 + 种子管理 |
| `icp.go` | 点-线 ICP、法向、退化轴诊断 |
| `keyframe.go` | 关键帧、有界表面/扫描 |
| `graph.go` | SE2 位姿图、GN+LM 优化、稠密求解 |
| `loop.go` | 回环检测/门控/记录、Finalize |
| `map.go` | 对数几率栅格、射线、快照、重建 |
| `state.go` | 原子化 gob 持久化 |
| `bridge.go` | 内存回放源 |
| `log.go` | 可插拔日志（输入/判定/误差） |
| `synth/` | 确定性本地数据生成器 |
| `cmd/slam-demo/` | 端到端示例 |

## 本地数据构造与验证

见 [`docs/local-data.md`](docs/local-data.md)。
