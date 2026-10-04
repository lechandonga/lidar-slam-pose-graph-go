# 本地数据构造、回放与验证

本工程不访问任何网络或外部服务。所有数据都可以在本地生成或从本地文件
回放，并且完全可复现。

## 1. 确定性数据生成器（`synth` 包）

### 世界

- `RectWorld(width, height, boxes)`：矩形外墙 + 若干轴对齐内框。
- `CorridorWorld(length, halfWidth)`：双侧平行墙的长直走廊（退化场景）。

### 路径

- `MakeStraightPath(length, n)`：直行。
- `MakeTurnPath(leg, perLeg, turnFrames)`：直行 + 原地 90° 转弯 + 直行。
- `MakeLoopPath(side, perSide, turnFrames)`：绕矩形闭合一圈（回环场景）。
- 也可以直接传显式 `[]Pose` 路径。

### 传感器与噪声（全部带种子 PRNG）

| 字段 | 含义 |
|---|---|
| `BeamNoise` | 每个距离的高斯噪声 σ（m） |
| `BearingJit` | 方位角抖动 σ（rad） |
| `OdomDrift` | 里程计比例误差（如 0.05 = 5%） |
| `OdomBias` | 里程计转角偏置率（rad/s） |
| `BadFrames map[int]Fault` | 注入坏帧：`few_points`/`nan`/`burst`/`huge` |
| `BadOdoms map[int]bool` | 注入坏/丢失里程计样本 |
| `Restarts map[int]bool` | 注入 odom 中途重启（原点重置） |
| `Seed` | 随机种子，决定一切噪声 |

生成器还通过 `GroundTruth()` 提供真值位姿，用于离线误差评估。

```go
gen := synth.New(
    synth.RectWorld(14, 10, nil),
    synth.Options{
        Path: synth.MakeLoopPath(4, 24, 6),
        Nbeams: 360, Fov: 2*math.Pi, MaxR: 20,
        BeamNoise: 0.03, OdomDrift: 0.02, OdomBias: 0.003,
        BadFrames: map[int]synth.Fault{20: {Kind: "nan"}},
        Restarts: map[int]bool{60: true},
        Seed: 42,
    },
)
src := synth.NewSource(gen) // 实现 root.DataSource
```

## 2. 本地文件回放

实现 `root.DataSource` 即可从文件回放（`Next() (Frame, bool)`）。内置
`root.FrameSource` 用于把内存中的 `[]Frame` 作为回放源（见 `bridge.go`），
也便于把录制好的数据解析后喂入流水线。

`FrameSource` 提供 `Rewind()`，因此同一份输入可以反复回放做确定性比对。

## 3. 复现验证

```bash
# 同种子重复 -> 逐帧位姿必须逐位一致
go test -count=1 -run TestDeterministicReplay -v .

# 各场景
go test -count=1 -run 'TestStraightLine|TestTurn|TestNoiseAndBadFrames' -v .
go test -count=1 -run 'TestLoopImprovesOverOdom' -v .
go test -count=1 -run 'TestRestartReanchors' -v .

# 保存 -> 加载 -> 续跑 == 一次跑完
go test -count=1 -run TestSaveResumeEquivalent -v .

# 并发地图查询（采集进行中）
CGO_ENABLED=1 go test -race -run TestConcurrentMapQueries .
```

测试日志（`-v`）打印：每类输入的门控判定、关键判定原因、优化迭代/收敛/
代价、以及 RMSE 等误差指标。

## 3.5 两段独立会话（里程计 / 时钟各自归零）

`synth.SplitSessions(world, path, opt, cut1, cut2)` 把一条完整轨迹切成两段
（`cut2 < cut1` 时两段有重叠），并让每段都成为**独立的一次开机**：

- 每段扫描和里程计都以自己的起点为原点重新表达（里程计从 0 积分）；
- 帧时间从 0 开始；
- 两段使用相互独立的噪声种子（由 `opt.Seed` 派生），模拟真实的不同上电
  周期；
- 扫描在**公共世界的真值位姿**上投射（生成器内部 `ScanPath`=世界路径、
  `Path`=局部路径）：激光看到的是物理世界，不能在平移后的局部坐标里对
  世界地图投射，否则扫描与该段标注的 `StartWorld` 不自洽；
- 同时返回每段在公共世界系的真值 `TruthWorld` 和第二段起点 `StartWorld`，
  供评估全局定位误差。

```go
a, b, fullTruth := synth.SplitSessions(world, path, opt, 30, 24)
srcA, srcB := synth.SessionSource(a), synth.SessionSource(b)
// 先消费 srcA、存档；Load 后 PrepareNewSession，再用 srcB 的首帧 Localize、
// BeginSession，然后继续消费 srcB。完整示例见 docs/multi-session.md。
```

## 4. 误差度量

测试与 demo 的轨迹 RMSE 都先做初始 SE2 对齐（流水线把首帧锚定在世界
原点，真值起点可能任意），再比较逐帧 `(x, y)`。角度误差单独取归一化
夹角的最大/均值。回环测试额外分别给出 SLAM 与纯里程计的误差，证明
回环修正带来明显提升。
