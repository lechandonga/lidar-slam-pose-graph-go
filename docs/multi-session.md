# 跨会话建图：独立里程计 / 时钟归零的多段扫描合并

一次进不去、换个地方开机、把同一块场地分几次扫完——这些场景下，每一段
（session）都是**独立的一次开机**：轮式里程计从 0 开始、时钟从 0 开始、
机器人起始位姿未知。本工程支持把这样的多段数据统一进同一张地图和同一条
轨迹，且不会因为「接不上」而污染已建好的旧图。

## 1. 两种「接着跑」的区别

| 场景 | 里程计/时钟 | 用法 |
|---|---|---|
| 中断恢复（同一次开机，进程重启） | 延续，能直接用 | `Save` → `LoadState` → 继续 `ProcessFrame` |
| 跨会话（换地方重新开机） | 各自归零、起点未知 | `LoadState` → `PrepareNewSession` → `Localize` → `BeginSession` |

`LoadState` 默认按「中断恢复」处理：恢复后立即可以继续喂帧。只有显式调用
`PrepareNewSession()` 后，流水线才进入「等待新会话锚定」状态，此时
`ProcessFrame` 会返回原因码 `awaiting_session_anchor` 并**原样拒绝**任何帧。

### 跨会话使用的前提

- **旧图必须非空且已覆盖新起点附近**：`PrepareNewSession` 在空图上会
  报错；首帧若落在旧图覆盖范围之外，`Localize` 直接判 `failed`，不会
  被强行「传送」到某个猜测位置。
- **首帧扫描要能唯一刻画所在位置**：开阔几何对称区域（空房间中央）可能
  得到 `ambiguous`，此时应移动机器人获得更独特的观测、或提供 `Hint`，
  而不是任选一个候选。
- **统一靠的是首帧锚定 + 增量，而不是假设时钟/里程计连续**：两段的
  世界坐标通过 `Localize` 的结果统一；之后只用每段自己的里程计**增量**；
  时钟与源帧号通过全局 FrameID 偏移统一（见第 3 节）。不需要、也不应
  手工对齐两段的绝对里程计或时间戳。

## 2. 新会话如何进旧坐标系

```go
// 1) 恢复上一段（或直接持有上一段的 *Pipeline）
p, err := root.LoadState("site.gob", logger)
if err != nil { /* 版本不支持 / 文件损坏：明确报错，不会加载出半成品 */ }

// 2) 声明要开一段全新的会话
if err := p.PrepareNewSession(); err != nil { /* 旧图为空 */ }

// 3) 在旧地图里给新会话的第一帧全局定位（不修改任何状态）
lr := p.Localize(firstScan, root.LocalizeHint{})
//    可选粗略线索（旧世界系里的大概位置/朝向）：
//    lr = p.Localize(scan, root.LocalizeHint{HasHint: true,
//        Pose: root.Pose{X: 3.2, Y: -1.0, Theta: 0.4}})

switch lr.Status {
case root.LocalizeUnique:
    // 唯一解：提交并锚定，之后正常 ProcessFrame
    if err := p.BeginSession(firstFrame, lr); err != nil { ... }
case root.LocalizeAmbiguous:
    // 多个位置/朝向都说得通——必须拒绝，不能随便挑一个
    // lr.Candidates 里给出全部候选供人工/更多观测裁决
case root.LocalizeFailed:
    // 对不上（起点不在旧图覆盖范围内 / 扫描太差 / 分太低）
}
```

`Localize` 只读不写：返回里同时给出

- `Status`：`unique` / `ambiguous` / `failed`；
- `Reason`：可区分的原因码（`localize_score_low`、`localize_residual_high`、
  `localize_ambiguous_multiple_modes`、`localize_no_overlap`、
  `localize_bad_scan:*`、`localize_empty_prior_map`）；
- `Score`（精配准内点率，0~1）、`Residual`（点到表面 RMS，米）、
  `Candidates`（排序后的全部显著候选及各自分数/残差）。

`BeginSession` 只接受 `unique`。失败或多解时直接返回错误，旧地图、位姿图、
轨迹、关键帧一个字节都不会被改动（有测试断言节点/边/占据格数不变）。

### 判定规则（阈值集中在 `Config`）

- **unique**：最佳 `Score >= LocalizeMinScore` 且
  `Residual <= LocalizeMaxResidual`，并且与空间上分离的次优候选分数差
  `>= LocalizeScoreMargin`；
- **ambiguous**：存在另一个位置/朝向显著不同、分数与残差都达标的候选，且
  分数差小于 margin（例如在几何对称的空房间中央做 360° 扫描）；
- **failed**：没有候选达到覆盖度/分数/残差要求。

给一个大致线索时，搜索只在 `LocalizeHintRadiusXY / LocalizeHintRadiusTheta`
范围内进行；线索附近若根本没有已测绘内容，会得到 failed 而不是被「传送」
过去。

## 3. 坐标、里程计与时钟怎么兼容

- 新会话首帧通过 `BeginSession` 放置在 `lr.Pose`（**旧世界系**）；
- 新会话自己的轮式里程计增量（而不是绝对值）从首帧起算，因此「里程计开机
  归零」不影响世界系运动；
- 内部给每帧分配**全局唯一 FrameID**（源 ID + 会话偏移），所以即使两段
  源数据都从 0 编号、时间戳都从 0 开始，轨迹/关键帧/日志也不会冲突；
- 每个关键帧带 `Session` 标记；同会话内沿用里程计链边，跨会话之间只通过
  锚定边和「跨会话重访」回环边连接。

锚定后，新会话在重叠区继续移动时，若扫描再次命中上一段的旧表面，会加入
**强**跨会话重访约束（世界系近邻、扫描对齐，阈值
`LocalizeCrossSessionRadius / LocalizeCrossMinScore`）。跨会话不做里程计
一致性校验（两个里程计系不可比），因此只接受强对齐，避免错配。最终
`Finalize` 统一优化整张图并从优化后关键帧**重建地图**，保证地图与修正后的
轨迹一致。

## 4. 地图合并的冲突处理

- **已确认占据不会被重复扫描擦掉**：log-odds 超过 `MapOccPersist` 的格子，
  后续会话的「自由空间」射线不再降低它。同一面墙扫十遍也不会变糊或变空；
  新观测只会让墙更确定。
- **重叠区不出现两条轮廓**：重叠墙通过锚定/跨会话回环被拉到同一坐标系，
  最终地图由全部关键帧在统一优化后的位姿上重新栅格化。
- **轨迹修正带动地图**：`Finalize` 先优化位姿图，再从优化后关键帧重建栅格，
  任何后续修正都不会留下「轨迹动了、图还是旧的」的错位。

## 5. 状态文件版本

- 当前版本 `stateVersion = 2`（含 Session / FrameIDBase / AwaitAnchor）。
- v1 文件**自动迁移**读取（关键帧归入会话 0，偏移归零）。注意 v1 的
  关键帧在盘上是带自定义 gob 包装的 `keyframeGob` 线格式；加载器用
  同名超集 `keyframeWire` 解码，旧字节按字段名匹配、缺失的 `Session`
  补 0，因此**上一版程序真实保存的文件**可以直接读出（见
  `testdata/real-v1.gob` 与 `TestRealOldVersionStateLoadsReg`）。
- 老文件没有新版引入的配置项（`Revisit*`、`PersistedLogCap` 等）时，
  gob 会把它们解成零值；加载时 `fillLegacyDefaults` 用当前默认值补齐
  这些字段，避免存档被正常读出后却以"半默认"配置运行（如日志无界、
  重访去重半径为 0）。
- 更新或更老的未知版本返回 `ErrUnsupportedState`；截断/损坏的 gob 返回
  明确错误；结构不一致（如边引用不存在的节点、网格长度对不上、出现
  NaN/Inf）一律拒绝加载。任何校验失败都不会构造出「半新半旧」的流水线。
- `Save` 先写同目录临时文件再原子 rename，并在持锁期间深拷贝关键帧/边，
  所以可以在采集、`Finalize` 进行的同时并发 `Save` 和查询地图。

## 5.5 重复扫同一片区域为什么不会越存越大

「加载状态 → 在已建图区域跑一小段 → 保存 → 再加载」时，规模必须收敛：

- **锚定去重（anchor coalescing）**：新会话首帧若与某个旧会话关键帧处
  在同一观测点（世界位姿近、扫描强配准且收敛到近似零相对变换、并被
  全局似然场独立确认），不再新建关键帧，而是把该会话锚定到已有节点。
- **运行中重访去重（revisit reuse）**：后续帧若到达别的会话已建关键帧
  的观测点，复用该节点并加一条强跨会话确认边，不新增节点、不重复栅格化。
  搜索窗 `RevisitSearchRadius/Theta` 容许两次独立经过之间的里程计漂移，
  但只有**收敛后**相对变换落在 `RevisitReuseRadius/Theta`（约 0.25 m /
  0.18 rad）内才判为同一观测点，因此长走廊里相隔一个关键帧间距的不同
  位置不会被错误地硬拉到一起。
- **已建图区不信任退化匹配**：在任何会话都没有表面的全新地形里，退化
  帧仍会按里程计位姿建图（首次探索）；在已有表面附近的退化匹配则不
  新增节点，避免在重叠区堆积漂移副本。
- **诊断日志有界**：保存时帧/回环/优化决策日志只保留最近
  `PersistedLogCap` 条（默认 4000），它们是诊断信息而非地图状态，避免
  超长会话或大量重复回放让文件无限增大。

净效果：只重复已建图区域时，关键帧数、约束边数在一轮后保持恒定，文件
大小收敛；进入真正的新区域时关键帧/地图照常增长，且不会为了抑制膨胀
删除既有数据（断言见 `TestRepeatedOverlapConvergesReg`，真实连续 6 轮
加载/跑/保存对比节点数、边数与字节数）。

## 6. 用本地数据复现两段扫描

`synth.SplitSessions(world, path, opt, cut1, cut2)` 把一条真值轨迹切成
`[0,cut1)` 和 `[cut2,end)` 两段（`cut2<cut1` 即重叠），每段都以自己的
起点为原点重新表达扫描与里程计、时钟从 0 开始、噪声使用独立种子：

生成时严格区分两个坐标系，保证**每段数据自身自洽**：

- 扫描点按该帧在**公共世界**里的真实位姿投线（波束几何取决于机器人
  在世界中的位置），所以第二段首帧描绘的就是 `StartWorld` 处看到的景象；
- 轮式里程计在**段内局部系**（首帧为 0,0,0）里按局部路径积分，里程计
  从 0 开始。两者用不同生成器、不同噪声种子分别产生，再按帧合成。

> 早期实现把扫描也按局部路径投线，等于让第二段首帧描绘的是世界原点
> （第一段起点）的景象，于是定位给出接近满分的错误解。现在用第二段
> 自己的首帧定位，结果回到它自己标注的 `StartWorld` 附近（典型误差
> 约 0.1 m / 0.01 rad），见 `TestSplitSelfConsistentReg`。

```go
a, b, fullTruth := synth.SplitSessions(world, path, opt, 30, 24)
p.ProcessFrame( /* ... session a via synth.SessionSource(a) ... */ )
p.Finalize(); p.Save("site.gob")
loaded, _ := root.LoadState("site.gob", logger)
loaded.PrepareNewSession()
srcB := synth.SessionSource(b)
first, _ := srcB.Next()
lr := loaded.Localize(first.Scan, root.LocalizeHint{})
loaded.BeginSession(first, lr)
for { fr, ok := srcB.Next(); !ok; loaded.ProcessFrame(fr) }
loaded.Finalize()
```

对应自动化验证见 `session_test.go`：定位成功（含误差）、失败不污染、
对称多解、hint 限制搜索、两段合并轨迹/地图一致性、反复存档不膨胀、
v1 迁移与损坏文件拒绝、以及 `-race` 下并发查询 + 存档。

本轮新增/加强的端到端回归集中在 `multisession_regression_test.go`
（日志打印业务判定与量化指标）：

| 用例 | 覆盖业务 | 量化指标 |
|---|---|---|
| `TestSplitSelfConsistentReg` | 第二段自身扫描定位回自己标注起点 | `errXY` / `errTh`、最佳/次优候选分 |
| `TestSplitFailureUntouchedReg` | 定位失败可区分且旧状态零改动 | 节点/边/轨迹/占据格前后计数 |
| `TestMergeMatchesSingleRunReg` | 两段合并对比一次跑完 | B 段轨迹 `rmse/max`、重叠区地图一致率、A-only 对照 |
| `TestRepeatedOverlapConvergesReg` | 连续 6 轮真实加载/跑重叠/保存 | 每轮节点数/边数/存档字节，末两轮必须相同 |
| `TestRealOldVersionStateLoadsReg` | 旧版本**真实存档**兼容 | 加载 `testdata/real-v1.gob`（v1 程序实际产出）节点/占据数、配置补全、续跑首帧 |
| `TestMergeConcurrentQuerySaveReg` | 合并进行中并发查询+保存 | `-race` 下无数据竞争 |

### 自己动手验证「数据自洽」与「对比一次跑完」

```bash
# 1) 第二段定位必须回到它自己的标注起点（打印偏差）
go test -count=1 -run TestSplitSelfConsistentReg -v .

# 2) 分两段合并 vs 一条轨迹一次跑完（轨迹 RMSE + 重叠区地图一致率）
go test -count=1 -run TestMergeMatchesSingleRunReg -v .

# 3) 同一片重叠区连续 6 轮 加载->跑->保存，节点/边/字节必须收敛
go test -count=1 -run TestRepeatedOverlapConvergesReg -v .

# 4) 用上一版程序真实产出的存档验证兼容（不是手工拼的最小结构）
go test -count=1 -run TestRealOldVersionStateLoadsReg -v .

# 5) 竞态
CGO_ENABLED=1 go test -race -run 'Concurrent|Reg$' .
```

`testdata/real-v1.gob` 由历史 v1 提交的程序真实跑出（绕一圈的轨迹 +
栅格地图后 `Save`）。要重新生成，可在该提交的 worktree 里运行同一
`NewPipeline + ProcessFrame + Finalize + Save` 流程得到状态文件，再放回
`testdata/`；这样验证的是真正的旧版产物，而非当前代码手工构造的结构。
