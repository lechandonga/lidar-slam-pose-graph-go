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

评分对扫描里的**每一个点**计权：一个落在任何已测绘表面之外（或超出地图
范围）的点贡献 0，而不是“免费满分”。因此把扫描整体平移/旋转到一个错误
位置、只靠少数点恰好贴到无关墙面的假设，拿不到接近满分；扫描覆盖点中
至少 50% 必须落在已测绘区域内（覆盖度门限），否则即使分数高也判 failed。
近场点（<2 m）权重最高（最远点的约 6 倍），开阔大房间的远墙雷同不会主导
定位。这样“高分错配”会被压成 failed/ambiguous，调用方据此拒绝锚定，
`BeginSession` 不会把错误位姿提交进旧图。

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

### 重复扫同一片区域（load → 跑一小段 → 保存 → 再加载）必须收敛

新一轮扫描若落在**已经有关键帧**的位置（平移
`<= RevisitSuppressionXY`、转角 `<= RevisitSuppressionTheta`，默认 0.4 m /
0.30 rad），流水线**复用已有节点**而不是新增：

- 不新增关键帧节点、不新增重复约束（锚点/回环边）、不再把同一扫描重复
  积分进栅格，所以节点数、边数、占据格数在“没有新区域”时保持恒定；
- 一段会话如果从头到尾都在复访旧图（未产生任何新节点），`Finalize` 会
  丢弃这段与旧轨迹物理重复的轨迹副本，使多轮后的轨迹仍等于单次合并结果；
- 诊断日志（逐帧决策 / 回环判定 / 优化记录）默认**不写入状态文件**
  （`Config.PersistLogs=false`），所以重复存档的文件大小也稳定；运行期
  `Decisions()/LoopDecisions()/OptLog()` 在内存中照常完整可查。需要把遥测
  一并落盘时把 `PersistLogs` 置 true，日志按 `MaxDecisionLog / MaxLoopLog /
  MaxOptLog` 环形截断、有上界。
- **新探索区域不受影响**：与任一已有节点距离超过阈值的位姿照常新增
  关键帧、边和地图（有测试 `TestNewTerrainStillGrows` 守护）；去重只复用
  节点，从不删除已有数据，也不会让地图/轨迹错位。

量化验证见 `TestRepeatedLoadRunSaveConverges`：真实地把同一段重叠数据
「加载→定位→跑→Finalize→保存」连做四轮，逐轮断言节点/边/占据格不增长、
相邻存档字节差 ≤ 64 B，并打印每轮锚点误差。

## 5. 状态文件版本

- 当前版本 `stateVersion = 2`（含 Session / FrameIDBase / AwaitAnchor）。
- v1 文件**自动迁移**读取。注意 v1 的每个关键帧使用自定义 gob 编码
  （线类型名 `keyframeGob`，没有 Session 字段）；当前版本按版本号选择
  对应线布局解码，再逐字段迁移（关键帧归入会话 0、`FrameIDBase=0`、
  `SurfaceBase` 从旧字段还原），不会把完好的 v1 文件误报成损坏。
- 兼容性用**旧版本程序真实产出的状态文件**验证：`testdata/legacy-v1.gob`
  由 v1 程序对真实环路数据 `Save` 生成（测试 `TestRealV1StateLoads`），
  断言节点/边/占据格完整恢复、会话为 0、再以 v2 重存内容不变。重新生成：
  在 v1 源码里 `go run ./cmd/genstate ./legacy-v1.gob`（见该测试注释）。
- **错误可区分**：版本号更新或更老的未知版本返回
  `errors.Is(err, ErrUnsupportedState)`（`TestErrorKindsDistinguishable`）；
  截断/损坏的 gob、结构不一致（边引用不存在的节点、网格长度对不上、
  NaN/Inf）一律报“corrupt”/“invalid”并拒绝加载。任何校验失败都不会
  构造出「半新半旧」的流水线。
- `Save` 先写同目录临时文件再原子 rename，并在持锁期间深拷贝关键帧/边，
  所以可以在采集、`Finalize` 进行的同时并发 `Save` 和查询地图。

## 6. 用本地数据复现两段扫描

`synth.SplitSessions(world, path, opt, cut1, cut2)` 把一条真值轨迹切成
`[0,cut1)` 和 `[cut2,end)` 两段（`cut2<cut1` 即重叠），每段都以自己的
起点为原点重新表达、时钟从 0 开始、噪声使用独立种子：

- **里程计/位姿路径**按段局部坐标表达（首帧为 `(0,0,0)`），轮式里程计
  在这个局部系里从 0 积分，模拟重新上电；
- **激光扫描在公共世界的真值位姿上对同一面世界墙做光线投射**（生成器的
  `ScanPath` = 原始世界路径，`Path` = 局部路径）。这一点是自洽性的关键：
  扫描观测的是物理世界本身，不能在“平移后的局部坐标”里对世界地图投射，
  否则造出的扫描与该段标注的 `StartWorld` 根本不对应（旧版本的缺陷）。

返回值里 `SplitSession.StartWorld` 是该段起点在**公共世界系**的标注真值，
`TruthWorld[i]` 是每帧的公共世界真值，供评估定位与轨迹误差。

### 6.1 先确认造出的数据自洽

用**某一段自己的**第一帧，在**这一段自己跑出来的地图**里全局定位，结果
必须回到该段局部原点 `(0,0,0)` 附近（这一段自己的世界系就是它首帧）；
再把第二段首帧放进**第一段的地图**里定位，结果必须回到第二段的
`StartWorld`。自动化判定见 `TestSplitDataSelfConsistent`（两段各自自定位，
打印 errXY/errTh）与 `TestSplitLocalizeRecoversAnnotatedStart`（跨会话锚点
对比标注起点）。当前实现两段自定位误差约 2–3 cm，跨会话锚点误差 < 0.1 m。

### 6.2 与“一次跑完”基准做量化对比

把同一条路径既（a）切成两段、各自独立坐标/时钟/里程计再合并，又（b）单
会话从头跑到尾，然后比较：

- **轨迹误差**：合并后第二段轨迹对照 `TruthWorld` 的 RMSE / 最大偏差
  （`TestSplitMergeMatchesSingleRun`，当前 RMSE ≈ 0.3 m）；
- **重叠区地图一致性**：在重叠段机器人位姿的视场包围盒内，把合并图与
  基准图的占据格按**世界坐标**对齐，做双向近邻匹配，报告
  `fwd/rev/mean` 一致率（当前 mean ≈ 0.95）。

定位落在旧图覆盖范围之外（例如在 `(60,60)` 投一帧）必须判 failed 而不是
高分错配；`TestSplitRelocalizeFailureKeepsOldState` 同时断言内存中的
节点/边/占据格与磁盘上的状态文件**逐字节不变**。

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

### 6.3 跨会话使用前提（汇总）

1. 两段必须扫到**物理重叠**且重叠区几何非退化（有近场特征），否则无法
   唯一定位，属正常 failed/ambiguous；
2. 每段坐标/时钟/里程计各自归零由 `PrepareNewSession + Localize +
   BeginSession` 统一：首帧放到旧世界系，之后只用里程计**增量**；
3. FrameID 全局唯一（源 ID + 会话偏移），轨迹/关键帧带 `Session` 标记；
4. 只有 `unique` 才锚定；`failed/ambiguous` 旧图与存档零改动。

全部自动化用例：`split_merge_test.go`（自洽、锚点偏差、合并基准、失败零
改动）、`revisit_bloat_test.go`（四轮真喂数据不膨胀、新区域仍增长、
`-race` 下并发续跑+查询+存档）、`legacy_v1_test.go`（真实 v1 存档迁移、
错误可区分），以及 `session_test.go` 既有的成功/失败/多解/hint 用例。
