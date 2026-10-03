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
- v1 文件**自动迁移**读取（关键帧归入会话 0，偏移归零）。
- 更新或更老的未知版本返回 `ErrUnsupportedState`；截断/损坏的 gob 返回
  明确错误；结构不一致（如边引用不存在的节点、网格长度对不上、出现
  NaN/Inf）一律拒绝加载。任何校验失败都不会构造出「半新半旧」的流水线。
- `Save` 先写同目录临时文件再原子 rename，并在持锁期间深拷贝关键帧/边，
  所以可以在采集、`Finalize` 进行的同时并发 `Save` 和查询地图。

## 6. 用本地数据复现两段扫描

`synth.SplitSessions(world, path, opt, cut1, cut2)` 把一条真值轨迹切成
`[0,cut1)` 和 `[cut2,end)` 两段（`cut2<cut1` 即重叠），每段都以自己的
起点为原点重新表达扫描与里程计、时钟从 0 开始、噪声使用独立种子：

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
