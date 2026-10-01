# lidar-slam-pose-graph-go

纯本地、确定性可复现的二维激光 SLAM（Go）：扫描匹配 + 轮式里程计融合、
关键帧位姿图与回环优化、log-odds 栅格地图、状态持久化。数据来自本地
确定性合成/回放，不依赖任何外部服务。

## 快速开始

```bash
go run ./cmd/slamdemo -out output      # 生成数据并运行，输出地图+轨迹+指标
go run ./cmd/slamdemo -v               # 打印每帧判定
go run ./cmd/slamdemo -save-at 50      # 中途保存/加载后续跑

go test ./...                          # 全量测试
CGO_ENABLED=1 go test -race ./internal/slam -run Concurrent
```

默认演示输出：`map.pgm`、`traj_slam.csv`、`traj_odom.csv`、
`traj_truth.csv`，以及节点/边、回环判定、RMSE 等指标。闭环场景下回环
修正使轨迹 RMSE 相对纯里程计改善约 55%。

## 代码结构

| 包 | 职责 |
| --- | --- |
| `internal/geom` | SE(2) 位姿、复合/求逆/相对变换、角度归一 |
| `internal/sensor` | 激光扫描、轮式里程计、同步帧数据类型 |
| `internal/gridmap` | log-odds 占据图、射线更新、带最近邻标签的距离场、PGM/CSV 导出 |
| `internal/scanmatch` | 有效点提取、距离场 Gauss–Newton 配准、退化/坏帧/偏差判定 |
| `internal/fusion` | SE(2) 切空间加权融合、里程计重启检测 |
| `internal/posegraph` | 关键帧位姿图、SE(2) LM 优化器、回环边三态（接受/降权/拒绝） |
| `internal/slam` | 管线编排、关键帧、回环检测、地图重建、原子化状态存取、并发控制 |
| `internal/sim` | 确定性合成世界、闭环轨迹、噪声激光与漂移里程计 |
| `cmd/slamdemo` | 本地端到端演示与产物输出 |

## 特性要点

- 坏帧/少点/高残差/大偏差/几何退化均有可区分判定码，拒绝或降权，绝不
  静默输出错误位姿；
- 矛盾回环按 χ² 与配准残差拒绝或 0.25 降权，记录 `Status/Reason/Chi2`；
- 优化器有明确收敛条件、迭代上限与失败回退，同图重复优化可复现；
- 回环后整体重建地图，地图与最终轨迹严格对齐；
- 状态写入为临时文件 + fsync + 原子 rename，保存后续跑与一次跑完逐位
  一致；
- 单一读写锁成组保护全部状态，采集/优化/查询并发安全（`-race` 验证）；
- 前端计算量由 `LocalKeyframes` 界定，地图固定尺寸，资源不随时间无界增长。

坐标/单位、处理流程、参数含义、回环与异常帧判定规则、数据构造与复现实验
步骤详见 [`docs/DESIGN.md`](docs/DESIGN.md)。
