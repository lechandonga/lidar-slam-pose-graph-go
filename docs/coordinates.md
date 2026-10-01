# 坐标、单位与位姿约定

## 单位

| 量 | 单位 | 说明 |
|---|---|---|
| 位置 | 米 m | 二维 `(x, y)` |
| 角度 | 弧度 rad | 归一化到 `[-π, π]`，逆时针为正 |
| 时间 | 秒 s | 源时钟单调值；融合不使用绝对时间 |
| 栅格 | m / cell | `MapResolution`（默认 0.05） |
| 激光距离 | m | 有效区间 `[MinRange, MaxRange]`，默认 `[0.05, 20]` |

## 坐标系

- **world（世界系）**：右手二维系，+x 指向机器人初始正前方，+y 指向左侧。
- **base（机器人体坐标系）**：与激光系重合（extrinsics 假设），
  激光点直接以 `(range·cos θ, range·sin θ)` 表达。
- **odom（里程计系）**：轮式里程计自己的连续系。检测到中途重启时，
  odom 系原点会重置；SLAM 通过扫描重锚定继续在 world 系定位。

## 位姿与复合

位姿 `Pose{X, Y, Theta}` 表示 `world <- base` 的刚体变换。

```
worldPoint = pose.Transform(basePoint)
worldX = x + cos θ · bx − sin θ · by
worldY = y + sin θ · bx + cos θ · by

relative = Between(a, b) = a⁻¹ · b      // a 到 b 的相对位姿
chained  = a.Compose(b)                 // 先 b 后 a
```

角度差分一律 `normAngle(θb − θa)`，避免跨 ±π 跳变。

## 锚定

第一条通过质量门控的激光帧把当前位姿锚定为世界原点 `(0, 0, 0)`。
因此评估精度时，估计轨迹与任意起点的真值之间需要做一次初始 SE2 对齐
（测试中的 `trajectoryRMSE` 与 demo 的 RMSE 都做了该对齐）。

## 边测量约定

位姿图边的相对测量定义在**节点 i 自身的世界朝向系**下：

```
e = [ R(θi)ᵀ (tj − ti) ,  norm(θj − θi − zθ) ]
```

链式里程计边在关键帧创建时用 `Between(prev.Pose, pose)`；
回环边在旧关键帧局部扫描系中匹配，再按相同约定转换。
两者约定一致，优化器 Jacobian 也据此推导（并有数值微分版本可交叉验证）。
