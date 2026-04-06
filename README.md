# VDA5050 vehicle simulator (YFAOS-aligned)

Standalone MQTT simulator that pairs with `aos-backend` `CommAdapterImpl` (v2.1): it **subscribes** to `order` and `instantActions`, and **publishes** `connection`, `state`, and `visualization` on the same `topicPrefix` the vehicle uses in AOS.

## Build

```bash
cd vda5050-simulator
mvn -q package
```

## Run

```bash
mvn -q exec:java
```

Override config:

```bash
mvn -q exec:java -Dsimulator.config=D:/path/to/my-sim.yaml
```

Ensure `topicPrefix` matches the vehicle in Operations Desk / DB (`interfaceName`, `majorVersion`, `manufacturer`, `serialNumber`). Default config uses `serialNumber: "Vehicle_2"` — change both sides if you use another vehicle name.

## 地图目录与更换地图

- 约定目录：[`maps/`](maps/README.md)（与工程根下 `vda5050-simulator/maps/` 对应）。
- **与 AOS 使用同一份** OpenTCS 驾驶层 XML（同一导出文件），放入 `maps/active.xml`，或通过 `map.xmlPath` 指向任意路径（如 `maps/archive/youle-final-1.xml`）。
- 更换地图：覆盖 `active.xml` 或改 `map.xmlPath` / 外部 YAML，重启模拟器。
- 相对路径 `maps/active.xml` 以**运行时的当前工作目录**为准；请在 `vda5050-simulator` 目录下执行 `mvn exec:java`，或改用绝对路径。
- 详见 [`maps/README.md`](maps/README.md)。

---

## 运动逻辑（核心说明）

本节描述模拟器**如何根据地图与订单计算车位姿**，是与 AOS 联调的**关键契约**，与 `aos-backend` 中 `AgvCommunicationAdapter`（仿真段）及 `MessageResponseMatcher` 的假设对齐。

### 两种工作模式

| 模式 | 条件 | 行为 |
|------|------|------|
| **地图模式** | `map.xmlPath` 有效，且边在植物中能匹配（含**反向路径**：订单为 A→B 而地图边为 B→A 时沿同几何反向走） | 沿 **POLYPATH** / **BEZIER** 采样 |
| **边级回退** | 某条边在植物中既不正向也不反向匹配、或几何构建失败 | 仅该边用 `nodePosition` **弦线**；仍失败则再尝试「全弦线」或「首末直线」 |

### 与 AOS 内置小车仿真（`AgvCommunicationAdapter`）的对比

| 维度 | AOS `AgvCommunicationAdapter` | 本 MQTT 模拟器（`SimulationEngine` + 植物 XML） |
|------|--------------------------------|------------------------------------------------|
| **数据来源** | OpenTCS **内核**里的 `MovementCommand`：`Point` 位姿、`Path` 长度与 `Layout` 与模型编辑器一致 | **独立加载**同一份驾驶层 XML + VDA **order** JSON；须与 AOS 使用**同一导出文件**及前缀配置，否则会出现「内核好、模拟器偏」 |
| **布局 → 世界 mm** | 写死 `controlX * 50`、`controlY * (-50)`（与默认 `layoutScaleMm=50`、`layoutFlipY=true` 一致） | `LayoutTransform` 可配置，**默认与上式相同** |
| **POLYPATH / ELBOW / SLANTED** | 三类都走 `PathGeometryCalculator.calculatePointOnPolylineWithTangent`（折线弧长） | XML 中 `ELBOW`、`SLANTED` **按 POLYPATH** 解析（与 AOS 一致）；此前误当成 DIRECT 的已修正 |
| **BEZIER** | `calculateBezierPointWithTangent`：`t = 已走 mm / path.length`，控制点同上变换；另有高阶 / `BEZIER_3` 分段逻辑 | 三次贝塞尔：`t = 已走 m / lengthMm`，与「按路径长度线性归一化」**同一思路**；**未实现**内核里 `BEZIER_3`（两段三次拼接）的专用分支 |
| **DIRECT** | 起终点直线按弧长取点 | `PathMotionFactory` 起终点直线，等价 |
| **推进方式** | `VelocityController` + `WayEntry`：按**时间**推进，速度受路径限速与车辆最大速度影响（可变速） | 按边 `maxSpeed`（或默认）**匀速**，`tick(dt)` 累加弧长 = `speed * dt` |
| **多段路径** | 内核一次 **`MovementCommand` 一步**（一条边），步与步由调度衔接 | 一个 **order** 内多条 **released** 边在内存中**串联**为一条 `RouteFollower` |
| **输出** | 写入内核 `Pose`（mm、度），监控从内核读 | VDA `state`/`visualization` 的 `agvPosition`（**米**、**弧度**），由适配器再换算进内核 |

**结论**：几何上（折线弧长、贝塞尔按长度比例取点、布局变换）与 AOS 小车仿真**对齐**；差别主要在**速度模型**（变速 vs 匀速）、**数据是否同源**（内核 vs 外部 XML），以及**未覆盖**的少数连接类型（如 `BEZIER_3` 专用分支）。联调时请优先保证 **XML + `map.*` 前缀** 与线上一致。

### 坐标系与布局点变换

- **OpenTCS 点坐标**：XML 里 `<point positionX positionY>` 为植物坐标系下的 **毫米（mm）**，与内核一致。
- **路径布局点**：`<pathLayout>` 下 `<controlPoint x y>` 为**布局坐标**（编辑器单位），**不能**直接与点的 `positionX/Y` 混加。
- 布局点 → 世界毫米，与 YFAOS `AgvCommunicationAdapter` 中代码一致：

  - \(X_{\mathrm{mm}} = \texttt{layoutX} \times \texttt{layoutScaleMm}\)（默认 `layoutScaleMm = 50`）
  - \(Y_{\mathrm{mm}} = -\texttt{layoutY} \times \texttt{layoutScaleMm}\)（当 `layoutFlipY: true`，对应源码中的 `y * (-50)`）

仿真器内部将 mm 转为米，写入 VDA `agvPosition`（米、`theta` 弧度）。

### pathLayout 与 connectionType

部分 OpenTCS 导出里 **`connectionType` 为空**或误标为 **`DIRECT`**，但仍带有 **`controlPoint`**。仿真器加载时会按控制点个数推断：**0 → DIRECT**，**1 → POLYPATH**，**2 → BEZIER（三次）**，**≥3 → POLYPATH**；若显式 `DIRECT` 却存在控制点，同样按控制点推断，避免只连起终点直线。

### POLYPATH（折线 / 多段直线）

世界坐标下的折线顶点顺序为：

**起点（source 点 mm）→ 各布局控制点经上述变换后的 mm → 终点（destination 点 mm）**

车辆沿相邻顶点逐段直线运动；累计弧长为各段欧氏长度之和。同一时刻的航向为该段切线方向（ atan2 ）。

### BEZIER（三次贝塞尔）

OpenTCS 中常见为 **两个布局控制点**，对应三次贝塞尔：

- P0 = 路径 **起点**（source 点，mm→m）
- P1、P2 = 两个布局控制点经**同样变换**后的世界坐标
- P3 = 路径 **终点**（destination 点）

参数 `t ∈ [0,1]` 由**已走过的路径长度 / 该边在 XML 中的 `length`（mm）** 换算（与仿真适配器里用路径长度归一化的思路一致）；在 `t` 处取曲线点及导数得到 `theta`。

### 订单边与植物路径的对应关系

对 VDA `order` 中 `edges` 按 **`sequenceId` 排序**，对**每一条边**在植物模型中查找路径：

1. **优先**用 `edgeId` 当作 OpenTCS 的 **path 名**（若配置了 `pathPrefix`，会尝试带前缀的候选名，便于与 `Vda5050MapNameCodec` 剥离规则一致）。
2. **否则**用 `startNodeId`、`endNodeId` 与 XML 里 path 的 `sourcePoint` / `destinationPoint` 匹配（同样支持 `pointPrefix` 候选）。

每条边生成一段 `MotionSegment`；多段边按顺序串联，时间步内可按速度**连续跨段**推进。

### 速度与推进

- 默认速度：`simulation.defaultSpeedMps`（m/s）。
- 若某条边在 JSON 中带 **`maxSpeed`**，该段使用对应值（与 VDA 边属性一致，单位按 AOS 下发约定，通常为 m/s）。

推进使用**时间步剩余时间**在段内、段间分配，避免单帧跨多段时距离错误。

### `lastNodeId` 与 AOS 确认（MessageResponseMatcher）

- 在**每一段行程进行中**：`lastNodeId` 为该段 **起点**（OpenTCS source 点名），表示车尚未完成本段、未到终点。
- **段结束时**：`lastNodeId` 更新为该段 **终点**（destination 点名）；若还有下一段，下一段从该点继续。
- **整单最后一段完成**：`lastNodeId` 为订单**最后一个节点**，`lastNodeSequenceId` 与末节点一致，此时 AOS 侧才可能以「到达 MovementCommand 目标点」的方式确认 `orderAccepted`（与 `lastNodeId` 对齐目的地名的逻辑一致）。

### 配置项小结（地图）

| 配置项 | 含义 |
|--------|------|
| `map.xmlPath` | OpenTCS 导出的 `.xml` 绝对路径 |
| `map.mapId` | 写入 `state`/`visualization` 的 `mapId`；不填则用 XML 根 `model/@name` |
| `map.applyStripping` | 与 `aos.vda5050.map-name-prefixes.apply-stripping` 一致；为 false 时上下行均用全名 |
| `map.pointPrefix` / `map.pathPrefix` | 与 `yeefungagv.plc.name-prefixes` 一致；为 true 时 `state.lastNodeId` 上报剥离后的 nodeId（同 `Vda5050MapNameCodec`） |
| `map.layoutScaleMm` / `map.layoutFlipY` | 布局控制点 → 世界 mm 的变换，默认与 `AgvCommunicationAdapter` 一致 |

---

## Behaviour (MVP)

- On connect, publishes `connection` with `connectionState: ONLINE`.
- Keeps `operatingMode` as `AUTOMATIC` (configurable).
- For each received `order`, motion follows the **Motion logic** section above; `lastNodeId` updates as described for `MessageResponseMatcher.orderAccepted` alignment.
- For `instantActions`, reports each action as `FINISHED` in the next `state` messages (`actionId` must match).
- `cancelOrder` clears the current order simulation and emits a finished action state.

## See also

- [src/main/resources/application.yaml](src/main/resources/application.yaml) — defaults, MQTT QoS, and `map.*` keys
- [src/main/resources/simulator-mqtt-matrix.yaml](src/main/resources/simulator-mqtt-matrix.yaml) — topic/QoS reference table
