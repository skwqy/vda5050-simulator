# 地图目录（与 AOS 一致）

本目录用于放置 **OpenTCS 驾驶层（driving course）XML**，须与 **AOS / 内核当前使用的植物模型为同一份文件**（同一导出、同一版本），模拟器才会按真实 POLYPATH/BEZIER 运动；否则请见主 `README.md` 中的回退直线逻辑。

## 目录约定

| 路径（相对 `vda5050-simulator/`） | 用途 |
|-----------------------------------|------|
| `maps/active.xml` | **当前启用地图**。`application.yaml` 里 `map.xmlPath` 默认指向此文件；更换地图时只需替换该文件或改配置指向其它文件。 |
| `maps/archive/`（可选） | 自行建立，用于存放历史版本或多项目备份，例如 `youle-final-1.xml`、`plant-2026-04.xml`。 |
| `maps/README.md` | 本说明。 |

## 与 AOS 保持一致

1. 在 AOS / Model Editor 中导出植物模型，或直接使用内核加载的 **同一份** `.xml`（OpenTCS 6 `model` 格式）。
2. 将文件复制到本目录，命名为 `active.xml`（或复制为任意名称并在 `application.yaml` 的 `map.xmlPath` 中写绝对/相对路径）。
3. AOS 侧车辆 `topicPrefix` 中的 **`manufacturer` / `serialNumber` / `interfaceName`** 须与模拟器 `application.yaml` 中 `mqtt.*` 一致（当前示例将 `serialNumber` 设为 `Vehicle_2` 时需与该车配置相同）。

## 切换地图

- **方式 A**：用新文件覆盖 `maps/active.xml`，重启模拟器。
- **方式 B**：保留多份 XML，修改 `map.xmlPath` 指向例如 `maps/archive/youle-final-1.xml`，或通过 `-Dsimulator.config=...` 指定另一套 YAML。

## Git

若地图含现场数据、体积大，可在本目录使用 `.gitignore` 忽略 `*.xml`（本仓库已提供），仅本地保留 `active.xml`。
