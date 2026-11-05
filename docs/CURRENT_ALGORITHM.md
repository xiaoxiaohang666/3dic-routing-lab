# 当前算法讲解（2026-11-05）

本文描述本工作树 `codex/fixed-resource-handoff-20261105` 的路由器和现有入口。代码与入口比旧版架构说明更新得更快；涉及实际运行配置时，以 `scripts/run_current.sh`、`scripts/run_fixed_resource_candidate.sh` 和 `scripts/run_package.sh` 为准。本文是算法说明，不把离线预测称为官方成绩。

## 目标与表示

输入为 DEF、LEF 和热场 JSON。路由器读取真实网络、实例方向及 PIN/OBS 几何、DEF TRACKS 和 GCell 网格，在双 Die 的 M1–M20 上为每网输出一组 Guide 矩形。平面走线遵守层首选方向；同 Die 换层使用相邻层重叠，跨 Die 网络恰好经 M10/M11 的一个 TSV。TSV 的禁止落点只依据 M10/M11 的 PIN/OBS，这是当前生产入口和官方答疑采用的范围。

优化对象是 Guide 的线长 W、资源拥塞 C、TSV 与热场的相关性 T，以及它们组成的完整 S。搜索时的边价只是产生候选的近似值；是否保留修改取决于重新编码 Guide 后的完整度量与合法性。物理边容量、Guide 资源账本和评分读回各有用途，不能互相代替。

```text
DEF / LEF / 热场
    ↓ 解析真实 PIN、层、网格、轨道和热值
RoutingGraph：合法边、容量、固定占用、TSV 候选
    ↓ 跨 Die 网的 TSV 分配；可选未来 TSV 资源预留
逐网接入 PIN/TSV、构建初始连接树
    ↓ Guide 编码、全量检查和首次计量
热点 rip-up/reroute → 动态 TSV/重匹配 → 有预算的线长恢复
    ↓ 全量检查、导出、序列化读回
Guide + stats；另用独立入口读回计分与校验
```

## 1. 建图和资源账本

`src/parser/` 将 DEF/LEF 坐标、实例变换和多形状 PIN 映射到实际金属层；`src/router/routing_graph.*` 在 GCell 中心建三维图，提供层方向、相邻层 via、轨道容量、固定 PIN/OBS 需求及 TSV 可用性。M10/M11 的 TSV 需求按其 5×5 μm 几何和轨道覆盖计算。同网 TSV 与 Guide 在落点处不重复计数；不同网需求仍相加。`src/objectives/net_cell_contribution.hpp` 汇集逐网逐格贡献，供搜索、全量计量和候选报价使用。

固定占用有两个明示模型：默认 `production` 遵循项目既有规则；`observed-20261104` 是从平台样本反推并经几份 Guide 返回验证的候选模型。后者仍有上边界文字冲突、零容量和隐藏例盲区，因此只在显式选用的候选/代码包入口中启用。见 [固定资源模型](rules/fixed-resource-model-20261105.md)。

## 2. TSV 分配与初始树

`src/router/tsv_policy.*` 枚举上下 Die 匹配的合法落点，以距离、资源和热等代价给跨 Die 网分配槽位。默认每格至多一个 TSV；预布线同格 hub 和自动选择是独立实验能力，并非当前 `run_package.sh` 设置。`future_tsv_reservation.cpp` 可把尚未布线网络的必要 TSV 需求临时计入搜索价；该网开始布线时移除对应预留，初始阶段结束后检查预留账本清零。

`NegotiatedRouter::route_all()` 当前先布跨 Die 网，再按 PIN 数、HPWL 和网名稳定排序。每网由真实 PIN 接入和已选 TSV 落点构成终端，使用接树搜索生成路径，并编码为符合 GCell 的矩形。搜索结合长度、via、当前及历史拥塞价格；已有树段可以复用。大设计使用有限预算的快速初始搜索，仍要经过该网及完整 Guide 的合法性检查。此前试过按引脚出线难度调初始顺序，三公开例差异在波动内，开关已撤销。

`--climb-escape-radius 3` 处理一种已定位的 003 拥塞原因：从 PIN 正上方逐层爬升可能经过预先已满的格子。若目标层柱子拥塞，算法在终端周围半径内按实际边价搜索可到达点，并结合到另一终端的距离选出口；直接爬升代价合适时仍可保留。该机制限于同 Die，不改变 TSV 数量或合法性约束。官方模式配合 a150-20261029 和 fast quality 时默认半径 3；显式传 0 可关闭。代码在 `src/router/climb_escape.cpp`。

## 3. 协商布线与后处理

初始树经全量校验后，`hotspot_rrr_router.cpp` 选资源热点及相关网络，成组拆线、重布并比较完整质量；失败或超时通过日志/检查点回滚。历史价格抑制网络反复占用同一热点。`--rrr-reserved-seconds`、轮数和网数上限控制候选入口的投入，不改变单次作业总预算。

后续可用 `dynamic_tsv_router.cpp` 在合法落点间移动 TSV 并重连整网，`tsv_rematch_router.cpp` 调整落点归属，`budgeted_recovery_router.cpp` 在剩余预算内尝试降低线长。实际启用哪些阶段取决于入口和配置。局部报价仅用来排序；候选须重新计量完整 Guide 的 L/Q/rho/W/C/T/S，再按验收规则保留或回滚。`via-stack-radius` 与 `recovery-global-wirelength` 仍默认 0；多 owner 同格 TSV 的通用动态搜索尚未实现。

## 4. 三个常用入口

| 入口 | 热增益与资源模型 | 当前用途 |
| --- | --- | --- |
| `scripts/run_current.sh` | 必须显式传 G；默认 `production` | 生产规则的可控研究与对照；A150/documented/site-20261029，TSV 禁放只看 M10/M11 |
| `scripts/run_fixed_resource_candidate.sh` | 显式 G；`observed-20261104` | 候选研究：RRR 预留 2000 秒、64 轮、较高网数上限、目标 Q=30、未来 TSV 预留、横走半径 3，默认 3000 秒 |
| `scripts/run_package.sh` | 平台不传 G，使用 hottest-K 估计；`observed-20261104` | 代码包入口；其余关键路由设置与候选入口对应，显式半径 3；包的 `run.sh` 给 3000 秒 |

入口的后置参数允许显式覆盖部分算法选项。`run_package.sh` 的 G 估计与研究时输入的已知 G 不同，不能只因其他设置相同就预言相同成绩。离线计分须记录所选模型、G、输入、二进制、Guide 和预算。

## 5. 输出验证与现有证据

导出前检查网络、PIN 接入、连通、方向、TSV 映射和 GCell 编码；写盘后重新读取实际序列化矩形再检查。`eda_validate_guide` 和 `eda_measure_documented` 可对已有 Guide 独立读回，后者与路由器共享部分实现，因此并非官方评分器的独立证明。

截至本说明，11 月 5 日官方快评确认逐例 PASS 的现有 Guide 为 001 `ee5d7301`、003 `b61d5a99`、005 `438122a4`（哈希前缀）；其中 001、003 来自另一批上传。横走半径 0→3 的同作业离线对照为 001 78.62→78.64、003 60.73→65.72、005 61.19→59.95；005 的退化原因未查实。最新代码包入口三例尚未重新布线，官方成绩 UNKNOWN，Best 未因入口变化更新。隐藏例和最终聚合收益也 UNKNOWN。详见 [项目状态](../PROJECT_STATUS.md)、[实验记录](../EXPERIMENT_LOG.md) 和 [决策记录](../DECISIONS.md)。
