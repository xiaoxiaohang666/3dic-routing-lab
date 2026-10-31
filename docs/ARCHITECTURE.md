# 代码结构与修改入口

本路由器用 C++20 读取 DEF、LEF 和热输入，生成双 die 多层全局布线 Guide。统一基线保留成熟流程及默认关闭的实验算子；内部版本参数不代表需维护同名长期分支。规则见 [CONTEST.md](CONTEST.md)，工具见 [SCORING.md](SCORING.md)，接手流程见 [AGENT_HANDOFF.md](AGENT_HANDOFF.md)。

## 输入到输出

```text
DEF + LEF + thermal JSON
  → 解析实例/方向/真实层 PIN、轨道、GCell、热图
  → RoutingGraph + 固定资源 + TSV 合法掩码
  → TSV 分配 → PIN 接入与初始树 → 协商布线
  → RRR / 线长恢复 / 动态 TSV / 局部实验算子
  → 完整 Guide 编码与 S 验收 → 检查点/回滚
  → 导出 → 独立重读与合法性/资源评分
```

## 核心模块

| 路径 | 职责 | 修改时注意 |
| --- | --- | --- |
| `src/main.cpp` | CLI、profile、输入校验、流程装配、stats、导出 | 显式记录 rules/objective/G；默认 CLI 与统一入口不一定同配置 |
| `src/parser/` | DEF/LEF/thermal、坐标单位和实例变换 | 实际层、多形状 PIN、轨道边界、LEF ORIGIN、DEF IO 影响资源合同 |
| `src/router/routing_graph.*` | 多层网格、容量/固定占用、接入、TSV 禁放 | 资源变化须同步搜索与独立读回 |
| `src/objectives/net_cell_contribution.hpp` | 单网/单层/单格 Guide/TSV 贡献、真实 PIN 抵扣 | 增量、全量和报价共同遵循的接口 |
| `src/objectives/official_metrics.*` | L、reference、Q、rho 和完整 S | 完整 S 决定取舍；名称 official 不等于官方认证 |
| `src/objectives/scoring_profile.hpp` | profile、Q 缩放、via 和热参数 | 旧兼容模式仅复现，不能混用分数或默改权重 |
| `src/scoring/compat_resources.*` | 独立固定资源计量 | 生产规则与诊断假设分开；不能用拟合替代闭 PIN 上边界 |
| `src/scoring/compat_validation.*` | Guide/PIN/网络结构读回 | 工程 PASS 不等于完整详细布线 DRC |
| `src/scoring/site_wirelength.*` | 观测到的 Guide 几何与相邻层费用 | 真实 via 计足，不能拆矩形逃费 |
| `src/validation/` | Guide 编码、全网合法性、导出前后检查 | 最终按真实序列化 Guide 计费 |
| `tests/` | 合同、事务、算子、CLI、包装脚本回归 | 旧版本名仍可能覆盖当前通用行为，删除前查依赖 |
| `benchmarks/sample001/` | 小型功能夹具 | 不代表正式公开例或隐藏例质量 |

## 路由文件职责

| 文件 | 职责 |
| --- | --- |
| `negotiated_router.cpp/.hpp` | 协商搜索、共享价格、候选 key、完整验收、状态接口；多 Agent 共享核心 |
| `pin_local_router.cpp`、`tsv_local_router.cpp` | PIN/TSV 局部接入和连接 |
| `tsv_policy.cpp/.hpp` | TSV 合法候选、热相关、索引和分配策略 |
| `dynamic_tsv_router.cpp` | 动态 TSV 移动、落点报价及完整重连 |
| `tsv_rematch_router.cpp`、`tsv_tree_move.cpp` | TSV 重匹配和保留树迁移实验 |
| `future_tsv_reservation.cpp` | 未来 TSV 资源保留及状态交接 |
| `hotspot_rrr_router.cpp` | 热点网络筛选、分组 rip-up/reroute、接受与回滚 |
| `rrr_checkpoint.cpp` | 重布检查点、增量重用和完整回退 |
| `budgeted_recovery_router.cpp` | 有预算的线长恢复、候选筛选和重连 |
| `via_stack_repair.cpp` | 默认关闭的 via 栈/子栈挪位，保护 PIN、接入和 TSV |
| `low_layer_repair.cpp` | 低层资源热点修复实验 |
| `adaptive_schedule.cpp` | 算子预算和调度 |
| `cost_diagnostics.*` | 搜索与报价诊断 |
| `baseline_router.*` | 基础路由及历史功能回归 |

现有恢复和动态 TSV 事务已经按完整 S 接受，也允许 Q 变大。v13 主要补候选覆盖、改善报价和局部重连能力，并非首次允许拥塞换线长/热收益。

## 报价与真实验收

搜索代价用于产生候选。最终须按同一规则编码完整 Guide，计算真实 L、Q、rho、W/C/T/S，再执行组验收和检查点规则。导数价格、单网长度界和 TSV 落点代理不能代替完整验收。

`dynamic_tsv_router.cpp` 当前将完整现状 Q 的导数与移除整网后的增量混用。待实现的同基态有限差分为：

```text
Q_b = 移除旧网后，剩余账本的 Q
m(slot) = 在同一账本加入候选必要 TSV 需求的 ΔQ
cost_C(slot) = 30 × [1 / sqrt(1 + Q_b)
                    - 1 / sqrt(1 + Q_b + m(slot))]
```

**该有限差分报价尚未实现。** 新旧 slot 使用同一 Q_b；缓存只在账本未变的单次查询内有效。此式仅报价必要 TSV，实际重连后的布线/via、热相关和完整 S 才决定接受。须保持报价非负与全局索引下界等假设，并用完整重新计费核对。

多网同格另需改造 owner/free-slot 索引和依赖二值密度的热检索系数。Pearson 状态支持 `count_square`，不代表整条检索和事务链已支持多重占用；必须测计数密度、排序及回滚。

## 构建和命令入口

CMake 收集 `src/*.cpp` 构建静态 `eda_core`，主程序是 `eda_router`。主要独立目标：

| 目标 | 用途 |
| --- | --- |
| `eda_validate_guide` | 输出 Guide 重读和合法性检查 |
| `eda_measure_documented` | 记录真实线长、资源、TSV/热和分项分数 |
| `eda_score_compat` | 历史兼容评分及受控差异诊断 |
| `eda_diagnose_site` | 官方观测与本地模型对账 |
| `eda_diagnose_pin_floor` | PIN floor/轨道假设对照和拥塞上界诊断 |

推荐 `scripts/run_current.sh` 和 `scripts/score_current.py`。内部 `run_v7/v8/v9`、`run_tsv_resource*`、`run_a150*` 保留为成熟 preset、回归依赖或历史消融；不能只看文件名判断规则，应记录实际执行参数及 stats。`submission/` 是打包模板，目录存在不等于已有当前正式可上传包。

构建、CTest、sanitizer、布线和评分在 **benchmark-host** 独立任务目录执行。需要 Linux x86-64、C++20/CMake 3.22+、Python 3 和匹配 FLUTE 资产；运行库条件见 [SCORING.md](SCORING.md)。不要在本地旧工作树生成新构建或实验状态。

## 证据

- [来源清单](provenance/source-manifest.json)：迁移的实际文件，不能只用旧 HEAD 追溯。
- [10 月 31 日资料](../research/evidence/20261031/)：迁移前源码清单、测试日志、003 配对摘要、拥塞松弛上界。
- [当前计划](archive/llh-v13-tradeoff-plan-20261031.md)：已实现内容、待办与完整预算标准。
- `research/agy-next-direction-20261030/scripts/`：旧统计分析，部分上界及建议已被 10 月 31 日报告修正。

生产实现、诊断假设和实际完成实验要明确区分。开 PIN 边界等拟合不能驱动生产验收；数学上界不是实测收益；005 WGLOBAL 的短预算微小差异见 research/evidence/20261031/README.md，尚无完整预算稳定收益。新实验用独立证据目录，默认行为经稳定配对后再升级。
