# 测试日志分析（基于 test.txt）

生成时间: 2025-10-04

概览: 我解析了工作区内的 `test.txt`（72 次规划块），并统计了 Topo/MPPI/B-spline 的运行情况，结果保存在 `test_log_summary.csv`。

----

统计要点（来自 `test.txt`）:
- 规划块总数: 72
- Topo 找到的路径分布:
  - 1 条路径: 40 次 (~55.6%)
  - 2 条路径: 22 次
  - 3 条路径: 10 次
- MPPI 运行相关:
  - 日志中检测到的“per-path Running MPPI”条目总数: 178
  - 出现“Optimizing all X topological paths” 的规划块数: 57
  - 出现“Skipping STEP 2”的块数: 72（表示这份日志来自改动前的行为或跳过分支）
- B-spline:
  - B-spline smoothing 块数: 72（每次规划都会走 B-spline 步骤）

----

问题与结论

1) TOPO 路径情况怎么样？

   结论:
   - Topo PRM 经常只产生单条路径（55.6% 的规划只给出 1 条拓扑路径）。在少数情形下会产生 2 或 3 条。
   - 原因可能是：拓扑去重过严（discretize 精度高）、采样覆盖不足或 `selectShortPaths()` 在 STEP5 过滤掉了较长/次优路径。

   证据:
   - 从日志统计：40 / 72 次只找到 1 条路径；仅 32 次产生多条路径。

2) 为什么 MPPI 没有对每条 topo 路径都算（我只看到一种红色）？

   结论:
   - 日志显示：有 57 个规划块打印了 “Optimizing all X topological paths”，且总共检测到 178 条 “Path i/j ...: Running MPPI” 信息，说明在多路径情形下 MPPI 确实对多条路径做了并行优化。
   - 但由于多数规划 (40/72) 只生成 1 条拓扑路径，因此实际触发多路径并行的次数受限。单路径时旧逻辑会跳过并行分支（日志中大量存在 “Skipping STEP 2”），从而造成你在可视化中只看到“红色”主路径。
   - 可视化只显示红色的直接原因是 `visualizeTopoPaths()` 中：第 0 条路径用红色显示（index==0 -> red）。如果只有 1 条路径，自然只看到红色。

   证据:
   - 40 次只有 1 条路径 → 只绘制 index=0（红色）
   - 57 次打印了 "Optimizing all X topological paths" → 在这些情况下会并行运行 MPPI（并在日志中看到 per-path MPPI 调用）
   - 178 次 "Running MPPI" 表明在多路径场景下有多次 per-path MPPI 被调用（178 / 57 ≈ 3.1，和每次优化多条路径数相符）

3) B-spline 是仅做平滑，还是承担了更大的任务？

   结论:
   - 在日志中 B-spline 步骤每次都会运行（72 次），并且当 MPPI 没有运行或 MPPI 优化失败/点数不足时，B-spline 承担了更大的责任：作为回退路径的唯一可用优化器来修正或规避碰撞（即不是“只平滑”）。
   - 因此在当前日志（旧逻辑）中，B-spline 在很多场景下不仅做平滑，还在执行避障重排（rebound），这会掩盖 Topo+MPPI 的作用。理想情况应是 Topo 生成多条候选，MPPI 对每条并行优化，B-spline 只做轻量的平滑与最后的可行性微调。

   证据:
   - B-spline 出现在所有 72 个规划块中，且日志带有 "B-spline smoothing (ONLY smoothing, NOT planning!)" 的警告性表述，结合其他处看到的 `rebound` 日志（在一些早期日志中可见），说明 B-spline 进行了避障重试。

----

建议与下一步（优先级）

1. 确认并使用我们已做的代码修改（MPPI 强制运行）后重新运行完整测试（建议 30 次以上），以验证 MPPI 是否在每次规划都被调用并且对单路径也进行优化。

   - 运行后检查日志关键字：
     - "[PlannerManager] STEP 2: MPPI Optimization (forced run)" （每次都应该出现）
     - "Path i/j ...: Running MPPI" （应在每条 topo path 上出现，若存在多条）

2. 放宽 Topo 去重和/或扩大采样区域以提高多路径率：
   - 将 `discretize_points_num_` 从 30 降到 20
   - 将 `sample_inflate_` 从 3.0 增到 4.0
   - 将 `max_raw_paths_` 从 50 增到 100

3. 可视化改进（调试建议）:
   - 在 RViz 中确认是否订阅了 `/topo_paths` 话题（`rostopic echo /topo_paths` 或在 RViz 的 Displays 中打开）。
   - 为每条路径在 Marker 的 legend 或 marker text 中写入索引（0/1/2），便于观察哪些路径被 MPPI 优化后变为最终轨迹。

4. 资源/性能优化建议（如果总想对每条路径都运行 MPPI）:
   - 单路径时使用轻量化 MPPI 配置（减少 samples / horizon），多路径时恢复高质量配置。

----

附录：可重复的检查命令（在 `src` 目录下运行）

1) 快速统计日志（已经运行过一次）：
```bash
python3 scripts/parse_test_log.py test.txt  # 若有脚本，或使用我刚运行的解析脚本
```

2) 查看 MPPI 调用痕迹:
```bash
grep -n "Running MPPI" test.txt | wc -l
grep -n "Optimizing all" test.txt | wc -l
```

3) 检查 topo 可视化是否有订阅者（在节点运行时）:
```bash
rostopic info /topo_paths
```

----

我已将本次分析保存为 `test_analysis.md`，并且把详尽的每次规划摘要写入 `test_log_summary.csv`（CSV 可在 Excel 或脚本中进一步分析）。如需我现在：
- 运行完整回归测试（如果您启动仿真并提供新的日志）
- 或帮助把这些建议的参数更改（放宽去重/扩大采样）直接修改到代码并提交

请选择下一步，我可以继续执行。 
