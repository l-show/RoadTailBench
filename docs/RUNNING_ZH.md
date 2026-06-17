# 运行 RoadTailBench

Dry run：

```powershell
rtb-run `
  --scene-root G:\Codex\RoadTailBench\scenes\rtb116_125 `
  --metadata-root G:\Codex\RoadTailBench\metadata\rtb116_125 `
  --scenes RTB116-RTB125 `
  --dry-run
```

RTB 脚本自己生成和控制 ego：

```powershell
rtb-run `
  --host localhost `
  --port 2000 `
  --scene-root G:\Codex\RoadTailBench\scenes\rtb116_125 `
  --metadata-root G:\Codex\RoadTailBench\metadata\rtb116_125 `
  --scenes RTB116 `
  --ego-mode scene_ego
```

模型控制 ego：

```powershell
rtb-run `
  --host localhost `
  --port 2000 `
  --scene-root G:\Codex\RoadTailBench\scenes\rtb116_125 `
  --metadata-root G:\Codex\RoadTailBench\metadata\rtb116_125 `
  --scenes RTB116 `
  --ego-mode agent_ego `
  --agent roadtailbench_zoo.adapters.rule_based:RuleBasedAdapter
```
