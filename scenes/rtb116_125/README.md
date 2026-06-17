# RTB116-RTB125 Scenes

These scene scripts are treated as RoadTailBench-owned code scenarios. Each `RTBXXX.py` owns the scenario's weather, actors, hazards, ego script, and dynamic behavior.

The initial migration keeps script behavior unchanged. Some scripts still contain legacy absolute helper-library paths; because `RoadTailBenchInitV9.py` is placed in this directory, future cleanup should first switch imports to the local copy, then verify each scene in CARLA.

Do not add Bench2Drive XML/XOSC files here. Metadata for evaluation lives in `../../metadata/rtb116_125`.
