import os


def _truthy(value):
    return str(value or "").strip().lower() in {"1", "true", "yes", "on"}


def _main():
    try:
        import unreal
    except ImportError:
        return

    map_name = os.environ.get("LEADERBOARD_UE_MAP") or os.environ.get("ROADTAILBENCH_UE_MAP")
    command_play = _truthy(os.environ.get("LEADERBOARD_COMMAND_PLAY")) or _truthy(os.environ.get("ROADTAILBENCH_COMMAND_PLAY"))

    if map_name:
        unreal.log(f"[RoadTailBench] UE startup map requested: {map_name}")

    if command_play:
        unreal.log("[RoadTailBench] UE startup command play requested.")
        unreal.EditorLevelLibrary.editor_play_simulate()


_main()
