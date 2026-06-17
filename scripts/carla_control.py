import argparse
import socket
import sys
import time


def wait_for_port(host, port, timeout):
    deadline = time.time() + float(timeout)
    last_error = None
    while time.time() < deadline:
        try:
            with socket.create_connection((host, int(port)), timeout=2.0):
                return True
        except OSError as exc:
            last_error = exc
            time.sleep(1.0)
    raise RuntimeError(f"CARLA port {host}:{port} is not reachable after {timeout}s; last_error={last_error}")


def connect_client(host, port, timeout):
    import carla

    client = carla.Client(host, int(port))
    client.set_timeout(float(timeout))
    return client


def command_play():
    try:
        import unreal
    except ImportError as exc:
        raise RuntimeError("Unreal Python module is unavailable. Run this inside the Unreal Editor Python environment.") from exc
    unreal.EditorLevelLibrary.editor_play_simulate()


def main():
    parser = argparse.ArgumentParser(description="CARLA helper for RoadTailBench Leaderboard.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--timeout", default=120.0, type=float)
    parser.add_argument("--wait", action="store_true", help="Wait until the CARLA port is reachable.")
    parser.add_argument("--map", default="", help="Map name to load with carla.Client.load_world().")
    parser.add_argument("--sleep-after-load", default=3.0, type=float)
    parser.add_argument("--print-world", action="store_true")
    parser.add_argument("--command-play", action="store_true", help="Call unreal.EditorLevelLibrary.editor_play_simulate().")
    args = parser.parse_args()

    if args.command_play:
        command_play()
        return 0

    if args.wait:
        wait_for_port(args.host, args.port, args.timeout)

    client = connect_client(args.host, args.port, args.timeout)
    if args.map:
        print(f"[carla-control] loading map: {args.map}", flush=True)
        world = client.load_world(args.map)
        if args.sleep_after_load > 0:
            time.sleep(float(args.sleep_after_load))
    else:
        world = client.get_world()

    if args.print_world:
        print(f"[carla-control] world={world.get_map().name}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"[carla-control] error: {exc}", file=sys.stderr, flush=True)
        raise
