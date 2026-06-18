import argparse
import glob
from pathlib import Path


CAM_NAMES = [
    "CAM_FRONT",
    "CAM_FRONT_RIGHT",
    "CAM_FRONT_LEFT",
    "CAM_BACK",
    "CAM_BACK_RIGHT",
    "CAM_BACK_LEFT",
]


def synth_360(run_dir, fps=20.0, output_name="360_surround.mp4"):
    import cv2
    import numpy as np

    video_dir = Path(run_dir) / "video"
    folders = {name: video_dir / name for name in CAM_NAMES}
    missing = [name for name, path in folders.items() if not path.exists()]
    if missing:
        raise SystemExit(f"Missing frame folders for 360 synthesis: {', '.join(missing)}")

    front_images = sorted(glob.glob(str(folders["CAM_FRONT"] / "*.jpg")) + glob.glob(str(folders["CAM_FRONT"] / "*.png")))
    if not front_images:
        raise SystemExit("CAM_FRONT has no saved frames. Run with --video-save-frames first.")

    output_path = video_dir / output_name
    writer = cv2.VideoWriter(str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), float(fps), (1920, 1080))
    crop_l, crop_r, center = 270, 1010, 640
    written = 0
    for img_path in front_images:
        frame_name = Path(img_path).name
        imgs = {}
        ok = True
        for name, folder in folders.items():
            path = folder / frame_name
            img = cv2.imread(str(path))
            if img is None:
                ok = False
                break
            imgs[name] = img
        if not ok:
            continue

        pano = np.concatenate([
            imgs["CAM_BACK"][:, center:crop_r],
            imgs["CAM_BACK_LEFT"][:, crop_l:crop_r],
            imgs["CAM_FRONT_LEFT"][:, crop_l:crop_r],
            imgs["CAM_FRONT"][:, crop_l:crop_r],
            imgs["CAM_FRONT_RIGHT"][:, crop_l:crop_r],
            imgs["CAM_BACK_RIGHT"][:, crop_l:crop_r],
            imgs["CAM_BACK"][:, crop_l:center],
        ], axis=1)
        pano_resized = cv2.resize(pano, (1920, int(720 * 1920 / 4440)))
        canvas = np.zeros((1080, 1920, 3), dtype=np.uint8)
        canvas[35:35 + pano_resized.shape[0], 0:1920] = pano_resized
        grid_y = 1080 - 720
        canvas[grid_y:grid_y + 360, 0:640] = cv2.resize(imgs["CAM_FRONT_LEFT"], (640, 360))
        canvas[grid_y:grid_y + 360, 640:1280] = cv2.resize(imgs["CAM_FRONT"], (640, 360))
        canvas[grid_y:grid_y + 360, 1280:1920] = cv2.resize(imgs["CAM_FRONT_RIGHT"], (640, 360))
        canvas[grid_y + 360:1080, 0:640] = cv2.resize(imgs["CAM_BACK_LEFT"], (640, 360))
        canvas[grid_y + 360:1080, 640:1280] = cv2.resize(imgs["CAM_BACK"], (640, 360))
        canvas[grid_y + 360:1080, 1280:1920] = cv2.resize(imgs["CAM_BACK_RIGHT"], (640, 360))
        writer.write(canvas)
        written += 1
    writer.release()
    print(output_path)
    print(f"[leaderboard-video] wrote {written} frames")
    return str(output_path)


def main():
    parser = argparse.ArgumentParser(description="RoadTailBench video utilities.")
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--mode", choices=["synth-360"], default="synth-360")
    parser.add_argument("--fps", default=20.0, type=float)
    args = parser.parse_args()
    if args.mode == "synth-360":
        synth_360(args.run_dir, args.fps)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
