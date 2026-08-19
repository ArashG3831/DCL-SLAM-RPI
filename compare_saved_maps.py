#!/usr/bin/env python3

import sys
import math
from pathlib import Path

import yaml
import numpy as np
from PIL import Image


def load_map(yaml_path: Path):
    meta = yaml.safe_load(yaml_path.read_text())
    image_path = Path(meta["image"])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path

    img = Image.open(image_path).convert("L")
    arr = np.array(img)

    # map_saver convention:
    # black/low = occupied, white/high = free, gray around 205 = unknown
    cls = np.full(arr.shape, -1, dtype=np.int8)  # -1 unknown
    cls[arr < 50] = 100                         # occupied
    cls[arr > 250] = 0                          # free

    # Convert image top-left indexing to map bottom-left indexing.
    cls = np.flipud(cls)

    res = float(meta["resolution"])
    origin = meta["origin"]
    ox = float(origin[0])
    oy = float(origin[1])
    yaw = float(origin[2]) if len(origin) > 2 else 0.0

    if abs(yaw) > 1e-6:
        print(f"WARNING: {yaml_path} has nonzero origin yaw={yaw}. This simple comparer assumes yaw=0.")

    h, w = cls.shape
    return {
        "yaml": yaml_path,
        "image": image_path,
        "grid": cls,
        "res": res,
        "origin": (ox, oy),
        "w": w,
        "h": h,
        "max_x": ox + w * res,
        "max_y": oy + h * res,
    }


def place_on_canvas(maps):
    res_values = [m["res"] for m in maps]
    ref_res = res_values[0]
    for r in res_values:
        if abs(r - ref_res) > 1e-9:
            raise RuntimeError(f"Map resolutions differ: {res_values}")

    min_x = min(m["origin"][0] for m in maps)
    min_y = min(m["origin"][1] for m in maps)
    max_x = max(m["max_x"] for m in maps)
    max_y = max(m["max_y"] for m in maps)

    W = int(math.ceil((max_x - min_x) / ref_res)) + 2
    H = int(math.ceil((max_y - min_y) / ref_res)) + 2

    canvases = []
    for m in maps:
        canvas = np.full((H, W), -1, dtype=np.int8)
        gx0 = int(round((m["origin"][0] - min_x) / ref_res))
        gy0 = int(round((m["origin"][1] - min_y) / ref_res))
        h, w = m["grid"].shape
        canvas[gy0:gy0+h, gx0:gx0+w] = m["grid"]
        canvases.append(canvas)

    return canvases, ref_res


def pct(x):
    return 100.0 * x


def main():
    if len(sys.argv) < 3:
        print("Usage:")
        print("  python3 compare_saved_maps.py map1.yaml map2.yaml [map3.yaml ...]")
        print()
        print("Example:")
        print("  python3 ~/compare_saved_maps.py ~/robot_map_tests/session_x/run_*/map.yaml")
        sys.exit(1)

    paths = [Path(p).expanduser() for p in sys.argv[1:]]
    maps = [load_map(p) for p in paths]
    canvases, res = place_on_canvas(maps)

    ref = canvases[0]
    ref_name = maps[0]["yaml"]

    print()
    print("Reference map:", ref_name)
    print("Resolution:", res)
    print()
    print("map,known_area_m2,common_known_m2,known_overlap_pct,cell_disagreement_pct,occupied_iou,free_iou")

    ref_known = ref >= 0
    ref_occ = ref == 100
    ref_free = ref == 0

    for m, cur in zip(maps, canvases):
        cur_known = cur >= 0
        cur_occ = cur == 100
        cur_free = cur == 0

        common_known = ref_known & cur_known
        known_union = ref_known | cur_known

        known_area = cur_known.sum() * res * res
        common_area = common_known.sum() * res * res
        known_overlap = (common_known.sum() / max(1, known_union.sum()))

        disagreement = ((ref != cur) & common_known).sum() / max(1, common_known.sum())

        occ_iou = (ref_occ & cur_occ).sum() / max(1, (ref_occ | cur_occ).sum())
        free_iou = (ref_free & cur_free).sum() / max(1, (ref_free | cur_free).sum())

        print(
            f"{m['yaml']},"
            f"{known_area:.4f},"
            f"{common_area:.4f},"
            f"{pct(known_overlap):.2f},"
            f"{pct(disagreement):.2f},"
            f"{occ_iou:.4f},"
            f"{free_iou:.4f}"
        )

    print()
    print("How to read:")
    print("  known_area_m2: how much area the map discovered.")
    print("  known_overlap_pct: how much known area overlaps with the reference map.")
    print("  cell_disagreement_pct: lower is better.")
    print("  occupied_iou: wall/obstacle similarity. Higher is better.")
    print("  free_iou: free-space similarity. Higher is better.")
    print()
    print("For strict comparison, start every run from the same physical pose.")


if __name__ == "__main__":
    main()
