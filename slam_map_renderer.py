#!/usr/bin/env python3
"""Reproducible common-scale OccupancyGrid rendering."""

import json
import math
import os

from PIL import Image, ImageDraw


def load_map(path):
    with open(path, encoding="utf-8") as stream:
        value = json.load(stream)
    if value.get("width", 0) <= 0 or value.get("height", 0) <= 0:
        raise ValueError(f"invalid map dimensions in {path}")
    if len(value.get("data", [])) != value["width"] * value["height"]:
        raise ValueError(f"map data length does not match dimensions in {path}")
    return value


def common_canvas(maps):
    resolution = float(maps[0]["resolution"])
    if any(abs(float(item["resolution"]) - resolution) > 1e-9 for item in maps):
        raise ValueError("maps do not have identical resolutions")
    min_x = min(float(item["origin_x"]) for item in maps)
    min_y = min(float(item["origin_y"]) for item in maps)
    max_x = max(float(item["origin_x"]) + item["width"] * resolution for item in maps)
    max_y = max(float(item["origin_y"]) + item["height"] * resolution for item in maps)
    origin_x = math.floor(min_x / resolution) * resolution
    origin_y = math.floor(min_y / resolution) * resolution
    width = max(1, int(math.ceil((max_x - origin_x) / resolution - 1e-9)))
    height = max(1, int(math.ceil((max_y - origin_y) / resolution - 1e-9)))
    return {"resolution": resolution, "origin_x": origin_x, "origin_y": origin_y,
            "width": width, "height": height}


def render_map(map_data, canvas, output_path):
    scale = 1.0 / canvas["resolution"]
    image = Image.new("L", (canvas["width"], canvas["height"]), 128)
    pixels = image.load()
    source_res = float(map_data["resolution"])
    offset_x = int(round((float(map_data["origin_x"]) - canvas["origin_x"]) / source_res))
    offset_y = int(round((float(map_data["origin_y"]) - canvas["origin_y"]) / source_res))
    width = int(map_data["width"])
    height = int(map_data["height"])
    data = map_data["data"]
    for y in range(height):
        destination_y = canvas["height"] - 1 - (offset_y + y)
        if not 0 <= destination_y < canvas["height"]:
            continue
        for x in range(width):
            destination_x = offset_x + x
            if not 0 <= destination_x < canvas["width"]:
                continue
            value = int(data[y * width + x])
            pixels[destination_x, destination_y] = 128 if value < 0 else (0 if value > 50 else 255)
    image.save(output_path)
    return output_path


def render_side_by_side(off, on, output_path):
    canvas = common_canvas([off, on])
    paths = []
    for item in (off, on):
        temporary = output_path + ".tmp_" + str(len(paths)) + ".png"
        render_map(item, canvas, temporary)
        paths.append(temporary)
    left = Image.open(paths[0]).convert("RGB")
    right = Image.open(paths[1]).convert("RGB")
    title_height = 28
    result = Image.new("RGB", (left.width * 2, left.height + title_height), "#30343b")
    result.paste(left, (0, title_height))
    result.paste(right, (left.width, title_height))
    draw = ImageDraw.Draw(result)
    draw.text((8, 7), "Scan Matching OFF", fill="white")
    draw.text((left.width + 8, 7), "Scan Matching ON", fill="white")
    result.save(output_path)
    for path in paths:
        try:
            os.unlink(path)
        except OSError:
            pass
    return output_path
