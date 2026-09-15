"""Pure live-footprint clearing for the map-to-Nav2 fusion boundary."""
import copy
import math


def _cell_center(grid, column, row):
    """Return the centre of a grid cell (the fusion grid is axis aligned)."""
    return (
        grid.info.origin.position.x + (column + 0.5) * grid.info.resolution,
        grid.info.origin.position.y + (row + 0.5) * grid.info.resolution,
    )


def footprint_cell_indices(grid, footprint, *, uncertainty_cells=1):
    """Return the bounded occupancy cells covered by one live footprint."""
    if uncertainty_cells < 0 or uncertainty_cells > 1:
        raise ValueError('uncertainty_cells must be 0 or 1')
    resolution = float(grid.info.resolution)
    if resolution <= 0.0:
        return set()
    radius = float(footprint['radius_m'])
    if radius <= 0.0:
        raise ValueError('footprint radius must be positive')
    clear_radius = radius + uncertainty_cells * resolution
    min_col = max(0, int(math.floor(
        (footprint['x'] - clear_radius - grid.info.origin.position.x)
        / resolution)))
    max_col = min(grid.info.width - 1, int(math.floor(
        (footprint['x'] + clear_radius - grid.info.origin.position.x)
        / resolution)))
    min_row = max(0, int(math.floor(
        (footprint['y'] - clear_radius - grid.info.origin.position.y)
        / resolution)))
    max_row = min(grid.info.height - 1, int(math.floor(
        (footprint['y'] + clear_radius - grid.info.origin.position.y)
        / resolution)))
    cells = set()
    for row in range(min_row, max_row + 1):
        for column in range(min_col, max_col + 1):
            x, y = _cell_center(grid, column, row)
            if math.hypot(x - footprint['x'], y - footprint['y']) <= clear_radius:
                cells.add(row * grid.info.width + column)
    return cells


def swept_footprint_cell_indices(grid, points, radius_m):
    """Rasterize a continuous circular footprint along a polyline.

    The sampling interval is half a grid cell, so consecutive samples cannot
    leave a hole in the swept footprint.  No uncertainty halo is added: the
    radius is the physical robot footprint supplied by the caller.
    """
    radius = float(radius_m)
    if radius <= 0.0:
        raise ValueError('radius_m must be positive')
    points = [(float(x), float(y)) for x, y in points]
    if not points:
        return set()
    cells = set()
    for start, end in zip(points, points[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, int(math.ceil(distance / max(1e-6, grid.info.resolution * 0.5))))
        for step in range(steps + 1):
            fraction = step / steps
            x = start[0] + fraction * (end[0] - start[0])
            y = start[1] + fraction * (end[1] - start[1])
            cells.update(footprint_cell_indices(
                grid, {'x': x, 'y': y, 'radius_m': radius},
                uncertainty_cells=0))
    if len(points) == 1:
        x, y = points[0]
        cells.update(footprint_cell_indices(
            grid, {'x': x, 'y': y, 'radius_m': radius},
            uncertainty_cells=0))
    return cells


def apply_incremental_patch(base_data, output_data, previous_cells, new_cells):
    """Restore and clear only cells touched by moving live footprints."""
    if len(base_data) != len(output_data):
        raise ValueError('base and output data lengths must match')
    modified = 0
    affected = set(previous_cells) | set(new_cells)
    for index in affected:
        desired = base_data[index]
        if index in new_cells and desired >= 0:
            desired = 0
        if output_data[index] != desired:
            output_data[index] = desired
            modified += 1
    return modified


def clear_live_footprints(
    grid, footprints, *, uncertainty_cells=1, max_pose_age_s=0.5,
):
    """Clear occupied evidence strictly beneath fresh physical footprints.

    ``footprints`` contains dictionaries with ``x``, ``y``, ``radius_m`` and
    ``pose_age_s``. Occupied cells beneath a fresh physical footprint become
    explicit free space (0); unknown cells remain unknown and stale poses
    clear no cells. The optional margin is bounded to one occupancy-grid cell.
    """
    if uncertainty_cells < 0 or uncertainty_cells > 1:
        raise ValueError('uncertainty_cells must be 0 or 1')
    result = copy.copy(grid)
    result.data = list(grid.data)
    resolution = float(grid.info.resolution)
    if resolution <= 0.0:
        return result, {'cleared_cell_count': 0, 'stale_pose_count': len(footprints)}
    total = 0
    stale = 0
    cleared_by_role = {'own': 0, 'peer': 0}
    stale_by_role = {'own': 0, 'peer': 0}
    details = []
    for footprint in footprints:
        age = footprint.get('pose_age_s')
        role = footprint.get('role', 'peer')
        if role not in cleared_by_role:
            role = 'peer'
        if age is None or age < 0.0 or age > max_pose_age_s:
            stale += 1
            stale_by_role[role] += 1
            details.append({**footprint, 'cleared_cell_count': 0,
                            'stale': True})
            continue
        # The one-cell margin is a bounded occupancy-grid uncertainty margin,
        # never Nav2's inflation radius.
        clear_radius = (float(footprint['radius_m'])
                        + uncertainty_cells * resolution)
        cells = footprint_cell_indices(
            grid, footprint, uncertainty_cells=uncertainty_cells)
        cleared = 0
        for index in cells:
            if result.data[index] >= 0:
                result.data[index] = 0
                cleared += 1
        total += cleared
        cleared_by_role[role] += cleared
        details.append({**footprint, 'cleared_cell_count': cleared,
                        'clear_radius_m': clear_radius, 'stale': False})
    return result, {
        'cleared_cell_count': total,
        'stale_pose_count': stale,
        'cleared_by_role': cleared_by_role,
        'stale_by_role': stale_by_role,
        'footprints': details,
    }


def sanitize_shared_map(grid, footprints, *, uncertainty_cells=1,
                        max_pose_age_s=0.5):
    """Named boundary API used by source-aware fusion and focused tests."""
    return clear_live_footprints(
        grid, footprints, uncertainty_cells=uncertainty_cells,
        max_pose_age_s=max_pose_age_s)
