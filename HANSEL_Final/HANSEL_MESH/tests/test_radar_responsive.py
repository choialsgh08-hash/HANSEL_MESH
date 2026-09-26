import math
from unittest.mock import patch

import pytest

from common.radar_geometry import RadarAxes
from monitor.radar_scene import RadarSceneEstimator


def track(x, y, confidence=.5, updated=True):
    return dict(source="point", point_confirmed=True, _updated_by_point=updated,
                confidence=confidence, forward_m=x, lateral_m=y,
                distance_m=math.hypot(x, y), _received_at_ns=1_000_000_000)


def test_dense_confirmed_points_respond_before_single_track_threshold():
    for mode, expected in [("balanced", 0), ("responsive", 3)]:
        scene = RadarSceneEstimator(RadarAxes(), path_mode=mode)
        scene._tracks = [track(.20, .05), track(.23, .05), track(.26, .05)]
        scene._update_near_occupancy(1)
        assert len(scene._near_cells) == expected


@pytest.mark.parametrize("tracks", [
    [track(.2, .05)], [track(.2, .05)] * 5,
    [track(.2, .05, updated=False), track(.23, .05), track(.26, .05)],
    [track(.2, .05, .4), track(.23, .05, .4), track(.26, .05, .4)],
])
def test_weak_old_or_duplicate_points_do_not_become_walls(tracks):
    scene = RadarSceneEstimator(RadarAxes(), path_mode="responsive")
    scene._tracks = tracks
    scene._update_near_occupancy(1)
    assert not scene._near_cells


@pytest.mark.parametrize("side", [-1, 1])
def test_spi_opening_cannot_ignore_confirmed_near_points(side):
    scene = RadarSceneEstimator(RadarAxes(), path_mode="responsive")
    scene._calibration_status = "spi_raw"
    scene._tracks = [track(.20, .03 * side), track(.23, .03 * side), track(.26, .03 * side)]
    scene._update_near_occupancy(1)
    obstacles = scene._planning_obstacle_payload(1)
    straight = scene._plan_path([])
    assert scene._path_collision_segments(straight, obstacles)
    straight["instant_opening_scores"] = []
    with patch.object(scene, "_plan_reflectivity_opening", return_value=straight):
        scene._update_stable_path(1)
    assert scene._planned_path != straight
    # A route may turn or stop if no clearance exists, but cannot traverse the points.
    assert not scene._path_collision_segments(scene._planned_path, obstacles)
