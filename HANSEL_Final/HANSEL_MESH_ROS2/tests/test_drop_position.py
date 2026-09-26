import math
import pytest
import _paths  # noqa: F401
from hansel_localization.core import PlanarOdometry


def test_drop_at_start_extrapolates_unit_offset_instead_of_stacking_at_head():
    odom = PlanarOdometry(1 / math.pi, 1)
    odom.update(0, 0, 0, 0)
    sample = odom.sample_behind(.4)
    assert sample.provisional and sample.pose.x_m == pytest.approx(-.4)


def test_event_interpolation_and_short_final_segment():
    odom = PlanarOdometry(1 / math.pi, 10, right_encoder_sign=1, history_min_step_m=.15)
    for i in range(21):
        odom.update(i * .1, i, i, 0)
    assert odom.sample_behind(.02).pose.x_m == pytest.approx(1.98)
    assert odom.sample_behind(.4, .95).pose.x_m == pytest.approx(.55)
    assert odom.sample_behind(.4, -1) is None
    assert odom.sample_behind(.4, 3) is None


def test_no_interpolation_across_missing_odometry_interval():
    odom = PlanarOdometry(1 / math.pi, 1)
    odom.update(0, 1, 1, 0)
    odom.update(.1, 1, 1, 0)
    odom.update(2, 1, 1, 0)
    odom.update(2.1, 1, 1, 0)
    assert odom.sample_behind(.2, 1) is None
