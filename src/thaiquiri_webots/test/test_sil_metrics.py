"""Regression tests for SIL scoring: preserve scale and reject unmatched data."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from thaiquiri_webots.sil_metrics import (
    horizontal_speed_summary, odometry_tracking, summarize_samples,
    surface_error, trajectory_error)


def make_path():
    times = np.linspace(0, 10, 101)
    truth = np.column_stack([times, times*.3, np.sin(times)*.2, times*0+1,
                             np.zeros((len(times), 3)), np.ones(len(times))])
    return truth


def test_alignment_removes_origin_and_heading_only():
    truth = make_path()
    rotation = Rotation.from_euler('z', .8)
    translation = np.array([4, -3, 2])
    estimate = truth.copy()
    estimate[:,1:4] = rotation.inv().apply(truth[:,1:4]-translation)
    estimate[:,4:8] = rotation.inv().as_quat()
    result, _ = trajectory_error(truth, estimate)
    assert result['position_rmse_m'] < 1e-12


def test_scale_error_is_not_fitted_away():
    truth = make_path()
    estimate = truth.copy()
    estimate[:,1:3] *= 1.2
    result, _ = trajectory_error(truth, estimate)
    assert result['position_rmse_m'] > .3
    assert result['end_drift_m'] > .5


def test_drift_timestamp_does_not_imply_full_trial_coverage():
    truth = make_path()
    result, _ = trajectory_error(truth, truth[:51])
    assert result['end_drift_m'] == pytest.approx(0)
    assert result['end_drift_stamp_s'] == pytest.approx(5)
    assert result['end_drift_scope'] == 'last matched valid estimate'


def test_missing_and_unmatched_data_fail_closed():
    assert trajectory_error([], [])[0]['available'] is False
    truth = make_path()
    estimate = truth.copy()
    estimate[:,0] += 30
    assert trajectory_error(truth, estimate)[0]['available'] is False
    assert summarize_samples([np.nan])['rmse'] is None
    assert summarize_samples([-2, 2])['rmse'] == pytest.approx(2)


def test_empty_map_is_unavailable_not_zero_error():
    assert surface_error([], [[0, 0, 0]]) == {'available': False}
    assert surface_error([[np.nan, 0, 0]], [[0, 0, 0]]) == {'available': False}


def test_surface_error_keeps_metric_distance():
    reference = [[0, 0, 0], [1, 0, 0], [2, 0, 0]]
    assert surface_error([[0, 0, .2], [1, 0, .2]], reference)['p95_m'] == pytest.approx(.2)


def test_tracking_loss_cannot_pass_by_publishing_invalid_odometry():
    # Valid ICP poses stop at 30 s, while the trial/invalid messages run to 130 s.
    result = odometry_tracking(np.arange(20, 30.01, .1), 20, 130)
    assert result['valid_rate_hz'] < 1
    assert result['max_gap_s'] == pytest.approx(100)
    assert result['final_age_s'] == pytest.approx(100)


def test_tracking_recovery_still_records_the_outage():
    stamps = np.concatenate((np.arange(20, 30, .1), np.arange(32, 130, .1)))
    result = odometry_tracking(stamps, 20, 130)
    assert result['valid_rate_hz'] > 3
    assert result['final_age_s'] < .5
    assert result['max_gap_s'] > 2


def test_tracking_missing_data_and_startup_loss():
    result = odometry_tracking([], 20, 130)
    assert result['valid_rate_hz'] == 0
    assert result['max_gap_s'] == 110
    assert result['final_age_s'] is None
    assert result['last_valid_stamp_s'] is None
    result = odometry_tracking(np.arange(25, 130, .1), 20, 130)
    assert result['max_gap_s'] == 5


def test_tracking_ignores_duplicates_and_out_of_window_stamps():
    result = odometry_tracking([21, 20, 21, np.nan, np.inf, 19, 23], 20, 22)
    assert result['valid_samples'] == 2
    assert result['valid_rate_hz'] == 1
    assert result['max_gap_s'] == 1


@pytest.mark.parametrize('start,end', [(20, 20), (21, 20), (np.nan, 130)])
def test_tracking_requires_valid_window(start, end):
    with pytest.raises(ValueError):
        odometry_tracking([], start, end)


def test_horizontal_speed_is_measured_and_not_the_requested_limit():
    truth = np.array([[20, 0, 0], [20.1, .06, .08], [20.2, .12, .16]])
    result = horizontal_speed_summary(truth)
    assert result['count'] == 2
    assert result['max_abs'] == pytest.approx(1.)


def test_horizontal_speed_rejects_bad_time_and_missing_data():
    assert horizontal_speed_summary([])['max_abs'] is None
    assert horizontal_speed_summary([[20, 0, 0], [20, 1, 0]])['max_abs'] is None
    assert horizontal_speed_summary([[20, 0, 0], [19, 1, 0]])['max_abs'] is None
    assert horizontal_speed_summary([[19, 0, 0], [20, 1, 0]])['max_abs'] is None
