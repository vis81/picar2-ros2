"""Summarising must never quietly average a degraded trial into a result."""
from picar2_benchmark.report import summarise


def _row(outcome, t=None, noise=1.0):
    r = {'outcome': outcome, 'sensor_noise': noise}
    if t is not None:
        r['time_s'] = t
    return r


def test_degraded_trials_are_excluded_not_averaged():
    rows = [_row('SUCCEEDED', 50.0), _row('SUCCEEDED', 52.0),
            _row('SIM_DEGRADED'), _row('RUNNER_ERROR')]
    s = summarise(rows)
    assert s['n_total'] == 4
    assert s['n_discarded'] == 2
    assert s['time_s']['n'] == 2
    assert s['time_s']['median'] == 51.0


def test_outcomes_are_categorical_not_a_success_rate():
    rows = [_row('SUCCEEDED', 1.0), _row('TIMEOUT', 2.0), _row('ABORTED', 3.0)]
    s = summarise(rows)
    assert s['outcomes'] == {'SUCCEEDED': 1, 'TIMEOUT': 1, 'ABORTED': 1}


def test_spread_is_reported_not_just_a_central_value():
    rows = [_row('SUCCEEDED', v) for v in (5.0, 28.0, 271.0)]
    s = summarise(rows)
    # the case that motivated this: a mean of 101 s describes no actual run
    assert s['time_s']['median'] == 28.0
    assert s['time_s']['min'] == 5.0
    assert s['time_s']['max'] == 271.0


def test_iqr_is_none_when_too_few_samples():
    """An IQR of 0.0 next to a wide range reads as 'perfectly repeatable' when
    it really means 'not enough data'. Report None instead."""
    s = summarise([_row('SUCCEEDED', v) for v in (78.8, 120.0, 120.0)])
    assert s['time_s']['iqr'] is None
    assert s['time_s']['min'] == 78.8 and s['time_s']['max'] == 120.0
