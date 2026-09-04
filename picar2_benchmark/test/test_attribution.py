from picar2_benchmark.attribution import classify, cusp_evidence


def test_collision_outranks_everything():
    c, _ = classify('TIMEOUT', {'min_clearance_m': -0.01}, 3.0, 90.0)
    assert c == 'COLLISION'


def test_degraded_trial_is_never_a_nav_result():
    c, _ = classify('SIM_DEGRADED', {}, 0.0, 0.0)
    assert c == 'SIM_DEGRADED'


def test_goal_checker_is_not_a_timeout():
    """Position reached, heading not. An Ackermann car cannot spin to fix it,
    so calling this a timeout hides a real and separately fixable outcome."""
    c, _ = classify('TIMEOUT', {'plans_received': 3}, 0.10, 90.0)
    assert c == 'GOAL_CHECKER'


def test_no_plan_is_the_planner():
    c, _ = classify('TIMEOUT', {'plans_received': 0}, 4.0, 10.0)
    assert c == 'PLANNER_NO_PATH'


def test_blocked_controller_distinguished_from_stuck_execution():
    blocked = classify('TIMEOUT', {'plans_received': 2, 'cmd_zero_pct': 85.0}, 4.0, 10.0)[0]
    stuck = classify('TIMEOUT', {'plans_received': 2, 'cmd_zero_pct': 5.0,
                                 'longest_stall_s': 30.0}, 4.0, 10.0)[0]
    assert blocked == 'CONTROLLER_BLOCKED'
    assert stuck == 'EXECUTION_STUCK'


def test_cusp_evidence_detects_pruned_reverse_leg():
    msg = cusp_evidence({'max_plan_cusps': 2, 'max_pruned_cusps': 0})
    assert msg is not None and '4757' in msg
    assert 'no reverse segments' in cusp_evidence(
        {'max_plan_cusps': 0, 'max_pruned_cusps': 0})


def test_heavy_oscillation_is_livelock_not_execution_stuck():
    """Measured on dead_end_reverse: 141 reversals, 109 controller aborts,
    commands non-zero 99% of the time. Calling that 'stuck' is wrong — the
    robot is moving constantly, just never committing to a direction."""
    c, why = classify('TIMEOUT', {
        'plans_received': 108, 'direction_reversals': 141,
        'follow_path_failures': 12, 'compute_path_failures': 0,
        'cmd_zero_pct': 0.8, 'longest_stall_s': 60.0}, 2.371, 4.3)
    assert c == 'LIVELOCK'
    assert '141' in why and '12' in why


def test_preemptions_are_not_failures():
    """follow_path goals are preempted once per replan cycle and report as
    ABORTED. Counting those as failures made 108 replans look like 108
    controller failures."""
    c, _ = classify('TIMEOUT', {
        'plans_received': 108, 'controller_goals_preempted': 109,
        'compute_path_failures': 0, 'follow_path_failures': 0,
        'direction_reversals': 2, 'cmd_zero_pct': 0.8,
        'longest_stall_s': 60.0}, 2.4, 4.3)
    assert c == 'EXECUTION_STUCK'      # not CONTROLLER_ABORT
