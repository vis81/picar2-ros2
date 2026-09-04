"""Turn a recorded trial into a failure class.

The point is to answer the question the hardware runs could not: when the robot
does not reach the goal, was that the planner, the controller, or the robot?
Ordered by evidence precedence — the first matching rule wins.
"""
from __future__ import annotations


def classify(outcome: str, metrics: dict, final_xy_err: float, final_yaw_err_deg: float,
             xy_tol: float = 0.25, yaw_tol_deg: float = 28.6) -> tuple[str, str]:
    """Return (class, one-line reason)."""
    if outcome in ('SIM_DEGRADED', 'RUNNER_ERROR'):
        return outcome, 'trial discarded before measurement'

    if metrics.get('min_clearance_m', 1.0) <= 0.0:
        return 'COLLISION', f"footprint overlapped an obstacle " \
                            f"(min clearance {metrics.get('min_clearance_m')} m)"

    if outcome == 'SUCCEEDED':
        return 'SUCCEEDED', 'reached the goal'

    moved = metrics.get('stall_total_s', 0.0)
    plans = metrics.get('plans_received', 0)
    # BT FAILURE transitions, not action-status aborts: an abort is usually
    # just the previous goal being preempted by the next replan.
    p_ab = metrics.get('compute_path_failures', 0)
    c_ab = metrics.get('follow_path_failures', 0)

    # arrived in position but not in heading: an Ackermann car cannot spin to
    # fix yaw, so this is a distinct and fixable outcome, not a timeout
    if final_xy_err <= xy_tol and final_yaw_err_deg > yaw_tol_deg:
        return 'GOAL_CHECKER', (f'within {xy_tol} m of the goal but {final_yaw_err_deg:.0f} deg '
                                f'off heading; the car cannot rotate in place')

    if plans == 0:
        return 'PLANNER_NO_PATH', 'no global plan was ever produced'

    if p_ab and not c_ab:
        return 'PLANNER_NO_PATH', f'{p_ab} planner aborts, no controller abort'

    # a planner failure *after* the robot moved is usually controller-induced:
    # the controller drove it somewhere unplannable
    if p_ab and metrics.get('direction_reversals', 0) > 0:
        return 'PLANNER_FAILED_AFTER_PROGRESS', (
            f'{p_ab} planner aborts after the robot had moved; likely driven into '
            f'a pocket by the controller')

    if metrics.get('cmd_zero_pct', 0) > 60 and plans:
        return 'CONTROLLER_BLOCKED', (
            f"valid plan but commands were ~zero {metrics['cmd_zero_pct']}% of the time; "
            f'RPP self-blocking on its collision check')

    # Oscillation outranks 'stuck': a robot flipping direction many times per
    # minute is not failing to receive commands, it is failing to commit to any.
    if metrics.get('direction_reversals', 0) >= 20:
        return 'LIVELOCK', (
            f"{metrics['direction_reversals']} direction reversals and "
            f"{c_ab} controller aborts without reaching the goal "
            f'(ended {final_xy_err:.2f} m away, {p_ab} planner aborts)')

    if c_ab and not p_ab:
        return 'CONTROLLER_ABORT', (
            f'{c_ab} controller aborts with a valid plan and no planner failure')

    if metrics.get('cmd_zero_pct', 100) < 40 and metrics.get('longest_stall_s', 0) > 10:
        return 'EXECUTION_STUCK', (
            'commands were being issued but ground truth did not move')

    if metrics.get('direction_reversals', 0) >= 3:
        return 'LIVELOCK', (
            f"{metrics['direction_reversals']} direction reversals without reaching "
            f'the goal (ended {final_xy_err:.2f} m away)')

    return 'TIMEOUT_SLOW', f'made progress but ran out of time ({final_xy_err:.2f} m short)'


def cusp_evidence(metrics: dict) -> str | None:
    """Direct evidence for navigation2#4757, or None.

    RPP prunes the global plan to the nearest pose; on a Reeds-Shepp path a pose
    *after* a cusp is often nearer than the current segment, so the reverse leg
    is dropped. If the planner's plan contains cusps and the plan RPP actually
    received does not, that is the bug, observed rather than inferred.
    """
    planned = metrics.get('max_plan_cusps')
    pruned = metrics.get('max_pruned_cusps')
    if planned is None or pruned is None:
        return None
    if planned > 0 and pruned == 0:
        return (f'planner produced paths with up to {planned} cusp(s); the plan RPP '
                f'received had none — the reverse leg was pruned away (navigation2#4757)')
    if planned > 0:
        return f'plans contained up to {planned} cusp(s); RPP received up to {pruned}'
    return 'no reverse segments in any plan'
