"""Flight accuracy eval: how far the drone strays from its setpoints, and why (sim or real drone).

Reads rosbags of flight_path runs (docs/design/flight_accuracy.md) and compares three signals:
    setpoint   /fmu/in/trajectory_setpoint         where Commander told PX4 to be
    estimate   /fmu/out/vehicle_local_position     where PX4 thinks it is
    truth      /mocap/vespr/pose                   where it really is (mocap, or mocap_sim)
    total error = truth - setpoint = estimation error (truth - estimate)
                                   + control error (estimate - setpoint)
Everything is put in PX4's local frame (north-east-down, from the takeoff spot). Mocap is lined up
with it once, from the drone sitting still before it arms. Signals are timed by when the bag
received them, and resampled onto the mocap times.

Writes flight_accuracy.json (every number), paths.png (all runs) and run_NN.png (one per run) to
--out, and prints the summary tables. With --before <bags>, a set flown before a change, it also
writes before_after.png and before_after.md: both sets side by side, to see what the change did.
    ros2 run perch_eval flight_accuracy ~/vespr_recordings/flight_accuracy/<date>/run_?? --out <dir>
    ros2 run perch_eval flight_accuracy <after>/run_?? --before <before>/run_?? --out <dir>
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

SETPOINT = '/fmu/in/trajectory_setpoint'
ESTIMATE = '/fmu/out/vehicle_local_position'
STATUS = '/fmu/out/vehicle_status_v1'
TRUTH = '/mocap/vespr/pose'
TOPICS = (SETPOINT, ESTIMATE, STATUS, TRUTH)

NAV_STATE_OFFBOARD = 14  # px4_msgs VehicleStatus.NAVIGATION_STATE_OFFBOARD
ARMING_STATE_ARMED = 2   # px4_msgs VehicleStatus.ARMING_STATE_ARMED
REST_WINDOW = 3.0        # s before arming used to line mocap up with PX4's frame
STILL = 1e-6             # setpoint speed (m/s, rad/s) below which it isn't moving
SOURCES = ('total', 'estimation', 'control')

# Chart colours: the reference categorical palette, slots in order (dataviz skill)
INK, INK_2, GRID, SURFACE = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
ACTUAL, ESTIMATED, GAP = '#2a78d6', '#eb6834', '#1baf7a'


# ---- reading ----

def read_bag(path):
    """{topic: (receive times in s, [messages])} for the eval's topics."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
             if t.name in TOPICS}
    missing = set(TOPICS) - set(types)
    if missing:
        raise ValueError(f'{path}: no {", ".join(sorted(missing))} in the bag')
    out = {topic: ([], []) for topic in TOPICS}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in types:
            out[topic][0].append(stamp * 1e-9)
            out[topic][1].append(deserialize_message(data, types[topic]))
    return out


def signals(raw):
    """The bag's messages as arrays. Positions NED in m, angles in rad."""
    t, msgs = raw[SETPOINT]
    setpoint = dict(t=np.array(t),
                    position=np.array([m.position for m in msgs], dtype=float),
                    velocity=np.array([m.velocity for m in msgs], dtype=float),
                    yaw=np.array([m.yaw for m in msgs], dtype=float),
                    yawspeed=np.array([m.yawspeed for m in msgs], dtype=float))
    t, msgs = raw[ESTIMATE]
    estimate = dict(t=np.array(t),
                    position=np.array([(m.x, m.y, m.z) for m in msgs], dtype=float),
                    yaw=np.array([m.heading for m in msgs], dtype=float))
    t, msgs = raw[STATUS]
    status = dict(t=np.array(t),
                  offboard=np.array([m.nav_state == NAV_STATE_OFFBOARD for m in msgs]),
                  armed=np.array([m.arming_state == ARMING_STATE_ARMED for m in msgs]))
    t, msgs = raw[TRUTH]
    truth = dict(t=np.array(t),
                 position=np.array([(m.pose.position.x, m.pose.position.y, m.pose.position.z)
                                    for m in msgs], dtype=float),
                 yaw=np.array([gz_yaw(m.pose.orientation) for m in msgs], dtype=float))
    return setpoint, estimate, status, truth


def gz_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# ---- maths ----

def wrap(angle):
    return np.remainder(np.asarray(angle) + np.pi, 2 * np.pi) - np.pi


def held(t_from, values, t_to):
    """values sampled at t_from, held until the next sample, read at t_to."""
    i = np.clip(np.searchsorted(t_from, t_to, side='right') - 1, 0, len(t_from) - 1)
    return values[i]


def interp(t_from, values, t_to):
    """Linear interpolation of each column of values onto t_to."""
    values = np.asarray(values, dtype=float)
    if values.ndim == 1:
        return np.interp(t_to, t_from, values)
    return np.column_stack([np.interp(t_to, t_from, values[:, k]) for k in range(values.shape[1])])


def interp_angle(t_from, angle, t_to):
    return wrap(np.interp(t_to, t_from, np.unwrap(angle)))


def align(truth_rest, yaw_rest, estimate_rest, est_yaw_rest):
    """(yaw offset, offset) taking mocap to PX4's frame, from samples of the drone sitting still.

    Mocap is z up; turning it 180 deg about x gives a z-down frame, where headings are -yaw.
    Then a turn about down and an offset line it up with PX4's NED frame.
    """
    yaw_offset = float(np.angle(np.mean(np.exp(1j * (np.asarray(est_yaw_rest)
                                                     + np.asarray(yaw_rest))))))
    turned = to_ned(np.mean(truth_rest, axis=0)[None, :], yaw_offset, np.zeros(3))[0]
    return yaw_offset, np.mean(estimate_rest, axis=0) - turned


def to_ned(position, yaw_offset, offset):
    x, y, z = np.asarray(position, dtype=float).T
    c, s = math.cos(yaw_offset), math.sin(yaw_offset)
    return np.column_stack([c * x + s * y, s * x - c * y, -z]) + offset


def yaw_to_ned(yaw, yaw_offset):
    return wrap(yaw_offset - np.asarray(yaw))


def classify(velocity, yawspeed):
    """'moving', 'turning' or 'holding' for each setpoint sample."""
    moving = np.linalg.norm(velocity, axis=1) > STILL
    turning = ~moving & (np.abs(yawspeed) > STILL)
    return np.where(moving, 'moving', np.where(turning, 'turning', 'holding'))


def components(error, velocity, kind):
    """The parts of a position error (NED, m), by what the setpoint was doing.

    moving: along (+ ahead of the setpoint), off_line (distance from the line of travel).
    holding and turning: horizontal (distance off the target, seen from above).
    All: altitude (+ above).
    """
    out = {'altitude': -error[:, 2]}
    moving = kind == 'moving'
    if moving.any():
        e, v = error[moving], velocity[moving]
        u = v / np.linalg.norm(v, axis=1, keepdims=True)
        along = np.sum(e * u, axis=1)
        out['along'] = (moving, along)
        out['off_line'] = (moving, np.linalg.norm(e - along[:, None] * u, axis=1))
    still = ~moving
    if still.any():
        out['horizontal'] = (still, np.linalg.norm(error[still, :2], axis=1))
    return out


def stats(values):
    v = np.asarray(values, dtype=float)
    if v.size == 0:
        return None
    return {'n': int(v.size), 'mean': float(np.mean(v)), 'rms': float(np.sqrt(np.mean(v * v))),
            'p95_abs': float(np.percentile(np.abs(v), 95)), 'max_abs': float(np.max(np.abs(v)))}


def runs_of(kind):
    """[(start, end exclusive)] of each stretch of equal values."""
    edges = np.flatnonzero(kind[1:] != kind[:-1]) + 1
    starts = np.concatenate([[0], edges])
    ends = np.concatenate([edges, [len(kind)]])
    return list(zip(starts.tolist(), ends.tolist()))


def describe(delta, turn):
    if abs(turn) > 1e-3:
        return f'turn {math.degrees(turn):+.0f}'
    words = (('north', 'south'), ('east', 'west'), ('down', 'up'))
    parts = [f'{words[k][0] if d > 0 else words[k][1]} {abs(d):.1f} m'
             for k, d in enumerate(delta) if abs(d) > 0.05]
    return ', '.join(parts) or 'no move'


def settling(t, position, yaw, target, target_yaw, previous, previous_yaw, tolerance):
    """How one signal (truth or PX4's estimate) settles over one hold on the setpoint."""
    error = position - target
    distance = np.linalg.norm(error, axis=1)
    above = distance > tolerance
    if not above.any():
        settle_time = 0.0
    elif above[-1]:
        settle_time = None  # still outside the tolerance when the hold ended
    else:
        settle_time = float(t[np.flatnonzero(above)[-1] + 1] - t[0])
    delta = target - previous
    turn = float(wrap(target_yaw - previous_yaw))
    overshoot = overshoot_deg = None
    if np.linalg.norm(delta) > 1e-3:
        overshoot = max(0.0, float(np.max(error @ (delta / np.linalg.norm(delta)))))
    if abs(turn) > 1e-3:
        past = math.copysign(1.0, turn) * wrap(yaw - target_yaw)
        overshoot_deg = max(0.0, math.degrees(float(np.max(past))))
    end = t >= t[-1] - 0.5
    second_half = t >= (t[0] + t[-1]) / 2
    rest = position[second_half]
    return {
        'settle_time': settle_time,
        'overshoot': overshoot,
        'overshoot_deg': overshoot_deg,
        'final_error': float(np.mean(distance[end])),
        'wobble': float(np.sqrt(np.mean(np.sum((rest - rest.mean(axis=0)) ** 2, axis=1)))),
    }


# ---- one run ----

def evaluate(setpoint, estimate, status, truth, tolerance):
    """Errors and settling for one run, plus the resampled series for the plots."""
    armed_at = status['t'][np.flatnonzero(status['armed'])]
    if armed_at.size == 0:
        raise ValueError('PX4 never armed')
    first_arm = armed_at[0]

    def before_arm(t):
        return (t < first_arm) & (t >= first_arm - REST_WINDOW)
    rest_truth, rest_est = before_arm(truth['t']), before_arm(estimate['t'])
    if not rest_truth.any() or not rest_est.any():
        raise ValueError(f'no mocap or PX4 estimate in the {REST_WINDOW:.0f} s before arming')
    # Pair each resting mocap sample with PX4's heading at that moment
    rest_t = truth['t'][rest_truth]
    yaw_offset, offset = align(truth['position'][rest_truth], truth['yaw'][rest_truth],
                               estimate['position'][rest_est],
                               interp_angle(estimate['t'], estimate['yaw'], rest_t))

    # The path: from the first hold after the takeoff climb to the last setpoint in offboard
    in_flight = held(status['t'], status['offboard'] & status['armed'], setpoint['t'])
    kind = classify(setpoint['velocity'], setpoint['yawspeed'])
    climbing = np.flatnonzero(in_flight & (kind == 'moving'))
    if climbing.size == 0:
        raise ValueError('no takeoff climb in offboard')
    after_climb = np.flatnonzero(in_flight & (kind == 'holding')
                                 & (np.arange(len(kind)) > climbing[0]))
    if after_climb.size == 0:
        raise ValueError('no hold after the takeoff climb')
    flying = np.flatnonzero(in_flight)
    start, end = setpoint['t'][after_climb[0]], setpoint['t'][flying[-1]]
    # The climb and the takeoff hold are in the plots and the settling, not in the error stats:
    # the drone is still climbing from the ground well into the takeoff hold
    first_step = np.flatnonzero(in_flight & (kind != 'holding')
                                & (np.arange(len(kind)) > after_climb[0]))
    if first_step.size == 0:
        raise ValueError('no step after the takeoff hold')
    stats_from = setpoint['t'][first_step[0]]
    climb_start = setpoint['t'][climbing[0]]

    t_all = truth['t']
    window = (t_all >= climb_start) & (t_all <= end)
    t = t_all[window]
    sp_position = interp(setpoint['t'], setpoint['position'], t)
    sp_velocity = held(setpoint['t'], setpoint['velocity'], t)
    sp_yawspeed = held(setpoint['t'], setpoint['yawspeed'], t)
    sp_yaw = interp_angle(setpoint['t'], setpoint['yaw'], t)
    est_position = interp(estimate['t'], estimate['position'], t)
    est_yaw = interp_angle(estimate['t'], estimate['yaw'], t)
    true_position = to_ned(truth['position'][window], yaw_offset, offset)
    true_yaw = yaw_to_ned(truth['yaw'][window], yaw_offset)
    kind_t = classify(sp_velocity, sp_yawspeed)
    on_path = t >= stats_from

    errors = {
        'total': (true_position - sp_position, wrap(true_yaw - sp_yaw)),
        'estimation': (true_position - est_position, wrap(true_yaw - est_yaw)),
        'control': (est_position - sp_position, wrap(est_yaw - sp_yaw)),
    }
    samples = {}
    for source, (error, yaw_error) in errors.items():
        parts = components(error[on_path], sp_velocity[on_path], kind_t[on_path])
        samples[source] = {}
        for name in ('moving', 'turning', 'holding'):
            mask = kind_t[on_path] == name
            if not mask.any():
                continue
            out = {'altitude': parts['altitude'][mask],
                   'heading_deg': np.degrees(yaw_error[on_path][mask])}
            for key in ('along', 'off_line', 'horizontal'):
                if key in parts:
                    sel, values = parts[key]
                    keep = mask[sel]
                    if keep.any():
                        out[key] = values[keep]
            samples[source][name] = out

    holds = []
    spans = runs_of(kind_t)
    for a, b in spans:
        if kind_t[a] != 'holding' or a == 0 or b - a < 2:
            continue
        p = next(s for s, e in spans if e == a)  # the step before this hold
        p = max(p - 1, 0)                          # its last sample before it started
        ts = t[a:b] - start
        # Targets as sent, not interpolated: [this hold's, the previous hold's]
        positions = held(setpoint['t'], setpoint['position'], t[[a, p]])
        yaws = held(setpoint['t'], setpoint['yaw'], t[[a, p]])
        target = (positions[0], yaws[0], positions[1], yaws[1], tolerance)
        holds.append({
            'after': describe(positions[0] - positions[1], float(wrap(yaws[0] - yaws[1]))),
            'start': float(ts[0]),
            'duration': float(ts[-1] - ts[0]),
            'total': settling(ts, true_position[a:b], true_yaw[a:b], *target),
            'control': settling(ts, est_position[a:b], est_yaw[a:b], *target),
        })

    return {
        'alignment': {'yaw_offset_deg': math.degrees(yaw_offset), 'offset': offset.tolist()},
        'duration': float(end - start),
        'samples': samples,
        'holds': holds,
        'series': {'t': t - start, 'setpoint': sp_position, 'estimate': est_position,
                   'truth': true_position, 'errors': {k: v[0] for k, v in errors.items()}},
    }


# ---- many runs ----

def summarize(results):
    """Stats over every run's samples pooled, and each hold over the runs."""
    pooled = {}
    for source in SOURCES:
        pooled[source] = {}
        for kind in ('moving', 'turning', 'holding'):
            keys = sorted({k for r in results for k in r['samples'][source].get(kind, {})})
            if keys:
                pooled[source][kind] = {
                    k: stats(np.concatenate([r['samples'][source][kind][k] for r in results
                                             if k in r['samples'][source].get(kind, {})]))
                    for k in keys}
    holds = []
    for i, hold in enumerate(results[0]['holds']):
        same = [r['holds'][i] for r in results if len(r['holds']) > i]
        summary = {'hold': i, 'after': hold['after'], 'runs': len(same)}
        for source in ('total', 'control'):
            settled = [h[source]['settle_time'] for h in same
                       if h[source]['settle_time'] is not None]
            part = {'not_settled': len(same) - len(settled), 'settle_time': stats(settled)}
            for key in ('overshoot', 'overshoot_deg', 'final_error', 'wobble'):
                values = [h[source][key] for h in same if h[source][key] is not None]
                part[key] = stats(values) if values else None
            summary[source] = part
        holds.append(summary)
    return {'pooled': pooled, 'holds': holds}


def cm(stat, key):
    return '' if stat is None else f'{100 * stat[key]:.1f}'


def report(summary, n_runs, tolerance):
    """The summary tables, in markdown."""
    lines = [f'Runs: {n_runs}. Errors in cm (heading in deg), pooled over every run.', '']
    for kind, keys in (('moving', ('along', 'off_line', 'altitude', 'heading_deg')),
                       ('turning', ('horizontal', 'altitude', 'heading_deg')),
                       ('holding', ('horizontal', 'altitude', 'heading_deg'))):
        lines += [f'**{kind.capitalize()}**', '',
                  '| Error | Part | mean | RMS | 95 % | max |', '|---|---|---|---|---|---|']
        for source in SOURCES:
            for key in keys:
                s = summary['pooled'][source].get(kind, {}).get(key)
                if s is None:
                    continue
                if key == 'heading_deg':
                    row = [f'{s[k]:.2f}' for k in ('mean', 'rms', 'p95_abs', 'max_abs')]
                else:
                    row = [cm(s, k) for k in ('mean', 'rms', 'p95_abs', 'max_abs')]
                lines.append(f'| {source} | {key} | ' + ' | '.join(row) + ' |')
        lines.append('')
    for source, what in (('total', 'the actual position (mocap)'),
                         ('control', "PX4's estimate")):
        lines += [f'**Settling: {what} vs the setpoint** (tolerance {100 * tolerance:.0f} cm). '
                  'Mean / max over the runs.', '',
                  '| # | After | settle time s | not settled | overshoot cm | final error cm '
                  '| wobble cm |', '|---|---|---|---|---|---|---|']
        for hold in summary['holds']:
            h = hold[source]
            st = h['settle_time']
            settle = '' if st is None else f'{st["mean"]:.2f} / {st["max_abs"]:.2f}'
            if h['overshoot_deg'] is not None:
                over = f'{h["overshoot_deg"]["mean"]:.1f} / {h["overshoot_deg"]["max_abs"]:.1f} deg'
            elif h['overshoot'] is not None:
                over = f'{cm(h["overshoot"], "mean")} / {cm(h["overshoot"], "max_abs")}'
            else:
                over = ''
            lines.append(
                f'| {hold["hold"]} | {hold["after"]} | {settle} '
                f'| {h["not_settled"]} of {hold["runs"]} | {over} '
                f'| {cm(h["final_error"], "mean")} / {cm(h["final_error"], "max_abs")} '
                f'| {cm(h["wobble"], "mean")} / {cm(h["wobble"], "max_abs")} |')
        lines.append('')
    return '\n'.join(lines)


# ---- plots ----

def style(ax, title=None):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(INK_2)
    ax.tick_params(colors=INK_2, labelsize=9)
    ax.xaxis.label.set_color(INK_2)
    ax.yaxis.label.set_color(INK_2)
    if title:
        ax.set_title(title, color=INK, fontsize=11, loc='left')


def top_view(ax, lines):
    """Map view: east to the right, north up, equal scale."""
    for position, kwargs in lines:
        ax.plot(position[:, 1], position[:, 0], **kwargs)
    ax.set_xlabel('east (m)')
    ax.set_ylabel('north (m)')
    ax.set_aspect('equal', adjustable='datalim')


def figure(n_panels, width=5.2):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, n_panels, figsize=(width * n_panels, 4.6),
                             gridspec_kw={'width_ratios': [1] + [1.4] * (n_panels - 1)})
    fig.patch.set_facecolor(SURFACE)
    return plt, fig, axes


def plot_all(results, out_png):
    plt, fig, (top, side) = figure(2)
    first = results[0]['series']
    preferred = dict(color=INK, linewidth=1.5, linestyle=(0, (4, 3)), label='preferred (setpoint)')
    actual = dict(color=ACTUAL, linewidth=1.0, alpha=0.6)
    top_view(top, [(r['series']['truth'], actual) for r in results]
             + [(first['setpoint'], preferred)])
    for r in results:
        side.plot(r['series']['t'], -r['series']['truth'][:, 2], **actual)
    side.plot(first['t'], -first['setpoint'][:, 2], **preferred)
    side.set_xlabel('time from the start of the path (s)')
    side.set_ylabel('altitude (m)')
    top.plot([], [], color=ACTUAL, linewidth=1.5, label=f'actual (mocap), {len(results)} runs')
    style(top, 'Flight paths from above')
    style(side, 'Altitude')
    top.legend(frameon=False, fontsize=9, labelcolor=INK, loc='best')
    fig.tight_layout()
    fig.savefig(out_png, dpi=130, facecolor=SURFACE)
    plt.close(fig)


def plot_run(result, title, out_png):
    plt, fig, (top, side, err) = figure(3, width=4.8)
    s = result['series']
    preferred = dict(color=INK, linewidth=1.5, linestyle=(0, (4, 3)), label='preferred (setpoint)')
    actual = dict(color=ACTUAL, linewidth=1.5, label='actual (mocap)')
    estimate = dict(color=ESTIMATED, linewidth=1.5, label='PX4 estimate')
    top_view(top, [(s['truth'], actual), (s['estimate'], estimate), (s['setpoint'], preferred)])
    for key, kwargs in (('truth', actual), ('estimate', estimate), ('setpoint', preferred)):
        side.plot(s['t'], -s[key][:, 2], **kwargs)
    side.set_xlabel('time from the start of the path (s)')
    side.set_ylabel('altitude (m)')
    for key, color, label in (('total', ACTUAL, 'total: actual - preferred'),
                              ('control', ESTIMATED, 'control: estimate - preferred'),
                              ('estimation', GAP, 'estimation: actual - estimate')):
        err.plot(s['t'], 100 * np.linalg.norm(s['errors'][key], axis=1), color=color,
                 linewidth=1.2, label=label)
    err.set_xlabel('time from the start of the path (s)')
    err.set_ylabel('error size (cm)')
    style(top, f'{title}: path from above')
    style(side, 'Altitude')
    style(err, 'Error size')
    top.legend(frameon=False, fontsize=8, labelcolor=INK, loc='best')
    err.legend(frameon=False, fontsize=8, labelcolor=INK, loc='upper left')
    fig.tight_layout()
    fig.savefig(out_png, dpi=130, facecolor=SURFACE)
    plt.close(fig)


def plot_before_after(sets, out_png):
    """sets: [(name, results)]. A column each: the paths from above (all runs), and how far
    PX4's estimate was from the actual position over time, the error a fix to the estimate
    should shrink."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, len(sets), figsize=(5.2 * len(sets), 9.2), squeeze=False,
                             gridspec_kw={'height_ratios': [1.3, 1]})
    fig.patch.set_facecolor(SURFACE)
    peak = max(np.max(np.linalg.norm(r['series']['errors']['estimation'][:, :2], axis=1))
               for _, results in sets for r in results)
    for col, (name, results) in enumerate(sets):
        top, bottom = axes[0, col], axes[1, col]
        preferred = dict(color=INK, linewidth=1.5, linestyle=(0, (4, 3)), label='preferred')
        top_view(top, [(results[0]['series']['setpoint'], preferred)])
        for i, r in enumerate(results):
            s = r['series']
            top.plot(s['estimate'][:, 1], s['estimate'][:, 0], color=ESTIMATED, linewidth=1.0,
                     alpha=0.7, label='PX4 estimate' if i == 0 else None)
            top.plot(s['truth'][:, 1], s['truth'][:, 0], color=ACTUAL, linewidth=1.0,
                     alpha=0.75, label='actual (mocap)' if i == 0 else None)
            bottom.plot(s['t'], 100 * np.linalg.norm(s['errors']['estimation'][:, :2], axis=1),
                        color=GAP, linewidth=1.0, alpha=0.8)
        style(top, f'{name}: {len(results)} runs')
        top.legend(frameon=False, fontsize=8, labelcolor=INK, loc='best')
        bottom.set_ylim(0, 105 * peak)
        bottom.set_xlabel('time from the start of the path (s)')
        bottom.set_ylabel('cm')
        style(bottom, 'PX4 estimate vs actual, horizontal')
    # The same scale in every column, so the paths compare by eye
    tops = axes[0]
    xs, ys = zip(*[(ax.get_xlim(), ax.get_ylim()) for ax in tops])
    for ax in tops:
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim(min(x[0] for x in xs), max(x[1] for x in xs))
        ax.set_ylim(min(y[0] for y in ys), max(y[1] for y in ys))
    fig.tight_layout()
    fig.savefig(out_png, dpi=110, facecolor=SURFACE)
    plt.close(fig)


def compare(sets):
    """The headline numbers of each set, in a markdown table. sets: [(name, results)]"""
    def pooled(summary, source, kind, key, field='mean'):
        s = summary['pooled'][source].get(kind, {}).get(key)
        return None if s is None else s[field]

    rows = [('PX4 estimate vs actual while holding, mean / max (cm)',
             lambda m: f'{100 * pooled(m, "estimation", "holding", "horizontal"):.1f} / '
                       f'{100 * pooled(m, "estimation", "holding", "horizontal", "max_abs"):.1f}'),
            ('Actual vs preferred while holding, mean / max (cm)',
             lambda m: f'{100 * pooled(m, "total", "holding", "horizontal"):.1f} / '
                       f'{100 * pooled(m, "total", "holding", "horizontal", "max_abs"):.1f}'),
            ('Actual off the line while moving, mean (cm)',
             lambda m: f'{100 * pooled(m, "total", "moving", "off_line"):.1f}'),
            ('Actual altitude while holding, mean (cm)',
             lambda m: f'{100 * pooled(m, "total", "holding", "altitude"):+.1f}'),
            ('Heading, actual vs PX4 estimate, mean / max (deg)',
             lambda m: f'{pooled(m, "estimation", "holding", "heading_deg"):+.1f} / '
                       f'{pooled(m, "estimation", "holding", "heading_deg", "max_abs"):.1f}'),
            ('Actual settled within tolerance after a step (takeoff not counted)',
             lambda m: '{} of {}'.format(*np.sum([(h['runs'] - h['total']['not_settled'],
                                                   h['runs']) for h in m['holds'][1:]], axis=0))),
            ('Actual distance from the target at the end of a hold, mean (cm)',
             lambda m: '{:.1f}'.format(100 * np.mean([h['total']['final_error']['mean']
                                                      for h in m['holds'][1:]])))]
    summaries = [(name, summarize(results)) for name, results in sets]
    lines = ['| | ' + ' | '.join(name for name, _ in summaries) + ' |',
             '|---|' + '---|' * len(summaries)]
    for label, value in rows:
        lines.append(f'| {label} | ' + ' | '.join(value(m) for _, m in summaries) + ' |')
    return '\n'.join(lines)


# ---- command line ----

def to_json(value):
    if isinstance(value, dict):
        return {k: to_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_json(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def evaluate_bags(bags, tolerance):
    """(results, skipped) for a set of runs."""
    results, failed = [], []
    for bag in bags:
        try:
            result = evaluate(*signals(read_bag(bag)), tolerance)
        except ValueError as e:
            print(f'{bag}: skipped, {e}')
            failed.append({'bag': str(bag), 'reason': str(e)})
            continue
        result['bag'] = str(bag)
        results.append(result)
    return results, failed


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('bags', nargs='+', type=Path, help='rosbag folders, one per run')
    parser.add_argument('--out', type=Path, required=True, help='folder for the JSON and plots')
    parser.add_argument('--tolerance', type=float, default=0.05,
                        help='distance from the setpoint that counts as settled, m')
    parser.add_argument('--before', nargs='+', type=Path, default=[],
                        help='rosbag folders of a set flown before a change, to compare with')
    parser.add_argument('--names', nargs=2, default=('Before', 'After'),
                        metavar=('BEFORE', 'AFTER'),
                        help='titles of the two sets in the comparison')
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)

    results, failed = evaluate_bags(args.bags, args.tolerance)
    if not results:
        raise SystemExit('no run could be evaluated')
    for i, result in enumerate(results, 1):
        plot_run(result, f'Run {i}', args.out / f'run_{i:02d}.png')
    plot_all(results, args.out / 'paths.png')
    summary = summarize(results)
    text = report(summary, len(results), args.tolerance)
    runs = [{'bag': r['bag'], 'alignment': r['alignment'], 'duration': r['duration'],
             'holds': r['holds'],
             'errors': {src: {kind: {k: stats(v) for k, v in parts.items()}
                              for kind, parts in r['samples'][src].items()}
                        for src in SOURCES}}
            for r in results]
    (args.out / 'flight_accuracy.json').write_text(json.dumps(to_json(
        {'tolerance': args.tolerance, 'runs': runs, 'skipped': failed, **summary}), indent=1))
    (args.out / 'summary.md').write_text(text + '\n')
    print(text)

    if args.before:
        before, _ = evaluate_bags(args.before, args.tolerance)
        if not before:
            raise SystemExit('no run of --before could be evaluated')
        sets = [(args.names[0], before), (args.names[1], results)]
        plot_before_after(sets, args.out / 'before_after.png')
        table = compare(sets)
        (args.out / 'before_after.md').write_text(table + '\n')
        print(table)
        print(f'before/after: {args.out / "before_after.png"}')
