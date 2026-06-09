#!/usr/bin/env python3
"""Offline Active SLAM log analyzer for goal utility and ping-pong symptoms."""

from __future__ import annotations

import argparse
import math
import re
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


TIMESTAMP_RE = re.compile(r"\[(\d+(?:\.\d+)?)\]")
GOAL_RE = re.compile(
    r"Sending NavigateToPose goal:.*?x=(?P<x>[-0-9.]+) y=(?P<y>[-0-9.]+).*?"
    r"cluster_id=(?P<cluster>-?\d+) source=(?P<source>\S+) "
    r"distance=(?P<distance>[-0-9.]+) score=(?P<score>[-0-9.]+) cost=(?P<cost>\S+)"
)
SELECTION_RE = re.compile(
    r"Goal selection: mode=(?P<mode>\S+) efficient_utility=(?P<efficient>\S+).*?"
    r"local_candidates=(?P<local>\d+) utility_candidates=(?P<utility>\d+).*?"
    r"rejected_by_clearance=(?P<clearance>\d+) rejected_by_blacklist=(?P<blacklist>\d+).*?"
    r"selected_cluster_id=(?P<cluster>-?\d+) selected_source=(?P<source>\S+) "
    r"selected_distance=(?P<distance>[-0-9.]+) selected_score=(?P<score>[-0-9.]+) "
    r"selected_cost=(?P<cost>\S+) selected_world=\((?P<x>[-0-9.]+), (?P<y>[-0-9.]+)\).*?"
    r"information_gain=(?P<gain>[-0-9.]+)"
)
SELECTION_NONE_RE = re.compile(
    r"Goal selection: mode=(?P<mode>\S+) efficient_utility=(?P<efficient>\S+).*?"
    r"local_candidates=(?P<local>\d+) utility_candidates=(?P<utility>\d+).*?"
    r"selected=none skip_reason=(?P<skip>.*)$"
)
RUNTIME_RE = re.compile(
    r"Runtime: .*?frontier_cells=(?P<cells>\d+) frontier_clusters=(?P<clusters>\d+) "
    r"best_id=(?P<best_id>-?\d+) best_size=(?P<best_size>\d+).*?"
    r"best_centroid_world=\((?P<x>[-0-9.]+), (?P<y>[-0-9.]+)\) "
    r"best_distance=(?P<distance>[-0-9.]+)"
)
RESULT_RE = re.compile(r"NavigateToPose result: (?P<result>\S+) source=(?P<source>\S+)")
REJECT_RE = re.compile(
    r"Planner candidate rejected: .*?cluster_id=(?P<cluster>-?\d+).*?"
    r"reason=(?P<reason>\S+).*?max_cost_near_path=(?P<near>\d+)"
)


@dataclass
class GoalEvent:
    stamp: float
    source: str
    cluster_id: int
    x: float
    y: float
    distance: float
    score: float
    cost: Optional[float]
    information_gain: Optional[float] = None


@dataclass
class SelectionEvent:
    stamp: float
    mode: str
    efficient_utility: str
    source: Optional[str]
    cluster_id: Optional[int]
    x: Optional[float]
    y: Optional[float]
    distance: Optional[float]
    score: Optional[float]
    cost: Optional[float]
    information_gain: Optional[float]
    local_candidates: int
    utility_candidates: int
    rejected_by_clearance: int
    rejected_by_blacklist: int
    skip_reason: str = "none"


@dataclass
class RuntimeEvent:
    stamp: float
    frontier_cells: int
    frontier_clusters: int
    best_id: int
    best_size: int
    best_x: float
    best_y: float
    best_distance: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--output-md", required=True, type=Path)
    parser.add_argument("--late-n", type=int, default=15)
    parser.add_argument("--bucket-size", type=float, default=1.0)
    return parser.parse_args()


def stamp(line: str) -> float:
    matches = TIMESTAMP_RE.findall(line)
    return float(matches[-1]) if matches else 0.0


def as_float(text: str) -> Optional[float]:
    if text in ("unknown", "None", "nan"):
        return None
    try:
        return float(text)
    except ValueError:
        return None


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def bucket(x: float, y: float, size: float) -> tuple[int, int]:
    return (int(math.floor(x / size)), int(math.floor(y / size)))


def direction_reversals(goals: list[GoalEvent]) -> int:
    count = 0
    for first, second, third in zip(goals, goals[1:], goals[2:]):
        before = (second.x - first.x, second.y - first.y)
        after = (third.x - second.x, third.y - second.y)
        before_norm = math.hypot(*before)
        after_norm = math.hypot(*after)
        if before_norm <= 1e-9 or after_norm <= 1e-9:
            continue
        cosine = (before[0] * after[0] + before[1] * after[1]) / (before_norm * after_norm)
        if math.degrees(math.acos(max(-1.0, min(1.0, cosine)))) >= 120.0:
            count += 1
    return count


def parse_log(path: Path) -> tuple[list[GoalEvent], list[SelectionEvent], list[RuntimeEvent], Counter]:
    goals: list[GoalEvent] = []
    selections: list[SelectionEvent] = []
    runtimes: list[RuntimeEvent] = []
    counters: Counter = Counter()

    for line in path.read_text(errors="replace").splitlines():
        if match := GOAL_RE.search(line):
            goals.append(
                GoalEvent(
                    stamp=stamp(line),
                    source=match.group("source"),
                    cluster_id=int(match.group("cluster")),
                    x=float(match.group("x")),
                    y=float(match.group("y")),
                    distance=float(match.group("distance")),
                    score=float(match.group("score")),
                    cost=as_float(match.group("cost")),
                )
            )
            counters[f"goal_source:{match.group('source')}"] += 1

        if match := SELECTION_RE.search(line):
            event = SelectionEvent(
                stamp=stamp(line),
                mode=match.group("mode"),
                efficient_utility=match.group("efficient"),
                source=match.group("source"),
                cluster_id=int(match.group("cluster")),
                x=float(match.group("x")),
                y=float(match.group("y")),
                distance=float(match.group("distance")),
                score=float(match.group("score")),
                cost=as_float(match.group("cost")),
                information_gain=float(match.group("gain")),
                local_candidates=int(match.group("local")),
                utility_candidates=int(match.group("utility")),
                rejected_by_clearance=int(match.group("clearance")),
                rejected_by_blacklist=int(match.group("blacklist")),
            )
            selections.append(event)
            counters[f"selection_source:{event.source}"] += 1

        if match := SELECTION_NONE_RE.search(line):
            selections.append(
                SelectionEvent(
                    stamp=stamp(line),
                    mode=match.group("mode"),
                    efficient_utility=match.group("efficient"),
                    source=None,
                    cluster_id=None,
                    x=None,
                    y=None,
                    distance=None,
                    score=None,
                    cost=None,
                    information_gain=None,
                    local_candidates=int(match.group("local")),
                    utility_candidates=int(match.group("utility")),
                    rejected_by_clearance=0,
                    rejected_by_blacklist=0,
                    skip_reason=match.group("skip"),
                )
            )

        if match := RUNTIME_RE.search(line):
            runtimes.append(
                RuntimeEvent(
                    stamp=stamp(line),
                    frontier_cells=int(match.group("cells")),
                    frontier_clusters=int(match.group("clusters")),
                    best_id=int(match.group("best_id")),
                    best_size=int(match.group("best_size")),
                    best_x=float(match.group("x")),
                    best_y=float(match.group("y")),
                    best_distance=float(match.group("distance")),
                )
            )

        if match := RESULT_RE.search(line):
            result = match.group("result").lower()
            source = match.group("source")
            counters[f"nav_result:{result}"] += 1
            counters[f"nav_result_source:{result}:{source}"] += 1

        if match := REJECT_RE.search(line):
            reason = match.group("reason")
            near = int(match.group("near"))
            cluster = match.group("cluster")
            counters[f"planner_reject:{reason}"] += 1
            counters[f"planner_reject_cluster:{cluster}"] += 1
            counters[f"planner_reject_cluster_reason:{cluster}:{reason}"] += 1
            if near >= 70:
                counters["planner_reject_max_cost_near_path_70_plus"] += 1

        if "planner_reject_cache_added:" in line:
            counters["planner_reject_cache_added"] += 1
        if "planner_reject_cluster_blacklisted:" in line:
            counters["planner_reject_cluster_blacklisted"] += 1
        if "high_cost_escape:" in line and "selected_escape=" in line:
            counters["high_cost_escape"] += 1
        if "high_cost_escape_failed" in line:
            counters["high_cost_escape_failed"] += 1
        if "global_max_cost_near_robot" in line:
            counters["global_max_cost_near_robot_logs"] += 1
        if "relaxed_selected=True" in line:
            counters["relaxed_selected_true"] += 1

    for goal in goals:
        if goal.information_gain is not None:
            continue
        matching = min(
            (
                selection
                for selection in selections
                if selection.source == goal.source
                and selection.cluster_id == goal.cluster_id
                and selection.x is not None
                and abs(selection.x - goal.x) <= 0.05
                and selection.y is not None
                and abs(selection.y - goal.y) <= 0.05
            ),
            key=lambda selection: abs(selection.stamp - goal.stamp),
            default=None,
        )
        if matching is not None:
            goal.information_gain = matching.information_gain

    return goals, selections, runtimes, counters


def fmt(value: Optional[float], digits: int = 3) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}"


def average(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def render_report(path: Path, goals: list[GoalEvent], selections: list[SelectionEvent], runtimes: list[RuntimeEvent], counters: Counter, late_n: int, bucket_size: float) -> str:
    goal_steps = [distance((a.x, a.y), (b.x, b.y)) for a, b in zip(goals, goals[1:])]
    goal_buckets = [bucket(goal.x, goal.y, bucket_size) for goal in goals]
    repeated_cluster = sum(1 for a, b in zip(goals, goals[1:]) if a.cluster_id == b.cluster_id)
    repeated_bucket = sum(1 for a, b in zip(goal_buckets, goal_buckets[1:]) if a == b)
    alternating = sum(
        1
        for first, second, third in zip(goal_buckets, goal_buckets[1:], goal_buckets[2:])
        if first == third and first != second
    )
    same_x_corridor = sum(1 for a, b in zip(goals, goals[1:]) if abs(a.x - b.x) <= 0.25)
    same_y_corridor = sum(1 for a, b in zip(goals, goals[1:]) if abs(a.y - b.y) <= 0.25)
    late_goals = goals[-late_n:]
    late_buckets = goal_buckets[-late_n:]
    late_cluster_counts = Counter(goal.cluster_id for goal in late_goals)
    late_bucket_counts = Counter(late_buckets)
    late_alternating = sum(
        1
        for first, second, third in zip(late_buckets, late_buckets[1:], late_buckets[2:])
        if first == third and first != second
    )
    late_selections = [item for item in selections if item.source is not None][-late_n:]
    late_runtimes = runtimes[-late_n:]
    late_low_gain = sum(
        1 for item in late_selections if item.information_gain is not None and item.information_gain < 0.05
    )
    late_low_score = sum(1 for item in late_selections if item.score is not None and item.score < 0.2)
    late_edge_goals = sum(1 for goal in late_goals if abs(goal.x) >= 6.0 or abs(goal.y) >= 5.5)
    utility_active = any(
        item.mode == "safe_viewpoint"
        and item.efficient_utility == "False"
        and item.score is not None
        and abs(item.score) > 1e-6
        for item in selections
    )
    low_gain_pingpong = (
        len(late_goals) >= 4
        and (alternating >= 2 or same_x_corridor >= max(2, len(goals) // 5))
        and late_low_gain >= max(2, len(late_selections) // 2)
    )
    strong_repeated_pattern = (
        repeated_cluster >= 5
        or alternating >= 5
        or same_x_corridor >= 10
        or late_alternating >= 2
        or (late_cluster_counts and late_cluster_counts.most_common(1)[0][1] > 6)
        or (late_bucket_counts and late_bucket_counts.most_common(1)[0][1] > 6)
    )
    pingpong_likely = low_gain_pingpong or strong_repeated_pattern
    poor_late_quality = late_low_gain >= max(2, len(late_selections) // 2) or late_low_score >= max(2, len(late_selections) // 2)

    goal_sources = {
        key.split(":", 1)[1]: value
        for key, value in sorted(counters.items())
        if key.startswith("goal_source:")
    }
    nav_results = {
        key.split(":", 1)[1]: value
        for key, value in sorted(counters.items())
        if key.startswith("nav_result:")
    }

    lines = [
        "# Active SLAM Goal Ping-Pong Analysis",
        "",
        f"- Log: `{path}`",
        f"- Total `Sending NavigateToPose goal`: `{len(goals)}`",
        f"- Goal sources: `{goal_sources}`",
        f"- Nav results: `{nav_results}`",
        f"- Utility appears active: `{'yes' if utility_active else 'no'}`",
        f"- Likely ping-pong: `{'yes' if pingpong_likely else 'no'}`",
        f"- Likely late-stage frontier quality poor: `{'yes' if poor_late_quality else 'no'}`",
        "",
        "## Ping-Pong Metrics",
        "",
        f"- Average distance between consecutive goals: `{average(goal_steps):.3f} m`",
        f"- Repeated consecutive cluster_id count: `{repeated_cluster}`",
        f"- Repeated consecutive `{bucket_size:.1f} m` region bucket count: `{repeated_bucket}`",
        f"- Alternating A-B-A region count: `{alternating}`",
        f"- Late-stage A-B-A region count: `{late_alternating}`",
        f"- Direction reversal count: `{direction_reversals(goals)}`",
        f"- Same-x corridor steps (`|dx| <= 0.25 m`): `{same_x_corridor}`",
        f"- Same-y corridor steps (`|dy| <= 0.25 m`): `{same_y_corridor}`",
        f"- Late-stage edge goals in last {late_n}: `{late_edge_goals}`",
        f"- Most repeated cluster in last {late_n}: `{late_cluster_counts.most_common(1)[0] if late_cluster_counts else 'n/a'}`",
        f"- Most repeated region in last {late_n}: `{late_bucket_counts.most_common(1)[0] if late_bucket_counts else 'n/a'}`",
        "",
        "## Utility Evidence",
        "",
        f"- `mode=safe_viewpoint` selections: `{sum(1 for item in selections if item.mode == 'safe_viewpoint')}`",
        f"- `efficient_utility=False` selections: `{sum(1 for item in selections if item.efficient_utility == 'False')}`",
        f"- Nonzero selected scores: `{sum(1 for item in selections if item.score is not None and abs(item.score) > 1e-6)}`",
        f"- Information gain logged: `{sum(1 for item in selections if item.information_gain is not None)}`",
        f"- Total local_candidates logged: `{sum(item.local_candidates for item in selections)}`",
        f"- Total utility_candidates logged: `{sum(item.utility_candidates for item in selections)}`",
        f"- Total rejected_by_clearance logged: `{sum(item.rejected_by_clearance for item in selections)}`",
        f"- Total rejected_by_blacklist logged: `{sum(item.rejected_by_blacklist for item in selections)}`",
        "",
        "## Safety and Recovery Metrics",
        "",
        f"- `high_cost_escape`: `{counters['high_cost_escape']}`",
        f"- `high_cost_escape_failed`: `{counters['high_cost_escape_failed']}`",
        f"- `planner_reject_cache_added`: `{counters['planner_reject_cache_added']}`",
        f"- `planner_reject_cluster_blacklisted`: `{counters['planner_reject_cluster_blacklisted']}`",
        f"- `reason=path_clearance`: `{counters['planner_reject:path_clearance']}`",
        f"- `max_cost_near_path >= 70`: `{counters['planner_reject_max_cost_near_path_70_plus']}`",
        f"- `relaxed_selected=True`: `{counters['relaxed_selected_true']}`",
        "",
        "## Last Goal Timeline",
        "",
        "| Time | Source | Cluster | Goal x/y | Distance | Score | Cost | Information gain | Step from previous | Region |",
        "| ---: | --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | --- |",
    ]

    previous: Optional[GoalEvent] = None
    for goal in late_goals:
        step = None if previous is None else distance((previous.x, previous.y), (goal.x, goal.y))
        lines.append(
            f"| {goal.stamp:.3f} | `{goal.source}` | {goal.cluster_id} | "
            f"({goal.x:.2f}, {goal.y:.2f}) | {goal.distance:.2f} | {goal.score:.3f} | "
            f"{fmt(goal.cost, 0)} | {fmt(goal.information_gain)} | {fmt(step)} | "
            f"`{bucket(goal.x, goal.y, bucket_size)}` |"
        )
        previous = goal

    lines.extend([
        "",
        "## Last Runtime Summaries",
        "",
        "| Time | Frontier cells | Frontier clusters | Best id | Best size | Best centroid | Best distance |",
        "| ---: | ---: | ---: | ---: | ---: | --- | ---: |",
    ])
    for runtime in late_runtimes:
        lines.append(
            f"| {runtime.stamp:.3f} | {runtime.frontier_cells} | {runtime.frontier_clusters} | "
            f"{runtime.best_id} | {runtime.best_size} | ({runtime.best_x:.2f}, {runtime.best_y:.2f}) | "
            f"{runtime.best_distance:.2f} |"
        )

    lines.extend([
        "",
        "## Final Verdict",
        "",
        f"- Likely ping-pong: `{'yes' if pingpong_likely else 'no'}`",
        "- Likely cause: late-stage low-information boundary frontiers remain valid, while path-clearance rejects push selection to nearby alternatives in the same region.",
        f"- Utility appears active: `{'yes' if utility_active else 'no'}`",
        f"- Late-stage frontier quality appears poor: `{'yes' if poor_late_quality else 'no'}`",
        f"- High-cost/recovery contributed: `{'yes' if counters['high_cost_escape'] else 'no'}`",
    ])
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    goals, selections, runtimes, counters = parse_log(args.log)
    report = render_report(args.log, goals, selections, runtimes, counters, args.late_n, args.bucket_size)
    args.output_md.write_text(report)
    print(report, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
