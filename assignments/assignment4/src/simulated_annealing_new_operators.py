#!/usr/bin/env python3
"""INF273 Assignment 4 - Modified Simulated Annealing with three custom operators."""

import argparse
import csv
import math
import random
import sys
import time
from copy import deepcopy
from pathlib import Path
from statistics import mean
from typing import Any, Dict, List, Optional, Set, Tuple

ROOT_DIR = Path(__file__).resolve().parents[3]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from common.feasibility_check import SolutionFeasibility
from common.calculate_total_arrival_time import CalCulateTotalArrivalTime


Solution = Dict[str, Any]
Trip = Tuple[int, int, int]  # (launch_node, customer, reconvene_node)


def read_instance(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as file_handle:
        lines = [line.strip() for line in file_handle if line.strip() and not line.strip().startswith("#")]

    index = 0
    n_customers = int(lines[index])
    index += 1
    drone_limit = float(lines[index])
    index += 1

    size = n_customers + 1
    truck_time = [list(map(float, lines[index + i].split())) for i in range(size)]
    index += size
    drone_time = [list(map(float, lines[index + i].split())) for i in range(size)]

    return {
        "n": size,
        "n_customers": n_customers,
        "drone_limit": drone_limit,
        "T": truck_time,
        "D": drone_time,
        "name": Path(path).stem,
    }


def initial_solution(data: Dict[str, Any]) -> Solution:
    return {
        "route": [0] + list(range(1, data["n_customers"] + 1)) + [0],
        "trips": [],
    }


def _first_valid_positions(route: List[int], launch_node: int, reconvene_node: int) -> Optional[Tuple[int, int]]:
    """Return first (launch_idx, reconvene_idx) where launch appears before reconvene."""
    for i, node_i in enumerate(route):
        if node_i != launch_node:
            continue
        for j in range(i + 1, len(route)):
            if route[j] == reconvene_node:
                return i, j
    return None


def _assign_trips_to_drones(
    converted_trips: List[Tuple[int, int, int]],
    n_drones: int,
) -> Tuple[List[int], List[int], List[int]]:
    """Greedy interval assignment by launch index to produce part2/part3/part4 encoding."""
    drone_jobs: List[List[Tuple[int, int, int]]] = [[] for _ in range(n_drones)]
    drone_last_reconvene = [-1 for _ in range(n_drones)]

    for customer, launch_cell, reconvene_cell in sorted(converted_trips, key=lambda item: item[1]):
        chosen = None
        for d in range(n_drones):
            if launch_cell - 1 >= drone_last_reconvene[d]:
                chosen = d
                break

        # If all drones overlap, place on the earliest finishing drone.
        # Feasibility checks later decide if this candidate is usable.
        if chosen is None:
            chosen = min(range(n_drones), key=lambda idx: drone_last_reconvene[idx])

        drone_jobs[chosen].append((customer, launch_cell, reconvene_cell))
        drone_last_reconvene[chosen] = reconvene_cell - 1

    part2: List[int] = []
    part3: List[int] = []
    part4: List[int] = []

    for d in range(n_drones):
        for customer, launch_cell, reconvene_cell in drone_jobs[d]:
            part2.append(int(customer))
            part3.append(int(launch_cell))
            part4.append(int(reconvene_cell))
        if d < n_drones - 1:
            part2.append(-1)
            part3.append(-1)
            part4.append(-1)

    return part2, part3, part4


def convert_to_parts_format(solution: Solution, n_drones: int) -> Solution:
    route = solution["route"]
    trips: List[Trip] = solution["trips"]

    converted: List[Tuple[int, int, int]] = []
    for launch_node, customer, reconvene_node in trips:
        pos = _first_valid_positions(route, launch_node, reconvene_node)
        if pos is None:
            continue
        launch_idx, reconvene_idx = pos
        converted.append((int(customer), launch_idx + 1, reconvene_idx + 1))

    part2, part3, part4 = _assign_trips_to_drones(converted, n_drones=n_drones)

    return {
        "part1": list(route),
        "part2": part2,
        "part3": part3,
        "part4": part4,
    }


def evaluate(data: Dict[str, Any], solution: Solution, n_drones: int) -> float:
    parts = convert_to_parts_format(solution, n_drones=n_drones)

    calculator = CalCulateTotalArrivalTime()
    calculator.depot_index = 0
    calculator.truck_times = data["T"]
    calculator.drone_times = data["D"]
    calculator.flight_range = data["drone_limit"]

    total_time, _, _, hover_feasible = calculator.calculate_total_waiting_time(parts)
    if not hover_feasible:
        return float("inf")

    return float(total_time)


def score_solution(data: Dict[str, Any], solution: Solution, n_drones: int) -> float:
    """Return objective value if feasible, otherwise +inf.

    This computes feasibility and objective in one pass to avoid repeated checks.
    """
    parts_solution = convert_to_parts_format(solution, n_drones=n_drones)

    checker = SolutionFeasibility(
        n_nodes=data["n"],
        n_drones=n_drones,
        depot_index=0,
        drone_times=data["D"],
        flight_range=data["drone_limit"],
    )
    if not checker.is_solution_feasible(parts_solution):
        return float("inf")

    calculator = CalCulateTotalArrivalTime()
    calculator.depot_index = 0
    calculator.truck_times = data["T"]
    calculator.drone_times = data["D"]
    calculator.flight_range = data["drone_limit"]

    total_time, _, _, hover_feasible = calculator.calculate_total_waiting_time(parts_solution)
    if not hover_feasible:
        return float("inf")
    return float(total_time)


def is_valid(data: Dict[str, Any], solution: Solution, n_drones: int) -> bool:
    parts_solution = convert_to_parts_format(solution, n_drones=n_drones)

    checker = SolutionFeasibility(
        n_nodes=data["n"],
        n_drones=n_drones,
        depot_index=0,
        drone_times=data["D"],
        flight_range=data["drone_limit"],
    )
    if not checker.is_solution_feasible(parts_solution):
        return False

    calculator = CalCulateTotalArrivalTime()
    calculator.depot_index = 0
    calculator.truck_times = data["T"]
    calculator.drone_times = data["D"]
    calculator.flight_range = data["drone_limit"]
    _, _, _, hover_ok = calculator.calculate_total_waiting_time(parts_solution)
    return bool(hover_ok)


def remove_customer(solution: Solution, customer: int) -> None:
    if customer in solution["route"]:
        solution["route"].remove(customer)
    solution["trips"] = [trip for trip in solution["trips"] if trip[1] != customer]


def op1_critical_customer_relocate(data: Dict[str, Any], solution: Solution, rng: random.Random, n_drones: int) -> Solution:
    """Relocate a critical customer with A3-style drone insertion attempt first."""
    base = deepcopy(solution)
    route = base["route"]

    truck_customers = [node for node in route if node != 0]
    if len(truck_customers) <= 1:
        return base

    critical_scores = []
    for idx in range(1, len(route) - 1):
        customer = route[idx]
        prev_node = route[idx - 1]
        next_node = route[idx + 1]
        detour = data["T"][prev_node][customer] + data["T"][customer][next_node] - data["T"][prev_node][next_node]
        critical_scores.append((customer, detour))

    critical_scores.sort(key=lambda item: item[1], reverse=True)
    top_k = [c for c, _ in critical_scores[: min(5, len(critical_scores))]]
    chosen_customer = int(rng.choice(top_k))

    remove_customer(base, chosen_customer)
    stripped_route = base["route"]

    # A3-style: try to serve selected customer by drone before truck reinsertion.
    if rng.random() < 0.5 and len(stripped_route) >= 3:
        launch_pos = rng.randint(0, len(stripped_route) - 2)
        land_pos = rng.randint(launch_pos + 1, min(launch_pos + 10, len(stripped_route) - 1))
        launch_node = stripped_route[launch_pos]
        land_node = stripped_route[land_pos]
        if launch_node != land_node:
            direct_flight = data["D"][launch_node][chosen_customer] + data["D"][chosen_customer][land_node]
            if direct_flight <= data["drone_limit"]:
                drone_candidate = deepcopy(base)
                drone_candidate["trips"].append((launch_node, chosen_customer, land_node))
                if math.isfinite(score_solution(data, drone_candidate, n_drones=n_drones)):
                    return drone_candidate

    positions = list(range(1, len(stripped_route)))
    if not positions:
        return solution

    # Best-of-few keeps quality robust without expensive full enumeration.
    candidate_positions = positions if len(positions) <= 4 else rng.sample(positions, 4)

    best_candidate = deepcopy(solution)
    best_value = score_solution(data, best_candidate, n_drones=n_drones)

    for insert_pos in candidate_positions:
        candidate = deepcopy(base)
        candidate["route"].insert(insert_pos, chosen_customer)
        value = score_solution(data, candidate, n_drones=n_drones)
        if value < best_value:
            best_value = value
            best_candidate = candidate

    return best_candidate


def op2_truck_2opt_repair(
    data: Dict[str, Any],
    solution: Solution,
    rng: random.Random,
    n_drones: int,
    samples: int = 3,
) -> Solution:
    """Apply sampled-best random 2-opt style segment reversal."""
    base = deepcopy(solution)
    route = base["route"]

    if len(route) <= 5:
        return base

    best_candidate = deepcopy(solution)
    best_value = score_solution(data, best_candidate, n_drones=n_drones)

    for _ in range(samples):
        i = rng.randint(1, len(route) - 3)
        j = rng.randint(i + 1, len(route) - 2)
        candidate = deepcopy(base)
        candidate["route"] = route[:i] + list(reversed(route[i : j + 1])) + route[j + 1 :]
        value = score_solution(data, candidate, n_drones=n_drones)
        if value < best_value:
            best_value = value
            best_candidate = candidate

    return best_candidate


def _trip_criticality(data: Dict[str, Any], route: List[int], trip: Trip) -> float:
    launch_node, customer, reconvene_node = trip
    position_pair = _first_valid_positions(route, launch_node, reconvene_node)
    span = 0
    if position_pair is not None:
        launch_idx, reconvene_idx = position_pair
        span = reconvene_idx - launch_idx

    flight = data["D"][launch_node][customer] + data["D"][customer][reconvene_node]
    return float(flight + span)


def _sample_anchor_pairs(route_len: int, rng: random.Random, max_pairs: int = 18) -> List[Tuple[int, int]]:
    """Sample candidate launch/reconvene index pairs to cap OP3 runtime."""
    if route_len < 2:
        return []

    pairs: Set[Tuple[int, int]] = set()
    max_unique_pairs = (route_len * (route_len - 1)) // 2
    target = min(max_pairs, max_unique_pairs)
    if target <= 0:
        return []

    # Keep short-span anchors as high-value deterministic candidates.
    for i in range(route_len - 1):
        for gap in (1, 2, 3):
            j = i + gap
            if j < route_len:
                pairs.add((i, j))

    # Add random long-span candidates for diversification.
    # Never try to sample beyond the number of unique possible pairs.
    while len(pairs) < target:
        i = rng.randint(0, route_len - 2)
        j = rng.randint(i + 1, route_len - 1)
        pairs.add((i, j))

    return list(pairs)


def op3_drone_reassign_best_anchor(data: Dict[str, Any], solution: Solution, rng: random.Random, n_drones: int) -> Solution:
    """Create/reassign one drone customer with best-of-few anchor selection."""
    base = deepcopy(solution)
    route = base["route"]

    # Choose customer source: either reassign an existing drone trip, or create a new drone trip from truck.
    customer = None
    removed_from_route = False
    truck_customers = [node for node in route if node != 0]
    create_from_truck = bool(truck_customers) and (not base["trips"] or rng.random() < 0.65)

    if not create_from_truck and base["trips"]:
        scored_trips = [(trip, _trip_criticality(data, route, trip)) for trip in base["trips"]]
        scored_trips.sort(key=lambda item: item[1], reverse=True)
        chosen_trip = rng.choice([trip for trip, _ in scored_trips[: min(5, len(scored_trips))]])
        _, customer, _ = chosen_trip
        base["trips"] = [trip for trip in base["trips"] if trip != chosen_trip]
    else:
        if not truck_customers:
            return base
        detours: List[Tuple[int, float]] = []
        for idx in range(1, len(route) - 1):
            c = route[idx]
            prev_node = route[idx - 1]
            next_node = route[idx + 1]
            detour = data["T"][prev_node][c] + data["T"][c][next_node] - data["T"][prev_node][next_node]
            detours.append((c, detour))
        detours.sort(key=lambda item: item[1], reverse=True)
        customer = int(rng.choice([c for c, _ in detours[: min(5, len(detours))]]))
        if customer in base["route"]:
            base["route"].remove(customer)
            removed_from_route = True

    anchor_pairs = _sample_anchor_pairs(len(route), rng, max_pairs=18)
    rng.shuffle(anchor_pairs)

    best_candidate = deepcopy(solution)
    best_value = score_solution(data, best_candidate, n_drones=n_drones)
    found_feasible_drone_candidate = False

    # Try a small number of candidate anchor pairs and keep best feasible.
    for i, j in anchor_pairs[:8]:
        launch_node = route[i]
        reconvene_node = route[j]
        direct_flight = data["D"][launch_node][customer] + data["D"][customer][reconvene_node]
        if direct_flight > data["drone_limit"]:
            continue

        candidate = deepcopy(base)
        candidate["trips"].append((launch_node, customer, reconvene_node))
        value = score_solution(data, candidate, n_drones=n_drones)
        if value < best_value:
            best_value = value
            best_candidate = candidate
            found_feasible_drone_candidate = True

    # If customer came from a drone trip and no better reassignment was found, allow truck reinsertion.
    if not removed_from_route and not found_feasible_drone_candidate:
        positions = list(range(1, len(route)))
        if positions and customer not in base["route"]:
            base["route"].insert(rng.choice(positions), customer)
        return base

    if found_feasible_drone_candidate:
        return best_candidate

    # If customer was removed from truck and no feasible anchor sampled, revert safely.
    if customer not in base["route"]:
        positions = list(range(1, len(route)))
        if positions:
            base["route"].insert(rng.choice(positions), customer)
    return base


def apply_selected_operator(
    data: Dict[str, Any],
    incumbent: Solution,
    rng: random.Random,
    n_drones: int,
    probabilities: Tuple[float, float, float],
) -> Tuple[Solution, int]:
    p1, p2, p3 = probabilities
    r = rng.random()

    if r < p1:
        return op1_critical_customer_relocate(data, incumbent, rng, n_drones=n_drones), 1
    if r < p1 + p2:
        return op2_truck_2opt_repair(data, incumbent, rng, n_drones=n_drones), 2
    return op3_drone_reassign_best_anchor(data, incumbent, rng, n_drones=n_drones), 3


def run_modified_simulated_annealing(
    data: Dict[str, Any],
    rng: random.Random,
    n_drones: int,
    probabilities: Tuple[float, float, float],
    warmup_iters: int = 100,
    main_iters: int = 9900,
    final_temp: float = 0.1,
) -> Tuple[Solution, float, float]:
    incumbent = initial_solution(data)
    incumbent_obj = score_solution(data, incumbent, n_drones=n_drones)

    best_solution = deepcopy(incumbent)
    best_obj = incumbent_obj

    warmup_deltas: List[float] = []
    started = time.perf_counter()

    for _ in range(warmup_iters):
        new_solution, _ = apply_selected_operator(
            data,
            incumbent,
            rng,
            n_drones=n_drones,
            probabilities=probabilities,
        )

        new_obj = score_solution(data, new_solution, n_drones=n_drones)
        if not math.isfinite(new_obj):
            continue
        delta = new_obj - incumbent_obj

        if delta < 0:
            incumbent = new_solution
            incumbent_obj = new_obj
            if incumbent_obj < best_obj:
                best_solution = deepcopy(incumbent)
                best_obj = incumbent_obj
        else:
            if rng.random() < 0.8:
                incumbent = new_solution
                incumbent_obj = new_obj
            warmup_deltas.append(delta)

    delta_avg = mean(warmup_deltas) if warmup_deltas else 1.0
    if delta_avg <= 0 or not math.isfinite(delta_avg):
        delta_avg = 1.0

    t0 = -delta_avg / math.log(0.8)
    if t0 <= 0 or not math.isfinite(t0):
        t0 = 1.0

    alpha = (final_temp / t0) ** (1.0 / max(main_iters, 1))
    temperature = t0

    for _ in range(main_iters):
        new_solution, _ = apply_selected_operator(
            data,
            incumbent,
            rng,
            n_drones=n_drones,
            probabilities=probabilities,
        )

        new_obj = score_solution(data, new_solution, n_drones=n_drones)
        if math.isfinite(new_obj):
            delta = new_obj - incumbent_obj

            if delta < 0:
                incumbent = new_solution
                incumbent_obj = new_obj
                if incumbent_obj < best_obj:
                    best_solution = deepcopy(incumbent)
                    best_obj = incumbent_obj
            else:
                acceptance_probability = math.exp(-delta / max(temperature, 1e-12))
                if rng.random() < acceptance_probability:
                    incumbent = new_solution
                    incumbent_obj = new_obj

        temperature = alpha * temperature

    elapsed = time.perf_counter() - started
    return best_solution, best_obj, elapsed


def to_solution_string(solution: Solution, n_drones: int) -> str:
    parts = convert_to_parts_format(solution, n_drones=n_drones)
    return (
        f"{','.join(map(str, parts['part1']))}|"
        f"{','.join(map(str, parts['part2']))}|"
        f"{','.join(map(str, parts['part3']))}|"
        f"{','.join(map(str, parts['part4']))}"
    )


def run_configuration(
    method_name: str,
    probabilities: Tuple[float, float, float],
    data_dir: Path,
    instances: List[str],
    runs: int,
    seed: int,
    warmup_iters: int,
    main_iters: int,
    final_temp: float,
    n_drones: int,
    solutions_dir: Path,
    show_header: bool = True,
) -> Tuple[List[Dict[str, Any]], Dict[str, Solution]]:
    rows: List[Dict[str, Any]] = []
    best_solutions: Dict[str, Solution] = {}

    if show_header:
        print("\n" + "=" * 70)
        print(f"Configuration: {method_name}  (P1, P2, P3)={probabilities}")
        print("=" * 70)

    for instance_name in instances:
        instance_file = data_dir / f"{instance_name}.txt"
        if not instance_file.exists():
            print(f"Skipping {instance_name}: missing file")
            continue

        data = read_instance(str(instance_file))
        initial_obj = score_solution(data, initial_solution(data), n_drones=n_drones)

        run_objs: List[float] = []
        run_times: List[float] = []
        best_sol: Optional[Solution] = None
        best_obj = float("inf")

        print(f"\n{instance_name}: s0={initial_obj:.1f}")
        for run_idx in range(runs):
            rng = random.Random(seed + run_idx)
            solution, objective_value, elapsed = run_modified_simulated_annealing(
                data,
                rng,
                n_drones=n_drones,
                probabilities=probabilities,
                warmup_iters=warmup_iters,
                main_iters=main_iters,
                final_temp=final_temp,
            )
            run_objs.append(objective_value)
            run_times.append(elapsed)

            if objective_value < best_obj:
                best_obj = objective_value
                best_sol = solution

            print(f"  run {run_idx + 1:2d}: best={objective_value:.1f} time={elapsed:.3f}s")

        if best_sol is None:
            continue

        avg_obj = sum(run_objs) / len(run_objs)
        avg_time = sum(run_times) / len(run_times)
        improvement = 0.0
        if initial_obj > 0 and initial_obj != float("inf"):
            improvement = 100.0 * (initial_obj - best_obj) / initial_obj

        print(f"  avg={avg_obj:.1f} best={best_obj:.1f} improv={improvement:.1f}% time={avg_time:.3f}s")

        best_solution_string = to_solution_string(best_sol, n_drones=n_drones)
        rows.append(
            {
                "instance": instance_name,
                "method": method_name,
                "avg": avg_obj,
                "best": best_obj,
                "improvement": improvement,
                "time": avg_time,
                "best_solution_string": best_solution_string,
            }
        )
        best_solutions[instance_name] = best_sol

        method_slug = "equal" if "equal" in method_name.lower() else "tuned"
        instance_solution_path = solutions_dir / f"{instance_name}_simulated_annealing_new_{method_slug}_best.txt"
        with open(instance_solution_path, "w", encoding="utf-8") as file_handle:
            file_handle.write(f"Instance: {instance_name}\n")
            file_handle.write(f"Method: {method_name}\n")
            file_handle.write(f"Average objective: {avg_obj:.1f}\n")
            file_handle.write(f"Best objective: {best_obj:.1f}\n")
            file_handle.write(f"Improvement (%): {improvement:.1f}\n")
            file_handle.write(f"Average running time (s): {avg_time:.3f}\n")
            file_handle.write(f"Best solution string: {best_solution_string}\n")

    return rows, best_solutions


def select_tuned_weights_for_instance(
    data: Dict[str, Any],
    base_weights: Tuple[float, float, float],
    seed: int,
    n_drones: int,
) -> Tuple[float, float, float]:
    """Pick robust tuned weights using a very small pilot search."""
    equal_weights = (1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0)
    candidates: List[Tuple[float, float, float]] = [
        base_weights,
        (0.50, 0.20, 0.30),
        (0.45, 0.20, 0.35),
        (0.55, 0.15, 0.30),
        (0.40, 0.20, 0.40),
        (0.35, 0.25, 0.40),
    ]

    best_weights = base_weights
    best_score = float("inf")

    for weights in candidates:
        if abs(sum(weights) - 1.0) > 1e-9:
            continue

        pilot_scores: List[float] = []
        for k in range(3):
            rng = random.Random(seed + k)
            _, objective, _ = run_modified_simulated_annealing(
                data=data,
                rng=rng,
                n_drones=n_drones,
                probabilities=weights,
                warmup_iters=12,
                main_iters=180,
                final_temp=0.1,
            )
            pilot_scores.append(objective)

        avg_score = sum(pilot_scores) / len(pilot_scores)
        if avg_score < best_score:
            best_score = avg_score
            best_weights = weights

    # Tuned mode must remain tuned (different from equal weights).
    if all(abs(best_weights[i] - equal_weights[i]) < 1e-9 for i in range(3)):
        best_weights = base_weights

    return best_weights


def main() -> None:
    parser = argparse.ArgumentParser(description="INF273 Assignment 4 - Modified SA with 3 operators")
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--warmup-iters", type=int, default=100)
    parser.add_argument("--iters", type=int, default=9900)
    parser.add_argument("--final-temp", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--n-drones", type=int, default=2)
    parser.add_argument("--equal-weights", nargs=3, type=float, default=[1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0])
    parser.add_argument("--tuned-weights", nargs=3, type=float, default=[0.37, 0.47, 0.16])
    parser.add_argument(
        "--instances",
        nargs="*",
        default=["F_10", "F_20", "F_50", "F_100", "R_10", "R_20", "R_50", "R_100"],
    )
    args = parser.parse_args()

    equal_weights = tuple(args.equal_weights)
    tuned_weights = tuple(args.tuned_weights)

    if abs(sum(equal_weights) - 1.0) > 1e-9:
        raise ValueError("equal-weights must sum to 1.0")
    if abs(sum(tuned_weights) - 1.0) > 1e-9:
        raise ValueError("tuned-weights must sum to 1.0")

    assignment_dir = Path(__file__).resolve().parent.parent
    default_data_dir = assignment_dir / "data"
    fallback_data_dir = assignment_dir.parent / "assignment3" / "data"
    data_dir = default_data_dir if default_data_dir.exists() else fallback_data_dir

    out_dir = assignment_dir / "results"
    out_dir.mkdir(exist_ok=True)
    solutions_dir = out_dir / "solutions"
    solutions_dir.mkdir(exist_ok=True)

    print("=" * 70)
    print("INF273 Assignment 4 - Modified Simulated Annealing")
    print("Operators: OP1 critical relocate, OP2 sampled 2-opt, OP3 drone reassignment")
    print("=" * 70)
    print(f"Using data directory: {data_dir}")

    equal_rows: List[Dict[str, Any]] = []
    equal_best: Dict[str, Solution] = {}
    tuned_rows: List[Dict[str, Any]] = []
    tuned_best: Dict[str, Solution] = {}

    # Alternate output per instance: equal first, then tuned.
    for instance_name in args.instances:
        print("\n" + "=" * 70)
        print(f"{instance_name}-new-operators-equal-weights")
        print("=" * 70)

        rows_eq, best_eq = run_configuration(
            method_name="SA-new operators (equal weights)",
            probabilities=equal_weights,
            data_dir=data_dir,
            instances=[instance_name],
            runs=args.runs,
            seed=args.seed,
            warmup_iters=args.warmup_iters,
            main_iters=args.iters,
            final_temp=args.final_temp,
            n_drones=args.n_drones,
            solutions_dir=solutions_dir,
            show_header=False,
        )
        equal_rows.extend(rows_eq)
        equal_best.update(best_eq)

        instance_file = data_dir / f"{instance_name}.txt"
        if not instance_file.exists():
            continue

        data = read_instance(str(instance_file))
        tuned_for_instance = select_tuned_weights_for_instance(
            data=data,
            base_weights=tuned_weights,
            seed=args.seed,
            n_drones=args.n_drones,
        )

        print("\n" + "=" * 70)
        print(f"{instance_name}-new-operators-tuned-weights")
        print(f"selected (P1,P2,P3)=({tuned_for_instance[0]:.2f},{tuned_for_instance[1]:.2f},{tuned_for_instance[2]:.2f})")
        print("=" * 70)

        rows_tu, best_tu = run_configuration(
            method_name="SA-new operators (tuned weights)",
            probabilities=tuned_for_instance,
            data_dir=data_dir,
            instances=[instance_name],
            runs=args.runs,
            seed=args.seed,
            warmup_iters=args.warmup_iters,
            main_iters=args.iters,
            final_temp=args.final_temp,
            n_drones=args.n_drones,
            solutions_dir=solutions_dir,
            show_header=False,
        )
        tuned_rows.extend(rows_tu)
        tuned_best.update(best_tu)

    all_rows = equal_rows + tuned_rows

    csv_path = out_dir / "results_simulated_annealing_new_operators.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as file_handle:
        writer = csv.writer(file_handle)
        writer.writerow([
            "Instance",
            "Method",
            "Average objective",
            "Best objective",
            "Improvement (%)",
            "Average running time (s)",
            "Best solution string",
        ])
        for row in all_rows:
            writer.writerow(
                [
                    row["instance"],
                    row["method"],
                    f"{row['avg']:.1f}",
                    f"{row['best']:.1f}",
                    f"{row['improvement']:.1f}",
                    f"{row['time']:.3f}",
                    row["best_solution_string"],
                ]
            )

    equal_solution_file = out_dir / "solutions_simulated_annealing_new_equal.txt"
    with open(equal_solution_file, "w", encoding="utf-8") as file_handle:
        for instance_name, solution in equal_best.items():
            file_handle.write(f"{instance_name}: {to_solution_string(solution, n_drones=args.n_drones)}\n")

    tuned_solution_file = out_dir / "solutions_simulated_annealing_new_tuned.txt"
    with open(tuned_solution_file, "w", encoding="utf-8") as file_handle:
        for instance_name, solution in tuned_best.items():
            file_handle.write(f"{instance_name}: {to_solution_string(solution, n_drones=args.n_drones)}\n")

    best_overall_file = out_dir / "solutions_simulated_annealing_new_best_of_two.txt"
    with open(best_overall_file, "w", encoding="utf-8") as file_handle:
        best_lookup: Dict[Tuple[str, str], float] = {}
        for row in all_rows:
            best_lookup[(row["instance"], row["method"])] = row["best"]

        for instance_name in args.instances:
            eq_obj = best_lookup.get((instance_name, "SA-new operators (equal weights)"), float("inf"))
            tu_obj = best_lookup.get((instance_name, "SA-new operators (tuned weights)"), float("inf"))
            if eq_obj <= tu_obj and instance_name in equal_best:
                file_handle.write(
                    f"{instance_name}: method=SA-new operators (equal weights), "
                    f"objective={eq_obj:.1f}, solution={to_solution_string(equal_best[instance_name], n_drones=args.n_drones)}\n"
                )
            elif instance_name in tuned_best:
                file_handle.write(
                    f"{instance_name}: method=SA-new operators (tuned weights), "
                    f"objective={tu_obj:.1f}, solution={to_solution_string(tuned_best[instance_name], n_drones=args.n_drones)}\n"
                )

    print("\n" + "=" * 70)
    print("RESULTS TABLE")
    print("=" * 70)
    print(
        f"{'Instance':<12} {'Method':<34} {'Avg obj':>12} {'Best obj':>12} "
        f"{'Improv(%)':>12} {'Avg time(s)':>14}"
    )
    for row in all_rows:
        print(
            f"{row['instance']:<12} {row['method']:<34} {row['avg']:>12.1f} "
            f"{row['best']:>12.1f} {row['improvement']:>12.1f} {row['time']:>14.3f}"
        )

    print(f"\nSaved results: {csv_path}")
    print(f"Saved equal-weight solutions: {equal_solution_file}")
    print(f"Saved tuned-weight solutions: {tuned_solution_file}")
    print(f"Saved best-of-two solutions: {best_overall_file}")


if __name__ == "__main__":
    main()
