#!/usr/bin/env python3
"""INF273 Assignment 3 - Simulated Annealing with 1-reinsert operator."""

import argparse
import csv
import math
import random
import sys
import time
from copy import deepcopy
from pathlib import Path
from statistics import mean

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from FeasibilityCheck import SolutionFeasibility
from CalCulateTotalArrivalTime import CalCulateTotalArrivalTime


def read_instance(path: str) -> dict:
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


def initial_solution(data: dict) -> dict:
    return {"route": [0] + list(range(1, data["n_customers"] + 1)) + [0], "trips": []}


def convert_to_parts_format(solution: dict) -> dict:
    route = solution["route"]
    trips = solution["trips"]

    converted = []
    for launch_node, customer, land_node in trips:
        launch_pos = None
        land_pos = None
        for route_idx, node in enumerate(route):
            if node == launch_node and launch_pos is None:
                launch_pos = route_idx
            if node == land_node and launch_pos is not None and route_idx > launch_pos:
                land_pos = route_idx
                break

        if launch_pos is not None and land_pos is not None:
            converted.append((customer, launch_pos, land_pos))

    converted.sort(key=lambda item: item[1])

    drone1 = []
    drone2 = []
    drone1_end = -1
    drone2_end = -1
    for customer, launch_pos, land_pos in converted:
        if launch_pos >= drone1_end:
            drone1.append((customer, launch_pos + 1, land_pos + 1))
            drone1_end = land_pos
        elif launch_pos >= drone2_end:
            drone2.append((customer, launch_pos + 1, land_pos + 1))
            drone2_end = land_pos

    part2 = [c for c, _, _ in drone1] + [-1] + [c for c, _, _ in drone2]
    part3 = [l for _, l, _ in drone1] + [-1] + [l for _, l, _ in drone2]
    part4 = [r for _, _, r in drone1] + [-1] + [r for _, _, r in drone2]

    return {"part1": route, "part2": part2, "part3": part3, "part4": part4}


def evaluate(data: dict, solution: dict) -> float:
    parts = convert_to_parts_format(solution)

    calculator = CalCulateTotalArrivalTime()
    calculator.depot_index = 0
    calculator.truck_times = data["T"]
    calculator.drone_times = data["D"]
    calculator.flight_range = data["drone_limit"]

    total_time, _, _, hover_feasible = calculator.calculate_total_waiting_time(parts)
    if not hover_feasible:
        return float("inf")

    return total_time


def is_valid(data: dict, solution: dict) -> bool:
    parts_solution = convert_to_parts_format(solution)

    checker = SolutionFeasibility(
        n_nodes=data["n"],
        n_drones=2,
        depot_index=0,
        drone_times=data["D"],
        flight_range=data["drone_limit"],
    )
    structural_ok = checker.is_solution_feasible(parts_solution)
    if not structural_ok:
        return False

    calculator = CalCulateTotalArrivalTime()
    calculator.depot_index = 0
    calculator.truck_times = data["T"]
    calculator.drone_times = data["D"]
    calculator.flight_range = data["drone_limit"]
    _, _, _, hover_ok = calculator.calculate_total_waiting_time(parts_solution)
    return hover_ok


def remove_customer(solution: dict, customer: int) -> None:
    if customer in solution["route"]:
        solution["route"].remove(customer)
    solution["trips"] = [trip for trip in solution["trips"] if trip[1] != customer]


def one_reinsert(data: dict, solution: dict, rng: random.Random) -> dict:
    new_solution = deepcopy(solution)

    truck_customers = [c for c in new_solution["route"] if c != 0]
    drone_customers = [trip[1] for trip in new_solution["trips"]]
    all_customers = truck_customers + drone_customers
    if not all_customers:
        return new_solution

    customer = rng.choice(all_customers)
    remove_customer(new_solution, customer)

    route = new_solution["route"]

    if rng.random() < 0.5 and len(route) >= 3:
        launch_pos = rng.randint(0, len(route) - 2)
        land_pos = rng.randint(launch_pos + 1, min(launch_pos + 10, len(route) - 1))
        launch_node = route[launch_pos]
        land_node = route[land_pos]

        if launch_node != land_node:
            direct_flight = data["D"][launch_node][customer] + data["D"][customer][land_node]
            if direct_flight <= data["drone_limit"]:
                new_solution["trips"].append((launch_node, customer, land_node))
                return new_solution

    insert_pos = rng.randint(1, len(route) - 1)
    new_solution["route"].insert(insert_pos, customer)
    return new_solution


def run_simulated_annealing(data: dict, rng: random.Random, warmup_iters: int = 100, main_iters: int = 9900, final_temp: float = 0.1) -> tuple[dict, float, float]:
    incumbent = initial_solution(data)
    incumbent_objective = evaluate(data, incumbent)

    best_solution = deepcopy(incumbent)
    best_objective = incumbent_objective

    warmup_deltas = []
    started = time.perf_counter()

    for _ in range(warmup_iters):
        new_solution = one_reinsert(data, incumbent, rng)
        if not is_valid(data, new_solution):
            continue

        new_objective = evaluate(data, new_solution)
        delta = new_objective - incumbent_objective

        if delta < 0:
            incumbent = new_solution
            incumbent_objective = new_objective
            if incumbent_objective < best_objective:
                best_solution = deepcopy(incumbent)
                best_objective = incumbent_objective
        else:
            if rng.random() < 0.8:
                incumbent = new_solution
                incumbent_objective = new_objective
            warmup_deltas.append(delta)

    delta_avg = mean(warmup_deltas) if warmup_deltas else 1.0
    t0 = -delta_avg / math.log(0.8)
    if not math.isfinite(t0) or t0 <= 0:
        t0 = 1.0

    temperature = t0
    alpha = (final_temp / t0) ** (1.0 / max(main_iters, 1))

    for _ in range(main_iters):
        new_solution = one_reinsert(data, incumbent, rng)
        if not is_valid(data, new_solution):
            temperature = alpha * temperature
            continue

        new_objective = evaluate(data, new_solution)
        delta = new_objective - incumbent_objective

        if delta < 0:
            incumbent = new_solution
            incumbent_objective = new_objective
            if incumbent_objective < best_objective:
                best_solution = deepcopy(incumbent)
                best_objective = incumbent_objective
        else:
            acceptance_probability = math.exp(-delta / max(temperature, 1e-12))
            if rng.random() < acceptance_probability:
                incumbent = new_solution
                incumbent_objective = new_objective

        temperature = alpha * temperature

    elapsed = time.perf_counter() - started
    return best_solution, best_objective, elapsed


def to_solution_string(solution: dict) -> str:
    parts = convert_to_parts_format(solution)
    return (
        f"{','.join(map(str, parts['part1']))}|"
        f"{','.join(map(str, parts['part2']))}|"
        f"{','.join(map(str, parts['part3']))}|"
        f"{','.join(map(str, parts['part4']))}"
    )


def validate_post_run(data: dict, solution: dict, expected_objective: float) -> tuple[bool, bool, bool, float]:
    parts = convert_to_parts_format(solution)

    checker = SolutionFeasibility(
        n_nodes=data["n"],
        n_drones=2,
        depot_index=0,
        drone_times=data["D"],
        flight_range=data["drone_limit"],
    )
    structural_ok = checker.is_solution_feasible(parts)

    calculator = CalCulateTotalArrivalTime()
    calculator.depot_index = 0
    calculator.truck_times = data["T"]
    calculator.drone_times = data["D"]
    calculator.flight_range = data["drone_limit"]
    recalculated_objective, _, _, hover_ok = calculator.calculate_total_waiting_time(parts)

    objective_match = abs(recalculated_objective - expected_objective) <= 1e-9
    return structural_ok, hover_ok, objective_match, recalculated_objective


def main() -> None:
    parser = argparse.ArgumentParser(description="INF273 Assignment 3 - Simulated Annealing")
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--warmup-iters", type=int, default=100)
    parser.add_argument("--iters", type=int, default=9900)
    parser.add_argument("--final-temp", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--instances", nargs="*", default=["F_10", "F_20", "F_50", "F_100", "R_10", "R_20", "R_50", "R_100"])
    args = parser.parse_args()

    base = Path(__file__).resolve().parent.parent
    data_dir = base / "Data"
    out_dir = base / "assignment3"
    solutions_dir = out_dir / "solutions"
    out_dir.mkdir(exist_ok=True)
    solutions_dir.mkdir(exist_ok=True)

    print("=" * 70)
    print("INF273 Assignment 3 - Simulated Annealing (1-reinsert)")
    print("=" * 70)

    rows = []
    best_by_instance = {}
    validator_total = 0
    validator_passed = 0

    for instance_name in args.instances:
        instance_file = data_dir / f"{instance_name}.txt"
        if not instance_file.exists():
            print(f"Skipping {instance_name}: missing file")
            continue

        data = read_instance(str(instance_file))
        initial_obj = evaluate(data, initial_solution(data))
        print(f"\n{instance_name}: s0={initial_obj:.1f}")

        run_objs = []
        run_times = []
        best_sol = None
        best_obj = float("inf")

        for run_idx in range(args.runs):
            rng = random.Random(args.seed + run_idx)
            solution, objective_value, elapsed = run_simulated_annealing(
                data,
                rng,
                warmup_iters=args.warmup_iters,
                main_iters=args.iters,
                final_temp=args.final_temp,
            )
            run_objs.append(objective_value)
            run_times.append(elapsed)
            if objective_value < best_obj:
                best_obj = objective_value
                best_sol = solution
            print(f"  run {run_idx + 1:2d}: best={objective_value:.1f} time={elapsed:.3f}s")

        avg_obj = sum(run_objs) / args.runs
        avg_time = sum(run_times) / args.runs
        improvement = 0.0
        if initial_obj > 0 and initial_obj != float("inf"):
            improvement = 100.0 * (initial_obj - best_obj) / initial_obj

        rows.append({
            "instance": instance_name,
            "method": "Simulated Annealing-1-reinsert",
            "avg": avg_obj,
            "best": best_obj,
            "improvement": improvement,
            "time": avg_time,
        })
        best_by_instance[instance_name] = best_sol

        structural_ok, hover_ok, objective_match, recalculated_obj = validate_post_run(data, best_sol, best_obj)
        validator_total += 1
        validation_pass = structural_ok and hover_ok and objective_match
        if validation_pass:
            validator_passed += 1

        validation_status = "PASS" if validation_pass else "FAIL"
        print(
            f"  validator={validation_status} "
            f"(structural={structural_ok}, hover={hover_ok}, objective_match={objective_match})"
        )

        instance_solution_path = solutions_dir / f"{instance_name}_simulated_annealing_best.txt"
        with open(instance_solution_path, "w", encoding="utf-8") as file_handle:
            file_handle.write(f"Instance: {instance_name}\n")
            file_handle.write("Method: Simulated Annealing-1-reinsert\n")
            file_handle.write(f"Best objective: {best_obj:.1f}\n")
            file_handle.write(f"Post-run structural feasible: {structural_ok}\n")
            file_handle.write(f"Post-run hover feasible: {hover_ok}\n")
            file_handle.write(f"Post-run objective match: {objective_match}\n")
            file_handle.write(f"Post-run recalculated objective: {recalculated_obj:.1f}\n")
            if best_sol is not None:
                file_handle.write(f"Best solution string: {to_solution_string(best_sol)}\n")

        print(f"  avg={avg_obj:.1f} best={best_obj:.1f} improv={improvement:.1f}% time={avg_time:.3f}s")

    csv_path = out_dir / "results_simulated_annealing.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as file_handle:
        writer = csv.writer(file_handle)
        writer.writerow(["Instance", "Method", "Average objective", "Best objective", "Improvement (%)", "Average running time (s)"])
        for row in rows:
            writer.writerow([
                row["instance"],
                row["method"],
                f"{row['avg']:.1f}",
                f"{row['best']:.1f}",
                f"{row['improvement']:.1f}",
                f"{row['time']:.3f}",
            ])

    sol_path = out_dir / "solutions_simulated_annealing.txt"
    with open(sol_path, "w", encoding="utf-8") as file_handle:
        for instance_name, solution in best_by_instance.items():
            if solution is None:
                continue
            file_handle.write(f"{instance_name}: {to_solution_string(solution)}\n")

    print(f"\n{'=' * 70}")
    print("RESULTS TABLE")
    print(f"{'=' * 70}")
    print(
        f"{'Instance':<12} {'Method':<15} {'Avg obj':>12} {'Best obj':>12} "
        f"{'Improv(%)':>12} {'Avg time(s)':>14}"
    )
    for row in rows:
        display_method = "Sim Annealing"
        print(
            f"{row['instance']:<12} {display_method:<15} {row['avg']:>12.1f} "
            f"{row['best']:>12.1f} {row['improvement']:>12.1f} {row['time']:>14.3f}"
        )

    print(f"\nSaved results: {csv_path}")
    print(f"Saved best solutions: {sol_path}")
    print(f"Saved per-instance solution files: {solutions_dir}")
    print(f"Post-run validator summary: {validator_passed}/{validator_total} passed")


if __name__ == "__main__":
    main()
