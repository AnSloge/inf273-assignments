#!/usr/bin/env python3
"""INF273 Assignment 2b – Blind Random Search for Truck-Drone Delivery Problem."""

import csv
import random
import time
from pathlib import Path

from FeasibilityCheck import SolutionFeasibility
from CalCulateTotalArrivalTime import CalCulateTotalArrivalTime


def read_instance(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as file_handle:
        lines = [line.strip() for line in file_handle if line.strip() and not line.startswith("#")]

    index = 0
    n_customers = int(lines[index])
    index += 1
    flight_limit = float(lines[index])
    index += 1

    truck_times = [list(map(float, lines[index + i].split())) for i in range(n_customers + 1)]
    index += n_customers + 1
    drone_times = [list(map(float, lines[index + i].split())) for i in range(n_customers + 1)]

    return {
        "n": n_customers,
        "limit": flight_limit,
        "T": truck_times,
        "D": drone_times,
        "name": Path(path).stem,
    }


def initial_solution(data: dict) -> dict:
    """Initial solution: truck serves customers in index order 1..N."""
    return {"route": [0] + list(range(1, data["n"] + 1)) + [0], "trips": []}


def convert_to_parts_format(solution: dict) -> dict:
    route = solution["route"]
    trips = solution["trips"]

    converted = []
    for launch_node, customer, return_node in trips:
        launch_pos = None
        return_pos = None
        for route_idx, node in enumerate(route):
            if node == launch_node and launch_pos is None:
                launch_pos = route_idx
            if node == return_node and launch_pos is not None and route_idx > launch_pos:
                return_pos = route_idx
                break

        if launch_pos is not None and return_pos is not None:
            converted.append((customer, launch_pos, return_pos))

    converted.sort(key=lambda item: item[1])

    drone1 = []
    drone2 = []
    drone1_end = -1
    drone2_end = -1
    for customer, launch_pos, return_pos in converted:
        if launch_pos >= drone1_end:
            drone1.append((customer, launch_pos + 1, return_pos + 1))
            drone1_end = return_pos
        elif launch_pos >= drone2_end:
            drone2.append((customer, launch_pos + 1, return_pos + 1))
            drone2_end = return_pos

    part2 = [c for c, _, _ in drone1] + [-1] + [c for c, _, _ in drone2]
    part3 = [l for _, l, _ in drone1] + [-1] + [l for _, l, _ in drone2]
    part4 = [r for _, _, r in drone1] + [-1] + [r for _, _, r in drone2]

    return {
        "part1": route,
        "part2": part2,
        "part3": part3,
        "part4": part4,
    }


def evaluate_with_calculator(data: dict, solution: dict) -> tuple[float, bool]:
    parts = convert_to_parts_format(solution)

    calculator = CalCulateTotalArrivalTime()
    calculator.depot_index = 0
    calculator.truck_times = data["T"]
    calculator.drone_times = data["D"]
    calculator.flight_range = data["limit"]

    total_time, _, _, is_hover_feasible = calculator.calculate_total_waiting_time(parts)
    return total_time, is_hover_feasible


def objective(data: dict, solution: dict) -> float:
    total_time, is_hover_feasible = evaluate_with_calculator(data, solution)
    if not is_hover_feasible:
        return float("inf")
    return total_time


def is_feasible(data: dict, solution: dict) -> bool:
    """Feasibility: structural checks + calculator hover/wait-time check."""
    parts_solution = convert_to_parts_format(solution)

    checker = SolutionFeasibility(
        n_nodes=data["n"] + 1,
        n_drones=2,
        depot_index=0,
        drone_times=data["D"],
        flight_range=data["limit"],
    )

    if not checker.is_solution_feasible(parts_solution):
        return False

    _, is_hover_feasible = evaluate_with_calculator(data, solution)
    return is_hover_feasible


def random_solution(data: dict, rng: random.Random) -> dict:
    n_customers = data["n"]
    flight_limit = data["limit"]
    drone_times = data["D"]

    customers = list(range(1, n_customers + 1))
    rng.shuffle(customers)

    n_drone_candidates = rng.randint(0, max(1, n_customers // 3))
    drone_candidates = customers[:n_drone_candidates]
    truck_customers = customers[n_drone_candidates:]

    route = [0] + truck_customers + [0]
    drone_intervals = [[], []]
    trips = []

    for customer in drone_candidates:
        if len(route) < 3:
            route.insert(rng.randint(1, len(route) - 1), customer)
            continue

        assigned = False
        for _ in range(10):
            launch_pos = rng.randint(0, len(route) - 2)
            return_pos = rng.randint(launch_pos + 1, len(route) - 1)
            launch_node = route[launch_pos]
            return_node = route[return_pos]

            direct_flight = drone_times[launch_node][customer] + drone_times[customer][return_node]
            if launch_node == return_node or direct_flight > flight_limit:
                continue

            for drone_idx in range(2):
                overlap = any(not (return_pos <= start or launch_pos >= end) for start, end in drone_intervals[drone_idx])
                if overlap:
                    continue

                trips.append((launch_node, customer, return_node))
                drone_intervals[drone_idx].append((launch_pos, return_pos))
                assigned = True
                break

            if assigned:
                break

        if not assigned:
            route.insert(rng.randint(1, len(route) - 1), customer)

    return {"route": route, "trips": trips}


def to_solution_string(solution: dict) -> str:
    """Contest-style best solution string."""
    parts = convert_to_parts_format(solution)
    return (
        f"{','.join(map(str, parts['part1']))}|"
        f"{','.join(map(str, parts['part2']))}|"
        f"{','.join(map(str, parts['part3']))}|"
        f"{','.join(map(str, parts['part4']))}"
    )


def run_brs(instance_path: str, runs: int = 10, iters: int = 10000, seed: int = 12345) -> dict:
    data = read_instance(instance_path)
    start_solution = initial_solution(data)
    start_objective = objective(data, start_solution)

    print(f"\n{'=' * 70}")
    print(f"Instance {data['name']} (N={data['n']}) | Initial objective = {start_objective:.2f}")
    print(f"{'=' * 70}")

    run_best_values = []
    run_times = []
    best_solution_overall = start_solution
    best_value_overall = start_objective

    for run_idx in range(runs):
        rng = random.Random(seed + run_idx)
        best_solution_in_run = start_solution
        best_value_in_run = start_objective

        started = time.perf_counter()
        for _ in range(iters):
            candidate = random_solution(data, rng)
            if not is_feasible(data, candidate):
                continue

            candidate_value = objective(data, candidate)
            if candidate_value < best_value_in_run:
                best_solution_in_run = candidate
                best_value_in_run = candidate_value

        elapsed = time.perf_counter() - started
        run_best_values.append(best_value_in_run)
        run_times.append(elapsed)

        if best_value_in_run < best_value_overall:
            best_value_overall = best_value_in_run
            best_solution_overall = best_solution_in_run

        print(f"Run {run_idx + 1:2d}: best={best_value_in_run:.2f} time={elapsed:.3f}s")

    average_objective = sum(run_best_values) / runs
    best_objective = min(run_best_values)
    average_time = sum(run_times) / runs
    improvement = 0.0
    if start_objective > 0 and start_objective != float("inf"):
        improvement = 100.0 * (start_objective - best_objective) / start_objective

    print(
        f"Summary {data['name']}: avg={average_objective:.2f}, "
        f"best={best_objective:.2f}, improvement={improvement:.2f}%, avg_time={average_time:.3f}s"
    )

    return {
        "instance": data["name"],
        "method": "Random Search",
        "initial": start_objective,
        "avg": average_objective,
        "best": best_objective,
        "improvement": improvement,
        "avg_time": average_time,
        "best_solution": best_solution_overall,
    }


def main() -> None:
    base = Path(__file__).resolve().parent
    data_dir = base / "Data"
    solutions_dir = base / "solutions"
    solutions_dir.mkdir(exist_ok=True)

    instance_files = [
        "F_10.txt",
        "F_20.txt",
        "F_50.txt",
        "F_100.txt",
        "R_10.txt",
        "R_20.txt",
        "R_50.txt",
        "R_100.txt",
    ]

    print("=" * 70)
    print("INF273 Assignment 2b - Blind Random Search")
    print("=" * 70)

    all_results = []
    for file_name in instance_files:
        instance_path = data_dir / file_name
        if not instance_path.exists():
            continue

        result = run_brs(str(instance_path))
        all_results.append(result)

        solution_path = solutions_dir / f"{result['instance']}_best.txt"
        with open(solution_path, "w", encoding="utf-8") as file_handle:
            file_handle.write(f"Instance: {result['instance']}\n")
            file_handle.write(f"Method: {result['method']}\n")
            file_handle.write(f"Average objective: {result['avg']:.1f}\n")
            file_handle.write(f"Best objective: {result['best']:.1f}\n")
            file_handle.write(f"Improvement (%): {result['improvement']:.1f}\n")
            file_handle.write(f"Average running time (s): {result['avg_time']:.3f}\n")
            file_handle.write(f"Best solution string: {to_solution_string(result['best_solution'])}\n")

    csv_path = base / "results_2b.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as file_handle:
        writer = csv.writer(file_handle)
        writer.writerow(
            [
                "Instance name",
                "Method",
                "Average objective",
                "Best objective",
                "Improvement (%)",
                "Average running time (in seconds)",
            ]
        )
        for result in all_results:
            writer.writerow(
                [
                    result["instance"],
                    result["method"],
                    f"{result['avg']:.1f}",
                    f"{result['best']:.1f}",
                    f"{result['improvement']:.1f}",
                    f"{result['avg_time']:.3f}",
                ]
            )

    print(f"\n{'=' * 70}")
    print("RESULTS TABLE")
    print(f"{'=' * 70}")
    print(
        f"{'Instance':<12} {'Method':<15} {'Avg obj':>12} {'Best obj':>12} "
        f"{'Improv(%)':>12} {'Avg time(s)':>14}"
    )
    for result in all_results:
        print(
            f"{result['instance']:<12} {result['method']:<15} {result['avg']:>12.1f} "
            f"{result['best']:>12.1f} {result['improvement']:>12.1f} {result['avg_time']:>14.3f}"
        )


if __name__ == "__main__":
    main()


