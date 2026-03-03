#!/usr/bin/env python3
"""INF273 Assignment 2b – Blind Random Search for Truck-Drone Delivery Problem"""

import csv, random, time, sys
from pathlib import Path
from collections import defaultdict

from FeasibilityCheck import SolutionFeasibility
from CalCulateTotalArrivalTime import CalCulateTotalArrivalTime


def read_instance(path: str) -> dict:
    with open(path) as fh:
        lines = [ln.strip() for ln in fh if ln.strip() and not ln.startswith("#")]
    idx = 0
    n = int(lines[idx]); idx += 1
    limit = float(lines[idx]); idx += 1
    T = [list(map(float, lines[idx + i].split())) for i in range(n + 1)]; idx += n + 1
    D = [list(map(float, lines[idx + i].split())) for i in range(n + 1)]
    return {"n": n, "limit": limit, "T": T, "D": D, "name": Path(path).stem}


def initial_solution(data: dict) -> dict:
    return {"route": [0] + list(range(1, data["n"] + 1)) + [0], "trips": []}


def objective(data: dict, sol: dict) -> float:
    """Sum of arrival times / 100 using CalCulateTotalArrivalTime."""
    parts = convert_to_parts_format(sol)
    
    # Create calculator instance
    calc = CalCulateTotalArrivalTime()
    calc.depot_index = 0
    calc.truck_times = data["T"]
    calc.drone_times = data["D"]
    calc.flight_range = data["limit"]
    
    total_time, _, _, feas = calc.calculate_total_waiting_time(parts)
    
    # If infeasible due to hover time, return a large penalty
    if not feas:
        return float('inf')
    
    return total_time


def convert_to_parts_format(sol: dict) -> dict:
    """Convert internal format to FeasibiltyCheck format (part1, part2, part3, part4)."""
    route, trips = sol["route"], sol["trips"]
    
    # Convert trips to positions
    conv = []
    for ln, cust, rn in trips:
        lp = rp = None
        for i, node in enumerate(route):
            if node == ln and lp is None: lp = i
            if node == rn and lp is not None and i > lp: rp = i; break
        if lp is not None and rp is not None:
            conv.append((cust, lp, rp))
    conv.sort(key=lambda x: x[1])
    
    # Assign to drones (greedy: first available drone)
    d1, d2, d1_end, d2_end = [], [], -1, -1
    for cust, lp, rp in conv:
        if lp >= d1_end: 
            d1.append((cust, lp+1, rp+1))  # 1-based indexing
            d1_end = rp
        elif lp >= d2_end: 
            d2.append((cust, lp+1, rp+1))
            d2_end = rp
    
    # Build parts
    p2 = [c for c,_,_ in d1] + [-1] + [c for c,_,_ in d2]
    p3 = [l for _,l,_ in d1] + [-1] + [l for _,l,_ in d2]
    p4 = [r for _,_,r in d1] + [-1] + [r for _,_,r in d2]
    
    return {
        "part1": route,
        "part2": p2,
        "part3": p3,
        "part4": p4
    }


def is_feasible(data: dict, sol: dict) -> bool:
    """Check feasibility using FeasibiltyCheck.py - the ONLY feasibility check."""
    # Convert to parts format
    parts_sol = convert_to_parts_format(sol)
    
    # Create feasibility checker
    checker = SolutionFeasibility(
        n_nodes=data["n"] + 1,  # includes depot
        n_drones=2,
        depot_index=0,
        drone_times=data["D"],
        flight_range=data["limit"]
    )
    
    # Use FeasibiltyCheck as the sole feasibility check
    return checker.is_solution_feasible(parts_sol)


def random_solution(data: dict, rng: random.Random) -> dict:
    n, limit, D = data["n"], data["limit"], data["D"]
    custs = list(range(1, n + 1))
    rng.shuffle(custs)
    n_try = rng.randint(0, max(1, n // 3))
    drone_cands, truck_custs = custs[:n_try], custs[n_try:]
    route = [0] + truck_custs + [0]
    
    intervals = [[], []]
    trips = []
    for cust in drone_cands:
        if len(route) < 3:
            route.insert(rng.randint(1, len(route)-1), cust)
            continue
        assigned = False
        for _ in range(10):
            lp = rng.randint(0, len(route)-2)
            rp = rng.randint(lp+1, len(route)-1)
            ln, rn = route[lp], route[rp]
            if ln == rn or D[ln][cust] + D[cust][rn] > limit: continue
            for d in range(2):
                if not any(not (rp <= el or lp >= er) for el, er in intervals[d]):
                    trips.append((ln, cust, rn))
                    intervals[d].append((lp, rp))
                    assigned = True
                    break
            if assigned: break
        if not assigned:
            route.insert(rng.randint(1, len(route)-1), cust)
    return {"route": route, "trips": trips}


def to_contest_str(sol: dict) -> str:
    """Convert solution to contest format string."""
    parts = convert_to_parts_format(sol)
    route = parts["part1"]
    p2 = parts["part2"]
    p3 = parts["part3"]
    p4 = parts["part4"]
    
    return f"{','.join(map(str,route))}|{','.join(map(str,p2))}|{','.join(map(str,p3))}|{','.join(map(str,p4))}"


def run_brs(path: str, runs: int = 10, iters: int = 10000, seed: int = 12345) -> dict:
    data = read_instance(path)
    s0 = initial_solution(data)
    obj_s0 = objective(data, s0)
    
    print(f"\n{'='*60}\n  {data['name']} | N={data['n']} | s0={obj_s0:.1f}\n{'='*60}")
    
    best_objs, times, best_sol, best_obj = [], [], None, float('inf')
    for r in range(runs):
        rng = random.Random(seed + r)
        cur_best, cur_obj = s0, obj_s0
        t0 = time.perf_counter()
        for _ in range(iters):
            cand = random_solution(data, rng)
            if is_feasible(data, cand):
                cand_obj = objective(data, cand)
                if cand_obj < cur_obj:
                    cur_best, cur_obj = cand, cand_obj
        elapsed = time.perf_counter() - t0
        best_objs.append(cur_obj)
        times.append(elapsed)
        if cur_obj < best_obj:
            best_obj, best_sol = cur_obj, cur_best
        print(f"  Run {r+1:2d}: {cur_obj:.1f} ({elapsed:.2f}s)")
    
    avg, best, improv = sum(best_objs)/runs, min(best_objs), 100*(obj_s0-min(best_objs))/obj_s0
    print(f"  Avg={avg:.1f} Best={best:.1f} Improv={improv:.1f}%")
    return {"name": data["name"], "s0": obj_s0, "avg": avg, "best": best, 
            "improv": improv, "time": sum(times)/runs, "sol": best_sol, "data": data}


def main():
    base = Path(__file__).resolve().parent
    (base / "solutions").mkdir(exist_ok=True)
    
    instances = ["F_10.txt", "F_20.txt", "F_50.txt", "F_100.txt",
                 "R_10.txt", "R_20.txt", "R_50.txt", "R_100.txt"]
    
    print("="*60 + "\n  INF273 Assignment 2b - Blind Random Search\n" + "="*60)
    
    results = []
    for fname in instances:
        path = base / "Data" / fname
        if path.exists():
            res = run_brs(str(path))
            results.append(res)
            # Save solution
            with open(base / "solutions" / f"{res['name']}_best.txt", "w") as f:
                f.write(f"Instance: {res['name']}\nObjective: {res['best']:.1f}\n")
                f.write(f"Solution: {to_contest_str(res['sol'])}\n")
    
    # Save CSV
    with open(base / "results_2b.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Instance", "Average", "Best", "Improvement(%)", "Time(s)"])
        for r in results:
            w.writerow([r["name"], f"{r['avg']:.2f}", f"{r['best']:.2f}", 
                       f"{r['improv']:.2f}", f"{r['time']:.4f}"])
    
    print(f"\n{'='*70}\n  RESULTS\n{'='*70}")
    print(f"  {'Instance':<10} {'s0':>10} {'Avg':>10} {'Best':>10} {'Improv%':>8}")
    for r in results:
        print(f"  {r['name']:<10} {r['s0']:>10.1f} {r['avg']:>10.1f} {r['best']:>10.1f} {r['improv']:>7.1f}%")


if __name__ == "__main__":
    main()
