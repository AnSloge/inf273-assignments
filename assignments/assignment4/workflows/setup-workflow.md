# Assignment 4 — Detailed Implementation Guide for Three Smart Operators in Modified Simulated Annealing

This document explains, in implementation-oriented detail, how to integrate **three problem-dependent operators** into the **modified Simulated Annealing (SA)** required in Assignment 4.

The assignment requires you to define **three operators** and plug them into the given SA variant, where an operator is selected using **roulette-wheel probabilities** \(P_1, P_2, P_3\). You must run two versions: one with **equal weights** and one with **tuned weights**. The modified SA first performs **100 preliminary operator applications** to estimate the initial temperature, then runs **9900 search iterations** with exponential cooling down to \(T_f = 0.1\). fileciteturn1file1 fileciteturn1file2

The course material also makes the modeling goal clear for the truck-and-drones multimodal system: there is **one truck with multiple drones**, drones can be launched and received at customer locations, each drone serves one customer per trip, drone flight time is limited, and the objective is to **minimize total waiting time**. fileciteturn1file3

Because of that, the best operators are not generic random moves. They should target the actual structure that creates waiting time: **truck order**, **drone assignment**, **launch/recovery synchronization**, and **feasibility with respect to drone endurance**.

---

## 1. Design Goal of the Three Operators

A good operator set should not contain three nearly identical moves. The cleanest design is:

- **OP1:** a small, focused, improvement-oriented move
- **OP2:** a structural truck-route move
- **OP3:** a synchronization-aware drone reassignment or repair move

That gives the SA a healthy balance between:

- **intensification**: improving promising solutions using small smart changes
- **diversification**: changing larger structural patterns to escape local optima
- **repair/problem-awareness**: exploiting the details of the truck-and-drones system

The three operators proposed in this guide are:

1. **Critical-customer relocate**
2. **Truck segment 2-opt / order repair**
3. **Drone reassignment with best feasible anchor pair**

These are strong because they attack the three main causes of bad solutions:

- a customer placed in the wrong order,
- a truck route with poor segment structure,
- a drone customer attached to a poor launch/recovery context.

---

## 2. Assumed Solution Representation

The exact implementation depends on your codebase, but the operator logic becomes much easier if the solution is represented in a structured way.

A practical representation is:

```text
Solution:
    truck_route: [0, ..., 0]
    drone_trips: list of trips

DroneTrip:
    launch_node
    drone_customer
    recovery_node
    drone_id (optional)
```

Where:

- `truck_route` is the sequence visited by the truck, usually including depot at start and end
- a `drone_customer` is not served directly by the truck
- `launch_node` and `recovery_node` are truck-visited nodes where the drone departs and rejoins
- each drone trip must satisfy the drone flight-time limit
- the evaluator computes the total waiting-time objective from the combined truck and drone schedule

If your code uses another encoding, do **not** rewrite the whole solver. Instead, implement a thin adaptation layer so that each operator can:

1. inspect the current solution,
2. create a candidate copy,
3. modify the candidate,
4. return the candidate.

---

## 3. Core Support Functions You Should Have Before Writing the Operators

Before implementing the operators themselves, build or verify the following utilities.

### 3.1 Deep-copy helper

Each operator must modify a candidate solution, not the incumbent directly.

```python
candidate = copy_solution(current_solution)
```

This must copy:

- truck route
- drone assignments
- cached metadata if any

If you accidentally mutate the incumbent in place, the SA will behave incorrectly.

### 3.2 Feasibility checker

You need one function such as:

```python
is_feasible(solution) -> bool
```

It should verify at least:

- each customer is served exactly once
- no customer is both truck-served and drone-served
- each drone trip uses a valid launch and recovery order along the truck route
- drone flight time is within the limit
- any other constraints from earlier assignments are respected

The modified SA pseudocode accepts a new solution only if it is feasible. fileciteturn1file2

### 3.3 Evaluation function

You already have this from earlier assignments, but all operators depend on it conceptually:

```python
evaluate(solution) -> float
```

This should return the total waiting-time objective.

### 3.4 Customer role lookup

These helper queries are very useful:

```python
get_truck_customers(solution)
get_drone_customers(solution)
find_customer_position_in_truck_route(solution, customer)
find_drone_trip_of_customer(solution, customer)
```

### 3.5 Optional: contribution or “criticality” score

To make operators smart, define a rough score indicating which customers or trips are likely harmful.

Examples:

- large waiting-time contribution
- long truck detour cost
- drone trip with little endurance slack
- launch/recovery pair causing truck idling or drone idling

You do **not** need a mathematically exact decomposition. A heuristic score is enough.

A typical helper might be:

```python
compute_customer_criticality(solution, customer) -> float
```

---

## 4. Operator Integration Architecture

All three operators should expose the same interface.

```python
def op1_critical_customer_relocate(solution, instance, rng):
    return new_solution


def op2_truck_2opt_repair(solution, instance, rng):
    return new_solution


def op3_drone_reassign_best_anchor(solution, instance, rng):
    return new_solution
```

Then define a roulette-wheel selector:

```python
def select_operator(rng, p1, p2, p3):
    r = rng.random()
    if r < p1:
        return 1
    elif r < p1 + p2:
        return 2
    else:
        return 3
```

Important:

- ensure `p1 + p2 + p3 = 1.0`
- for equal weights use `1/3, 1/3, 1/3`
- for tuned weights use trial-and-error values such as `0.5, 0.3, 0.2` or similar

Then define a dispatcher:

```python
def apply_selected_operator(solution, instance, rng, p1, p2, p3):
    op_id = select_operator(rng, p1, p2, p3)
    if op_id == 1:
        return op1_critical_customer_relocate(solution, instance, rng), op_id
    elif op_id == 2:
        return op2_truck_2opt_repair(solution, instance, rng), op_id
    else:
        return op3_drone_reassign_best_anchor(solution, instance, rng), op_id
```

Return `op_id` as well if you later want operator statistics.

---

## 5. Operator 1 — Critical-Customer Relocate

## 5.1 Main idea

This operator targets a **customer that appears to hurt the solution the most**, and relocates it to a better position.

This is much smarter than random insertion because it does not waste effort on already well-placed customers.

Depending on how the chosen customer is currently served, the move does one of two things:

- if it is a **truck customer**, remove it from its current truck position and reinsert it at a different truck position
- if it is a **drone customer**, either:
  - change its launch/recovery anchors, or
  - convert it to truck service and reinsert it into the truck route, if your representation supports this cleanly

The simplest robust version is to restrict OP1 to **truck-customer relocation only**, while letting OP3 handle drone reassignment.

That is usually easier to debug.

---

## 5.2 Why this operator is beneficial

It directly supports the assignment’s request for **problem-dependent operators** because it focuses on the parts of the solution that are likely responsible for large waiting time. fileciteturn1file1

It is also a strong **intensification** operator:

- small move,
- objective-aware target selection,
- often feasible,
- often improves the incumbent without destroying solution structure.

---

## 5.3 Implementation logic

### Step A — choose a critical truck customer

Build the set of truck-served customers excluding depot.

For each truck customer `c`, compute a criticality score. Examples of reasonable scores:

```text
criticality(c) = local detour cost of c
```

or

```text
criticality(c) = increase in local arrival/waiting time caused by current position of c
```

A simple local detour estimate is:

```text
detour(c) = dist(prev(c), c) + dist(c, next(c)) - dist(prev(c), next(c))
```

If the problem uses travel times rather than distances, use travel times.

Then select:

- either the maximum-criticality customer,
- or one customer from the top-k most critical, chosen randomly.

Using top-k adds some diversity.

### Step B — remove the customer from the route

If the current truck route is:

```text
[0, a, b, c, d, e, 0]
```

and `c` is selected, remove it:

```text
[0, a, b, d, e, 0]
```

### Step C — evaluate candidate insertion positions

Try reinserting `c` into every legal truck position.

For each position `i`:

1. build a temporary candidate,
2. insert `c`,
3. verify feasibility,
4. compute objective value.

Choose the best feasible insertion position.

This is a **best-insertion relocate** move.

### Step D — return the best candidate

If no feasible reinsertion improves anything, you still have options:

- return the best feasible candidate even if worse,
- or return the original solution unchanged.

Returning the original solution is the safest implementation.

---

## 5.4 Pseudocode

```python
def op1_critical_customer_relocate(solution, instance, rng):
    candidate_base = copy_solution(solution)
    truck_customers = get_truck_customers(candidate_base)

    if len(truck_customers) <= 1:
        return candidate_base

    scored = []
    for c in truck_customers:
        score = compute_truck_customer_criticality(candidate_base, c, instance)
        scored.append((c, score))

    scored.sort(key=lambda x: x[1], reverse=True)
    top_k = scored[:min(5, len(scored))]
    chosen_customer = rng.choice(top_k)[0]

    route = candidate_base.truck_route[:]
    old_pos = route.index(chosen_customer)
    route.pop(old_pos)

    best_solution = solution
    best_value = evaluate(solution)

    for insert_pos in range(1, len(route)):  # skip depot at index 0
        temp = copy_solution(candidate_base)
        temp.truck_route = route[:]
        temp.truck_route.insert(insert_pos, chosen_customer)

        if is_feasible(temp):
            val = evaluate(temp)
            if val < best_value:
                best_solution = temp
                best_value = val

    return copy_solution(best_solution)
```

---

## 5.5 Important implementation notes

- Never insert before the first depot or after the final depot in an invalid way.
- If drone trips reference truck-route indices rather than node IDs, route modifications may invalidate them. In that case, update anchor references carefully.
- If route relocation can break drone launch/recovery ordering, either:
  - repair affected drone trips after reinsertion, or
  - skip reinsertion positions that would break them.

The second option is much easier and safer.

---

## 6. Operator 2 — Truck Segment 2-Opt / Order Repair

## 6.1 Main idea

This operator performs a **2-opt-style reversal on the truck route**, but only on the truck sequence. It is a structural move that changes the order of a route segment.

The classical 2-opt idea is well known from routing problems: choose two cut points and reverse the segment between them. In this assignment, that can still be very useful because truck ordering affects both truck travel and drone synchronization. The local-search material explicitly presents move operators such as swap, 2-opt, 3-opt, and related neighborhood moves. fileciteturn1file4

---

## 6.2 Why this operator is beneficial

This operator is valuable because OP1 only makes small local relocations. Sometimes the route contains a broader structural defect that one relocate cannot repair.

A 2-opt-style move can:

- remove inefficient segment ordering,
- reduce truck travel time,
- indirectly reduce waiting time,
- create a new synchronization pattern for multiple drone trips at once.

This makes OP2 a good **medium-scale diversification + repair** operator.

---

## 6.3 Implementation logic

### Step A — choose two cut positions

From the internal truck route positions, choose indices `i` and `j` such that:

```text
i < j
1 <= i < j < len(route) - 1
```

This keeps the depots fixed.

Example:

```text
route = [0, a, b, c, d, e, f, 0]
i = 2, j = 5
segment = [b, c, d, e]
reversed = [e, d, c, b]
new route = [0, a, e, d, c, b, f, 0]
```

### Step B — build the reversed candidate

Create a copy and replace the segment with its reverse.

### Step C — validate the drone structure

This is the most important practical issue.

If your drone trips are defined by **node IDs** and feasibility is checked using order in the new truck route, then after a 2-opt move some drone trips may become invalid because:

- recovery could now appear before launch,
- timing may become infeasible,
- waiting time may increase sharply.

You have three implementation choices:

#### Option 1 — strict feasibility only

After reversal, run `is_feasible(candidate)`. If not feasible, return the original solution.

This is the simplest version.

#### Option 2 — route-only safe segment selection

Before applying 2-opt, ensure the reversed segment does not contain launch/recovery nodes involved in drone trips, or does not invert required anchor order.

This is cleaner but requires more bookkeeping.

#### Option 3 — post-move repair

After the 2-opt move, repair invalid drone trips by reassigning or removing them.

This is powerful but more complex. For assignment work, Option 1 or 2 is usually enough.

---

## 6.4 Strong version of OP2: sampled-best 2-opt

Do not perform just one random 2-opt move. A stronger and still simple approach is:

1. sample `m` random `(i, j)` pairs, for example `m = 10`
2. evaluate each feasible candidate
3. return the best sampled candidate

This keeps the operator smart without becoming a full local search inside one move.

---

## 6.5 Pseudocode

```python
def op2_truck_2opt_repair(solution, instance, rng, samples=10):
    base = copy_solution(solution)
    route = base.truck_route

    if len(route) <= 5:
        return base

    best_solution = solution
    best_value = evaluate(solution)

    for _ in range(samples):
        i = rng.randint(1, len(route) - 3)
        j = rng.randint(i + 1, len(route) - 2)

        temp = copy_solution(base)
        temp.truck_route = (
            route[:i] +
            list(reversed(route[i:j+1])) +
            route[j+1:]
        )

        if is_feasible(temp):
            val = evaluate(temp)
            if val < best_value:
                best_solution = temp
                best_value = val

    return copy_solution(best_solution)
```

---

## 6.6 Important implementation notes

- Keep depots fixed.
- If the truck route includes only truck customers and drone-served customers are absent from the truck sequence, the move is easier.
- If launch/recovery nodes are regular truck customers, the move may still be safe as long as feasibility is rechecked afterward.
- This operator may frequently return the original solution if feasibility is fragile. That is acceptable, but if it happens too often, you should make the move more constraint-aware.

---

## 7. Operator 3 — Drone Reassignment with Best Feasible Anchor Pair

## 7.1 Main idea

This is the most problem-dependent operator.

Choose a drone-served customer and try to assign it to a **better launch/recovery pair** along the current truck route.

This operator explicitly targets the multimodal synchronization structure. That makes it exactly the type of creative, problem-aware move the assignment encourages. fileciteturn1file1

---

## 7.2 Why this operator is beneficial

A drone assignment can be poor even when the truck route itself looks reasonable. For example:

- the launch node may be too early,
- the recovery node may be too late,
- the drone may barely satisfy the endurance limit,
- the truck may wait too long for the drone,
- or the drone may wait too long for the truck.

Reassigning the customer to a better anchor pair can significantly reduce waiting time without changing the whole truck route.

This operator is excellent for **repair and synchronization improvement**.

---

## 7.3 Implementation logic

### Step A — choose a drone customer to reconsider

Get all drone-served customers.

If none exist, return the original solution.

Choose one customer either:

- uniformly at random, or better,
- based on “bad trip” score, such as:
  - small endurance slack,
  - large trip duration,
  - large contribution to waiting,
  - launch/recovery pair far apart in truck time.

### Step B — remove its current drone trip

Temporarily delete the current trip for that drone customer.

### Step C — enumerate candidate anchor pairs

Let the current truck route be:

```text
[0, v1, v2, v3, ..., vk, 0]
```

For the chosen drone customer `u`, consider pairs `(launch, recovery)` such that:

- both nodes are on the truck route,
- `launch` appears before `recovery`,
- the drone trip `launch -> u -> recovery` is within the flight-time limit,
- any drone availability constraints are satisfied,
- any launch/recovery concurrency constraints are satisfied if modeled.

For each feasible pair:

1. create a candidate solution,
2. assign `u` to that pair,
3. check feasibility,
4. evaluate objective.

Pick the best feasible reassignment.

### Step D — optional restricted search for speed

Full enumeration may be expensive. A practical compromise is:

- consider only launch nodes within a window around the current launch,
- consider only recovery nodes within a window around the current recovery,
- or randomly sample a bounded number of feasible pairs.

Since the assignment says operator quality is more important than runtime, a somewhat more expensive but smarter operator is perfectly reasonable. fileciteturn1file1

---

## 7.4 Pseudocode

```python
def op3_drone_reassign_best_anchor(solution, instance, rng):
    base = copy_solution(solution)
    drone_customers = get_drone_customers(base)

    if not drone_customers:
        return base

    scored = []
    for c in drone_customers:
        score = compute_drone_trip_criticality(base, c, instance)
        scored.append((c, score))

    scored.sort(key=lambda x: x[1], reverse=True)
    chosen_customer = rng.choice(scored[:min(5, len(scored))])[0]

    base = remove_drone_customer_assignment(base, chosen_customer)

    truck_nodes = base.truck_route
    best_solution = solution
    best_value = evaluate(solution)

    for i in range(len(truck_nodes) - 1):
        launch = truck_nodes[i]
        for j in range(i + 1, len(truck_nodes)):
            recovery = truck_nodes[j]

            if not drone_trip_within_limit(launch, chosen_customer, recovery, instance):
                continue

            temp = copy_solution(base)
            add_drone_assignment(temp, chosen_customer, launch, recovery)

            if is_feasible(temp):
                val = evaluate(temp)
                if val < best_value:
                    best_solution = temp
                    best_value = val

    return copy_solution(best_solution)
```

---

## 7.5 Optional stronger variation

A very strong variation is to allow OP3 to try both:

1. **drone-to-drone reassignment** using a new anchor pair,
2. **drone-to-truck conversion** by inserting the customer into the truck route.

Then return the best feasible option.

This makes OP3 even more robust because sometimes the best “repair” is to stop serving that customer by drone at all.

However, only use this if your code structure is clean enough. Otherwise, keep OP3 focused.

---

## 8. How to Plug the Operators into the Modified SA

The assignment’s modified SA works as follows:

1. start from an initial solution `s0`
2. repeatedly select one of `OP1`, `OP2`, `OP3` using roulette-wheel selection
3. do 100 preliminary moves to estimate deterioration and compute `T0`
4. run 9900 iterations with exponential cooling from `T0` to `Tf = 0.1` fileciteturn1file2

---

## 8.1 Preliminary phase for temperature estimation

You need to follow the pseudocode exactly enough.

For `w = 1..100`:

1. select an operator with probabilities `P1, P2, P3`
2. apply it to the incumbent
3. compute `ΔE = f(new) - f(incumbent)`
4. if feasible and improving, accept it
5. else if feasible, accept with probability according to the assignment’s initial sampling procedure
6. store deterioration values used for `DeltaAvg`

Then compute:

```text
DeltaAvg = mean(Δw)
T0 = - DeltaAvg / ln(0.8)
alpha = (Tf / T0)^(1/9900)
```

This is taken directly from the assignment’s modified SA description. fileciteturn1file2

### Important practical note

If your 100 preliminary moves produce many unchanged or invalid candidates, `DeltaAvg` may become unstable or meaningless.

That is one reason the operators must be:

- reasonably feasible,
- not too destructive,
- and capable of producing varied but sensible neighbors.

---

## 8.2 Main SA loop

At each iteration:

1. choose operator by roulette wheel
2. generate `new_solution`
3. compute `ΔE`
4. if feasible and `ΔE < 0`, accept
5. else if feasible, accept with probability `exp(-ΔE / T)`
6. update best solution if needed
7. cool temperature by `T = alpha * T`

This matches the simulated annealing acceptance logic in the lecture material: improving moves are always accepted, while worse moves are accepted with probability depending on deterioration and temperature. fileciteturn1file4

---

## 8.3 Clean implementation skeleton

```python
def simulated_annealing_modified(s0, instance, p1, p2, p3, rng):
    Tf = 0.1
    incumbent = copy_solution(s0)
    best_solution = copy_solution(s0)

    delta_samples = []

    # Preliminary 100 iterations
    for _ in range(100):
        new_solution, op_id = apply_selected_operator(incumbent, instance, rng, p1, p2, p3)

        if not is_feasible(new_solution):
            continue

        delta_e = evaluate(new_solution) - evaluate(incumbent)

        if delta_e < 0:
            incumbent = new_solution
            if evaluate(incumbent) < evaluate(best_solution):
                best_solution = copy_solution(incumbent)
        else:
            # Use the assignment's preliminary acceptance rule as implemented in your baseline
            if rng.random() < 0.8:
                incumbent = new_solution
            delta_samples.append(delta_e)

    if not delta_samples:
        delta_avg = 1.0
    else:
        delta_avg = sum(delta_samples) / len(delta_samples)
        if delta_avg <= 0:
            delta_avg = 1.0

    T0 = -delta_avg / math.log(0.8)
    alpha = (Tf / T0) ** (1.0 / 9900.0)
    T = T0

    # Main SA phase
    for _ in range(9900):
        new_solution, op_id = apply_selected_operator(incumbent, instance, rng, p1, p2, p3)

        if is_feasible(new_solution):
            delta_e = evaluate(new_solution) - evaluate(incumbent)

            if delta_e < 0:
                incumbent = new_solution
                if evaluate(incumbent) < evaluate(best_solution):
                    best_solution = copy_solution(incumbent)
            else:
                if rng.random() < math.exp(-delta_e / T):
                    incumbent = new_solution

        T *= alpha

    return best_solution
```

Adjust the preliminary-phase acceptance to match your exact baseline text file implementation.

---

## 9. Equal Weights vs Tuned Weights

The assignment explicitly requires two versions:

- **equal weights:** `P1 = P2 = P3 = 1/3`
- **tuned weights:** chosen by trial and error fileciteturn1file1

A good tuning strategy is to assign more weight to the most reliable operator.

A sensible expectation for these three operators is:

- **OP1** is likely the most reliable improver
- **OP2** is useful but more disruptive
- **OP3** is powerful but may be more expensive and more constrained

So good trial values could look like:

```text
(0.34, 0.33, 0.33)
(0.50, 0.30, 0.20)
(0.45, 0.20, 0.35)
(0.60, 0.25, 0.15)
```

You should test a few combinations over a subset of instances and compare:

- average objective,
- best objective,
- stability over 10 runs.

If OP3 proves especially strong on your instances, shift more weight toward it.

---

## 10. Recommended Implementation Order

To reduce debugging pain, implement in this order.

### Phase 1 — infrastructure

1. verify `copy_solution`
2. verify `is_feasible`
3. verify `evaluate`
4. implement operator selection by roulette wheel
5. run SA with dummy operators that return unchanged solution

### Phase 2 — implement OP1 first

OP1 is the easiest to debug because it only changes one truck-customer position.

Test cases:

- route stays valid
- no customer duplication
- no customer loss
- objective can be computed afterward

### Phase 3 — implement OP2

Start with a strict version:

- sample one or a few 2-opt moves
- accept only if feasible
- otherwise return original solution

### Phase 4 — implement OP3 last

This is the most complex one because it depends on drone-anchor consistency.

Start with:

- choose one drone customer,
- enumerate candidate launch/recovery pairs,
- keep the best feasible one.

---

## 11. Debugging Checklist

When an operator seems to “not work,” it is usually one of these issues.

### 11.1 Solution corruption

Symptoms:

- missing customers
- duplicated customers
- inconsistent drone assignments

Check:

- every operator removes exactly once and inserts exactly once
- customer-service mode is updated correctly

### 11.2 Route-index vs node-ID mismatch

If drone trips store route indices and you reorder the truck route, you must update those references.

This is a common source of silent bugs.

### 11.3 Too many infeasible candidates

If an operator often returns infeasible solutions, SA gets weak because it explores too little.

Fixes:

- make the operator constraint-aware before constructing candidates
- use sampled-best among only legal move positions
- restrict move scope to safer choices

### 11.4 Objective recomputation errors

If `ΔE` values look strange, confirm that the evaluator recomputes the full solution state consistently after each move.

### 11.5 Preliminary temperature estimation becomes unstable

If `T0` is zero, negative, or absurdly large, inspect the 100 preliminary `ΔE` samples.

Possible causes:

- unchanged neighbors returned too often,
- invalid moves skipped too often,
- incorrect `ΔE` sign,
- faulty acceptance bookkeeping.

---

## 12. How to Explain These Operators in the Report

Your PDF only needs a **brief explanation**, but it should still sound deliberate and problem-aware. fileciteturn1file1

A concise explanation could follow this structure.

### Operator 1 — Critical-customer relocate

This operator identifies a truck customer that appears to contribute strongly to waiting time or local detour, removes it from its current truck position, and reinserts it in the best feasible position. The purpose is to intensify the search around promising solutions while focusing on harmful route positions rather than making a purely random move.

### Operator 2 — Truck segment 2-opt repair

This operator selects a truck-route segment and reverses it using a 2-opt style move. It is intended to correct broader ordering mistakes in the truck route that cannot be fixed by a single relocate move. Because truck order affects synchronization with drone trips, this move can also improve the overall waiting-time objective.

### Operator 3 — Drone reassignment with best anchor pair

This operator selects a drone-served customer and tries to reassign it to a better launch/recovery pair on the current truck route, subject to feasibility and drone endurance. It is a problem-dependent move designed specifically for the truck-and-drones structure, with the goal of reducing synchronization inefficiency and total waiting time.

---

## 13. Recommended Final Practical Choices

If you want the safest strong implementation, use these exact versions.

### OP1

- target only truck customers
- choose from top-5 by local detour score
- best feasible reinsertion over all truck positions

### OP2

- sampled-best 2-opt on truck route
- sample 10 random `(i, j)` pairs
- keep best feasible candidate

### OP3

- choose from top-5 worst drone trips
- fully enumerate feasible launch/recovery pairs
- keep best feasible reassignment

This combination is strong, defensible, and well aligned with the assignment.

---

## 14. Final Advice Before Coding

The biggest implementation risk is not the SA formula. It is **representation consistency** after an operator move.

So before benchmarking anything, verify after every operator call that:

```python
assert all_customers_served_exactly_once(candidate)
assert is_feasible(candidate)
```

at least during debugging.

Once those invariants hold, the SA framework will usually work as expected.

If you later want to improve beyond this baseline, the next natural step is to add **operator statistics** such as:

- number of times each operator selected,
- number of feasible outputs,
- average `ΔE`,
- number of accepted moves,
- number of improving moves.

That will help you justify tuned weights more convincingly.

---

## 15. One Important Note About Your Repo TXT File

This guide is written to fit the assignment specification and the modified SA pseudocode from the assignment PDF. I was **not able to inspect the TXT file from your repository in this session**, so when you integrate this into code, align the function names and acceptance logic with your exact baseline implementation. The operator designs above are still directly compatible with the assignment requirements. fileciteturn1file1turn1file2
