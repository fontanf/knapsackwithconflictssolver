# KnapsackWithConflictsSolver

A solver for the knapsack problem with conflicts.

## Problem description

Input:
- a knapsack with capacity $C$
- $n$ items; for each item $j = 1..n$, a weight $w_j$ and a profit $p_j$
- a graph $G$ such that each node corresponds to an item
Problem:
- Select a subset of items such that:
  - the total weight of the selected items does not exceed the knapsack
    capacity
  - if there exists an edge between vertex $j_1$ and vertex $j_2$ in $G$, then item
    $j_1$ and item $j_2$ must not be both selected
Objective:
- Maximize the total profit of the selected items

## Implemented algorithms

- Fast algorithms
  - Greedy `--algorithm greedy`
  - Sequential decomposition (maximum weight independent set then knapsack) `--algorithm sequential-decomposition --stable-weight-strategy 0`
  - Best greedy (greedy + sequential decomposition) `--algorithm greedy-best`

- Simple bounds
  - Fractional knapsack relaxation `--algorithm fractional-knapsack-bound`
  - Integer knapsack relaxation `--algorithm binary-knapsack-bound`
  - Fractional multiple-choice knapsack relaxation `--algorithm fractional-multiple-choice-knapsack-bound`
  - Integer multiple-choice knapsack relaxation `--algorithm binary-multiple-choice-knapsack-bound`

- Mixed integer linear programming
  - Model 1, $|E(G)|$ constraints, `--algorithm milp`
    - Linear relaxation `--algorithm milp-linear-relaxation`
  - Model 2, $n$ constraints `--algorithm milp-2`
    - Linear relaxation `--algorithm milp-2-linear-relaxation`

## Usage (command line)

Compile:
```shell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
cmake --install build --config Release --prefix install
```

Setup Python environment to use the Python scripts:
```shell
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Download data files:
```shell
python scripts/download_data.py
```

Run:

```shell
./install/bin/knapsackwithconflictssolver  --verbosity-level 1  --input data/hifi2006/I1\ -\ I10/10I1 --format hifi2006  --algorithm greedy
```
```
===================================
    KnapsackWithConflictsSolver    
===================================

Instance
--------
Number of items:         1000
Capacity:                2000
Total weight:            50033
Weight ratio:            25.0165
Total profit:            60033
Number of conflicts:     49855
Average # of conflicts:  99.71

Algorithm
---------
Greedy

Parameters
----------
Time limit:                  inf
Messages
    Verbosity level:         1
    Standard output:         1
    File path:               
    # streams:               0
Logger
    Has logger:              0
    Standard error:          0
    File path:               

    Time (s) # items      Weight      Profit       Bound         Gap     Gap (%)                 Comment
    -------- -------      ------      ------       -----         ---     -------                 -------
       0.000       0           0           0       60033       60033      100.00                        
       0.001      42        1073        1493       60033       58540       97.51                        

Final statistics
----------------
Value:                        1493
Bound:                        60033
Absolute optimality gap:      58540
Relative optimality gap (%):  97.513
Time (s):                     0.00097237

Solution
--------
Number of items:      42 / 1000 (4.2%)
Weight:               1073 / 2000 (53.65%)
Number of conflicts:  0
Feasible:             1
Profit:               1493
```
