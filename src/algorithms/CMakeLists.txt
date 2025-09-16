add_library(KnapsackWithConflictsSolver_greedy)
target_sources(KnapsackWithConflictsSolver_greedy PRIVATE
    greedy.cpp)
target_include_directories(KnapsackWithConflictsSolver_greedy PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(KnapsackWithConflictsSolver_greedy PUBLIC
    KnapsackWithConflictsSolver_knapsack_with_conflicts)
add_library(KnapsackWithConflictsSolver::greedy ALIAS KnapsackWithConflictsSolver_greedy)

add_library(KnapsackWithConflictsSolver_milp)
target_sources(KnapsackWithConflictsSolver_milp PRIVATE
    milp.cpp)
target_include_directories(KnapsackWithConflictsSolver_milp PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(KnapsackWithConflictsSolver_milp PUBLIC
    KnapsackWithConflictsSolver_knapsack_with_conflicts
    MathOptSolversCMake::milp)
add_library(KnapsackWithConflictsSolver::milp ALIAS KnapsackWithConflictsSolver_milp)
