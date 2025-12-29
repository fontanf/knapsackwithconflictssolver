add_library(KnapsackWithConflictsSolver_greedy)
target_sources(KnapsackWithConflictsSolver_greedy PRIVATE
    greedy.cpp)
target_include_directories(KnapsackWithConflictsSolver_greedy PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(KnapsackWithConflictsSolver_greedy PUBLIC
    KnapsackWithConflictsSolver_knapsack_with_conflicts)
add_library(KnapsackWithConflictsSolver::greedy ALIAS KnapsackWithConflictsSolver_greedy)

add_library(KnapsackWithConflictsSolver_sequential_decomposition)
target_sources(KnapsackWithConflictsSolver_sequential_decomposition PRIVATE
    sequential_decomposition.cpp)
target_include_directories(KnapsackWithConflictsSolver_sequential_decomposition PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(KnapsackWithConflictsSolver_sequential_decomposition PUBLIC
    KnapsackWithConflictsSolver_knapsack_with_conflicts
    SetCoveringSolver_local_search_row_weighting
    KnapsackSolver::dynamic_programming_primal_dual)
add_library(KnapsackWithConflictsSolver::sequential_decomposition ALIAS KnapsackWithConflictsSolver_sequential_decomposition)

add_library(KnapsackWithConflictsSolver_greedy_best)
target_sources(KnapsackWithConflictsSolver_greedy_best PRIVATE
    greedy_best.cpp)
target_include_directories(KnapsackWithConflictsSolver_greedy_best PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(KnapsackWithConflictsSolver_greedy_best PUBLIC
    KnapsackWithConflictsSolver_knapsack_with_conflicts)
add_library(KnapsackWithConflictsSolver::greedy_best ALIAS KnapsackWithConflictsSolver_greedy_best)

add_library(KnapsackWithConflictsSolver_bounds)
target_sources(KnapsackWithConflictsSolver_bounds PRIVATE
    bounds.cpp)
target_include_directories(KnapsackWithConflictsSolver_bounds PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(KnapsackWithConflictsSolver_bounds PUBLIC
    OptimizationTools::graph
    KnapsackWithConflictsSolver_knapsack_with_conflicts
    KnapsackSolver::dynamic_programming_primal_dual
    MultipleChoiceKnapsackSolver::milp
    MultipleChoiceKnapsackSolver::dynamic_programming_bellman)
add_library(KnapsackWithConflictsSolver::bounds ALIAS KnapsackWithConflictsSolver_bounds)

add_library(KnapsackWithConflictsSolver_milp)
target_sources(KnapsackWithConflictsSolver_milp PRIVATE
    milp.cpp)
target_include_directories(KnapsackWithConflictsSolver_milp PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(KnapsackWithConflictsSolver_milp PUBLIC
    KnapsackWithConflictsSolver_knapsack_with_conflicts
    MathOptSolversCMake::milp)
add_library(KnapsackWithConflictsSolver::milp ALIAS KnapsackWithConflictsSolver_milp)

add_library(KnapsackWithConflictsSolver_milp_2)
target_sources(KnapsackWithConflictsSolver_milp_2 PRIVATE
    milp_2.cpp)
target_include_directories(KnapsackWithConflictsSolver_milp_2 PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(KnapsackWithConflictsSolver_milp_2 PUBLIC
    KnapsackWithConflictsSolver_knapsack_with_conflicts
    MathOptSolversCMake::milp)
add_library(KnapsackWithConflictsSolver::milp_2 ALIAS KnapsackWithConflictsSolver_milp_2)

add_library(KnapsackWithConflictsSolver_milp_3)
target_sources(KnapsackWithConflictsSolver_milp_3 PRIVATE
    milp_3.cpp)
target_include_directories(KnapsackWithConflictsSolver_milp_3 PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(KnapsackWithConflictsSolver_milp_3 PUBLIC
    KnapsackWithConflictsSolver_knapsack_with_conflicts
    OptimizationTools::graph
    MathOptSolversCMake::milp)
add_library(KnapsackWithConflictsSolver::milp_3 ALIAS KnapsackWithConflictsSolver_milp_3)

if(KNAPSACKWITHCONFLICTSSOLVER_USE_ORTOOLS)
    list(APPEND CMAKE_PREFIX_PATH "$ENV{ORTOOLSDIR}")
    find_package(ortools CONFIG REQUIRED)
    add_library(KnapsackWithConflictsSolver_cp_sat_ortools)
    target_sources(KnapsackWithConflictsSolver_cp_sat_ortools PRIVATE
        cp_sat_ortools.cpp)
    target_include_directories(KnapsackWithConflictsSolver_cp_sat_ortools PUBLIC
        ${PROJECT_SOURCE_DIR}/include)
    target_compile_definitions(KnapsackWithConflictsSolver_cp_sat_ortools PUBLIC
        ORTOOLS_FOUND=1)
    target_link_libraries(KnapsackWithConflictsSolver_cp_sat_ortools PUBLIC
    KnapsackWithConflictsSolver_knapsack_with_conflicts
        ortools::ortools)
    add_library(KnapsackWithConflictsSolver::cp_sat_ortools ALIAS KnapsackWithConflictsSolver_cp_sat_ortools)
endif()

add_library(KnapsackWithConflictsSolver_lagrangian_relaxation)
target_sources(KnapsackWithConflictsSolver_lagrangian_relaxation PRIVATE
    lagrangian_relaxation.cpp)
target_include_directories(KnapsackWithConflictsSolver_lagrangian_relaxation PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(KnapsackWithConflictsSolver_lagrangian_relaxation PUBLIC
    KnapsackWithConflictsSolver_knapsack_with_conflicts
    OptimizationTools::graph
    MathOptSolversCMake::box_constrained_nlp)
add_library(KnapsackWithConflictsSolver::lagrangian_relaxation ALIAS KnapsackWithConflictsSolver_lagrangian_relaxation)

add_library(KnapsackWithConflictsSolver_column_generation)
target_sources(KnapsackWithConflictsSolver_column_generation PRIVATE
    column_generation.cpp)
target_include_directories(KnapsackWithConflictsSolver_column_generation PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(KnapsackWithConflictsSolver_column_generation PUBLIC
    KnapsackWithConflictsSolver_knapsack_with_conflicts
    ColumnGenerationSolver::columngenerationsolver
    KnapsackSolver::dynamic_programming_primal_dual
    OptimizationTools::graph)
add_library(KnapsackWithConflictsSolver::column_generation ALIAS KnapsackWithConflictsSolver_column_generation)
