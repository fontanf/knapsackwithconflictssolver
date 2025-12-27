find_package(Threads REQUIRED)
add_library(KnapsackWithConflictsSolver_knapsack_with_conflicts)
target_sources(KnapsackWithConflictsSolver_knapsack_with_conflicts PRIVATE
    instance.cpp
    instance_builder.cpp
    solution.cpp
    utils.cpp
    algorithm_formatter.cpp)
target_include_directories(KnapsackWithConflictsSolver_knapsack_with_conflicts PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(KnapsackWithConflictsSolver_knapsack_with_conflicts PUBLIC
    OptimizationTools::utils
    OptimizationTools::containers
    Threads::Threads)
add_library(KnapsackWithConflictsSolver::knapsack_with_conflicts ALIAS KnapsackWithConflictsSolver_knapsack_with_conflicts)

add_subdirectory(algorithms)

if(KNAPSACKWITHCONFLICTSSOLVER_BUILD_MAIN)
    add_executable(KnapsackWithConflictsSolver_main)
    target_sources(KnapsackWithConflictsSolver_main PRIVATE
        main.cpp)
    target_link_libraries(KnapsackWithConflictsSolver_main PUBLIC
        KnapsackWithConflictsSolver_greedy
        KnapsackWithConflictsSolver_sequential_decomposition
        KnapsackWithConflictsSolver_greedy_best
        KnapsackWithConflictsSolver_bounds
        KnapsackWithConflictsSolver_milp
        KnapsackWithConflictsSolver_milp_2
        KnapsackWithConflictsSolver_milp_3
        KnapsackWithConflictsSolver_lagrangian_relaxation
        Boost::program_options)
    set_target_properties(KnapsackWithConflictsSolver_main PROPERTIES OUTPUT_NAME "knapsackwithconflictssolver")
    install(TARGETS KnapsackWithConflictsSolver_main)

    add_executable(KnapsackWithConflictsSolver_mps_writer_main)
    target_sources(KnapsackWithConflictsSolver_mps_writer_main PRIVATE
        mps_writer_main.cpp)
    target_link_libraries(KnapsackWithConflictsSolver_mps_writer_main PUBLIC
        KnapsackWithConflictsSolver_knapsack_with_conflicts
        KnapsackWithConflictsSolver_milp
        KnapsackWithConflictsSolver_milp_2
        KnapsackWithConflictsSolver_milp_3
        Boost::program_options
        Boost::filesystem)
    set_target_properties(KnapsackWithConflictsSolver_mps_writer_main PROPERTIES OUTPUT_NAME "knapsackwithconflictssolver_mps_writer")
    install(TARGETS KnapsackWithConflictsSolver_mps_writer_main)
endif()
