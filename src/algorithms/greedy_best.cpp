#include "knapsackwithconflictssolver/algorithms/greedy_best.hpp"

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

#include "knapsackwithconflictssolver/algorithms/greedy.hpp"
#include "knapsackwithconflictssolver/algorithms/sequential_decomposition.hpp"

using namespace knapsackwithconflictssolver;

Output knapsackwithconflictssolver::greedy_best(
        const Instance& instance,
        const Parameters& parameters)
{
    Output output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Best greedy");

    if (parameters.timer.needs_to_end()) {
        algorithm_formatter.end();
        return output;
    }

    algorithm_formatter.print_header();

    Parameters greedy_parameters;
    greedy_parameters.timer = parameters.timer;
    greedy_parameters.verbosity_level = 0;
    auto greedy_output = greedy(instance, greedy_parameters);
    algorithm_formatter.update_solution(greedy_output.solution, "greedy");

    SequentialDecompositionParameters sequential_decomposition_0_parameters;
    sequential_decomposition_0_parameters.timer = parameters.timer;
    sequential_decomposition_0_parameters.verbosity_level = 0;
    sequential_decomposition_0_parameters.stable_weight_strategy = 0;
    auto sequential_decomposition_0_output = sequential_decomposition(instance, sequential_decomposition_0_parameters);
    algorithm_formatter.update_solution(sequential_decomposition_0_output.solution, "seq. dec. 0");

    SequentialDecompositionParameters sequential_decomposition_1_parameters;
    sequential_decomposition_1_parameters.timer = parameters.timer;
    sequential_decomposition_1_parameters.verbosity_level = 0;
    sequential_decomposition_1_parameters.stable_weight_strategy = 1;
    auto sequential_decomposition_1_output = sequential_decomposition(instance, sequential_decomposition_1_parameters);
    algorithm_formatter.update_solution(sequential_decomposition_1_output.solution, "seq. dec. 1");

    algorithm_formatter.end();
    return output;
}
