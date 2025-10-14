#include "knapsackwithconflictssolver/algorithms/sequential_decomposition.hpp"

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

#include "setcoveringsolver/instance_builder.hpp"
#include "setcoveringsolver/algorithms/greedy.hpp"
#include "knapsacksolver/instance_builder.hpp"
#include "knapsacksolver/algorithms/dynamic_programming_primal_dual.hpp"

using namespace knapsackwithconflictssolver;

Output knapsackwithconflictssolver::sequential_decomposition(
        const Instance& instance,
        const SequentialDecompositionParameters& parameters)
{
    Output output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Sequential decomposition");

    if (parameters.timer.needs_to_end()) {
        algorithm_formatter.end();
        return output;
    }

    algorithm_formatter.print_header();

    // Build setcovering instance.
    setcoveringsolver::InstanceBuilder setcovering_instance_builder;
    setcovering_instance_builder.add_sets(instance.number_of_items());
    setcovering_instance_builder.add_elements(instance.number_of_conflicts());
    setcoveringsolver::ElementId element_id = 0;
    for (ItemId item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        const Item& item = instance.item(item_id);
        setcoveringsolver::Cost weight = (parameters.stable_weight_strategy == 0)?
            (1e6 * item.profit) / item.weight:
            item.profit;
        setcovering_instance_builder.set_cost(item_id, weight);
        for (const ItemConflict& conflict: item.neighbors) {
            if (item_id < conflict.item_id) {
                setcovering_instance_builder.add_arc(item_id, element_id);
                setcovering_instance_builder.add_arc(conflict.item_id, element_id);
                element_id++;
            }
        }
    }
    setcoveringsolver::Instance setcovering_instance = setcovering_instance_builder.build();

    //setcoveringsolver::setcovering::GreedyParameters setcovering_parameters;
    setcoveringsolver::Parameters setcovering_parameters;
    setcovering_parameters.timer = parameters.timer;
    setcovering_parameters.verbosity_level = 0;
    setcovering_parameters.reduction_parameters.reduce = false;
    setcovering_parameters.new_solution_callback = [&instance, &algorithm_formatter](
            const setcoveringsolver::Output& setcovering_output,
            const std::string&)
    {
        // Build knapsack instance.
        knapsacksolver::InstanceBuilder kp_instance_builder;
        kp_instance_builder.set_capacity(instance.capacity());
        std::vector<ItemId> kp_to_orig;
        for (ItemId item_id = 0;
                item_id < instance.number_of_items();
                ++item_id) {
            if (setcovering_output.solution.contains(item_id))
                continue;
            const Item& item = instance.item(item_id);
            kp_instance_builder.add_item(item.profit, item.weight);
            kp_to_orig.push_back(item_id);
        }
        knapsacksolver::Instance kp_instance = kp_instance_builder.build();

        knapsacksolver::DynamicProgrammingPrimalDualParameters kp_parameters;
        //kp_parameters.timer = parameters.timer;
        kp_parameters.verbosity_level = 0;
        auto kp_output = knapsacksolver::dynamic_programming_primal_dual(
                kp_instance,
                kp_parameters);

        // Retrieve solution.
        Solution solution(instance);
        for (knapsacksolver::ItemId kp_item_id = 0;
                kp_item_id < kp_instance.number_of_items();
                ++kp_item_id) {
            if (!kp_output.solution.contains(kp_item_id))
                continue;
            ItemId item_id = kp_to_orig[kp_item_id];
            solution.add(item_id);
        }

        // Update output.
        algorithm_formatter.update_solution(solution, "");
    };
    setcoveringsolver::greedy_or_greedy_reverse(
            setcovering_instance,
            setcovering_parameters);

    algorithm_formatter.end();
    return output;
}
