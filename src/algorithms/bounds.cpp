#include "knapsackwithconflictssolver/algorithms/bounds.hpp"

#include "knapsackwithconflictssolver/utils.hpp"

#include "knapsacksolver/instance_builder.hpp"
#include "knapsacksolver/algorithms/dynamic_programming_primal_dual.hpp"

#include "multiplechoiceknapsacksolver/instance_builder.hpp"
#include "multiplechoiceknapsacksolver/algorithms/milp.hpp"
#include "multiplechoiceknapsacksolver/algorithms/dynamic_programming_bellman.hpp"

using namespace knapsackwithconflictssolver;

Output knapsackwithconflictssolver::fractional_knapsack_bound(
        const Instance& instance,
        const Parameters& parameters)
{
    Output output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Fractional knapsack bound");
    algorithm_formatter.print_header();

    std::vector<ItemId> sorted_items = compute_sorted_items(instance);
    Profit profit = 0;
    Weight weight = 0;
    for (ItemId item_id: sorted_items) {
        const Item& item = instance.item(item_id);
        if (weight + item.weight > instance.capacity()) {
            profit += std::ceil((double)(item.profit * (instance.capacity() - weight)) / item.weight);
            break;
        }
        profit += item.profit;
        weight += item.weight;
    }

    // Update output.
    algorithm_formatter.update_bound(profit, "");

    algorithm_formatter.end();
    return output;
}

Output knapsackwithconflictssolver::binary_knapsack_bound(
        const Instance& instance,
        const Parameters& parameters)
{
    Output output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Binary knapasck bound");
    algorithm_formatter.print_header();

    // Build knapsack instance.
    knapsacksolver::InstanceBuilder kp_instance_builder;
    kp_instance_builder.set_capacity(instance.capacity());
    for (ItemId item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        const Item& item = instance.item(item_id);
        kp_instance_builder.add_item(item.profit, item.weight);
    }
    knapsacksolver::Instance kp_instance = kp_instance_builder.build();

    knapsacksolver::DynamicProgrammingPrimalDualParameters kp_parameters;
    //kp_parameters.timer = parameters.timer;
    kp_parameters.verbosity_level = 0;
    auto kp_output = knapsacksolver::dynamic_programming_primal_dual(
            kp_instance,
            kp_parameters);

    // Update output.
    algorithm_formatter.update_bound(kp_output.bound, "");

    algorithm_formatter.end();
    return output;
}

Output knapsackwithconflictssolver::fractional_multiple_choice_knapsack_bound(
        const Instance& instance,
        const Parameters& parameters)
{
    Output output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Fractional multiple-choice knapasck bound");
    algorithm_formatter.print_header();

    std::vector<std::vector<ItemId>> clique_partition = compute_clique_partition(instance);

    // Build knapsack instance.
    multiplechoiceknapsacksolver::InstanceBuilder mckp_instance_builder;
    mckp_instance_builder.set_capacity(instance.capacity());
    for (multiplechoiceknapsacksolver::GroupId group_id = 0;
            group_id < clique_partition.size();
            ++group_id) {
        for (ItemId item_id: clique_partition[group_id]) {
            const Item& item = instance.item(item_id);
            mckp_instance_builder.add_item(
                    group_id,
                    item.profit,
                    item.weight);
        }
    }
    multiplechoiceknapsacksolver::Instance mckp_instance = mckp_instance_builder.build();

    multiplechoiceknapsacksolver::MilpParameters mckp_parameters;
    //mckp_parameters.timer = parameters.timer;
    mckp_parameters.verbosity_level = 0;
    auto mckp_output = multiplechoiceknapsacksolver::milp_linear_relaxation(
            mckp_instance,
            mckp_parameters);

    // Update output.
    algorithm_formatter.update_bound(mckp_output.bound, "");

    algorithm_formatter.end();
    return output;
}

Output knapsackwithconflictssolver::binary_multiple_choice_knapsack_bound(
        const Instance& instance,
        const Parameters& parameters)
{
    Output output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Binary multiple-choice knapsack bound");
    algorithm_formatter.print_header();

    std::vector<std::vector<ItemId>> clique_partition = compute_clique_partition(instance);

    // Build knapsack instance.
    multiplechoiceknapsacksolver::InstanceBuilder mckp_instance_builder;
    mckp_instance_builder.set_capacity(instance.capacity());
    for (multiplechoiceknapsacksolver::GroupId group_id = 0;
            group_id < clique_partition.size();
            ++group_id) {
        for (ItemId item_id: clique_partition[group_id]) {
            const Item& item = instance.item(item_id);
            mckp_instance_builder.add_item(
                    group_id,
                    item.profit,
                    item.weight);
        }
    }
    multiplechoiceknapsacksolver::Instance mckp_instance = mckp_instance_builder.build();

    multiplechoiceknapsacksolver::Parameters mckp_parameters;
    //mckp_parameters.timer = parameters.timer;
    mckp_parameters.verbosity_level = 0;
    auto mckp_output = multiplechoiceknapsacksolver::dynamic_programming_bellman_array(
            mckp_instance,
            mckp_parameters);

    // Update output.
    algorithm_formatter.update_bound(mckp_output.bound, "");

    algorithm_formatter.end();
    return output;
}
