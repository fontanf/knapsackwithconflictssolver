#include "knapsackwithconflictssolver/algorithms/greedy.hpp"

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

using namespace knapsackwithconflictssolver;

Output knapsackwithconflictssolver::greedy(
        const Instance& instance,
        const Parameters& parameters)
{
    Output output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Greedy");

    if (parameters.timer.needs_to_end()) {
        algorithm_formatter.end();
        return output;
    }

    algorithm_formatter.print_header();

    // Sort the items by profit/weight.
    std::vector<ItemId> sorted_item_ids(instance.number_of_items(), -1);
    std::iota(sorted_item_ids.begin(), sorted_item_ids.end(), 0);
    std::sort(
            sorted_item_ids.begin(),
            sorted_item_ids.end(),
            [&instance](ItemId item_id_1, ItemId item_id_2)
            {
                const Item& item_1 = instance.item(item_id_1);
                const Item& item_2 = instance.item(item_id_2);
                return (double)item_1.profit / item_1.weight
                    > (double)item_2.profit / item_2.weight;
            });
    Solution solution(instance);
    std::vector<uint8_t> available_items(instance.number_of_items(), 1);
    for (ItemId pos = 0;
            pos < instance.number_of_items();
            ++pos) {
        ItemId item_id = sorted_item_ids[pos];

        // Check conflicts.
        if (available_items[item_id] == 0)
            continue;

        // Check capacity.
        const Item& item = instance.item(item_id);
        if (solution.weight() + item.weight > instance.capacity()) {
            solution.add(item_id);
            ItemId item_best_id = -1;
            Profit profit_best = 0;
            for (ItemId item_id: solution.items()) {
                const Item& item = instance.item(item_id);
                Profit profit = solution.profit() - item.profit;
                Weight weight = solution.weight() - item.weight;
                if (weight > instance.capacity())
                    continue;
                if (item_best_id == -1
                        || profit_best < profit) {
                    item_best_id = item_id;
                    profit_best = profit;
                }
            }
            solution.remove(item_best_id);
            break;
        }

        // Add the item to the solution.
        solution.add(item_id);

        // Update conflicts.
        available_items[item_id] = 0;
        for (const ItemConflict& conflict: item.neighbors)
            available_items[conflict.item_id] = 0;
    }

    // Update output.
    algorithm_formatter.update_solution(solution, "");

    algorithm_formatter.end();
    return output;
}
