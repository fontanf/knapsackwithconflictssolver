#include "knapsackwithconflictssolver/utils.hpp"

using namespace knapsackwithconflictssolver;

std::vector<ItemId> knapsackwithconflictssolver::compute_sorted_items(const Instance& instance)
{
    std::vector<ItemId> sorted_items(instance.number_of_items());
    std::iota(sorted_items.begin(), sorted_items.end(), 0);
    std::sort(
            sorted_items.begin(),
            sorted_items.end(),
            [&instance](ItemId item_1_id, ItemId item_2_id)
            {
                const Item& item_1 = instance.item(item_1_id);
                const Item& item_2 = instance.item(item_2_id);
                return (double)item_1.profit / item_1.weight
                    > (double)item_2.profit / item_2.weight;
            });
    return sorted_items;
}

namespace
{

double conflict_score(
        const Instance& instance,
        ConflictId conflict_id)
{
    const Conflict& conflict = instance.conflict(conflict_id);
    const Item& item_1 = instance.item(conflict.item_1_id);
    const Item& item_2 = instance.item(conflict.item_2_id);
    return (double)(item_1.profit + item_2.profit) / (item_1.weight + item_2.weight);
}

}

std::vector<ConflictId> compute_sorted_conflicts(const Instance& instance)
{
    std::vector<ConflictId> sorted_conflicts(instance.number_of_conflicts());
    std::iota(sorted_conflicts.begin(), sorted_conflicts.end(), 0);
    std::sort(
            sorted_conflicts.begin(),
            sorted_conflicts.end(),
            [&instance](ConflictId conflict_id_1, ConflictId conflict_id_2)
            {
                const Conflict& conflict_1 = instance.conflict(conflict_id_1);
                const Conflict& conflict_2 = instance.conflict(conflict_id_2);
                return conflict_score(instance, conflict_id_1)
                    > conflict_score(instance, conflict_id_2);
            });
    return sorted_conflicts;
}
