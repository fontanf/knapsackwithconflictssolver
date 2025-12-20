#include "knapsackwithconflictssolver/utils.hpp"

#include "optimizationtools/containers/indexed_set.hpp"

using namespace knapsackwithconflictssolver;

std::vector<ItemId> knapsackwithconflictssolver::compute_sorted_items(const Instance& instance)
{
    std::vector<ItemId> sorted_items(instance.number_of_items());
    std::iota(sorted_items.begin(), sorted_items.end(), 0);
    std::sort(
            sorted_items.begin(),
            sorted_items.end(),
            [&instance](ItemId item_id_1, ItemId item_id_2)
            {
                const Item& item_1 = instance.item(item_id_1);
                const Item& item_2 = instance.item(item_id_2);
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

std::vector<std::vector<ItemId>> knapsackwithconflictssolver::compute_clique_cover(const Instance& instance)
{
    std::vector<std::vector<ItemId>> clique_cover;
    std::vector<ItemId> sorted_items = compute_sorted_items(instance);
    optimizationtools::IndexedSet clique_candidates(instance.number_of_items());
    optimizationtools::IndexedSet neighbors_tmp(instance.number_of_items());
    for (ConflictId conflict_id = 0;
            conflict_id < instance.number_of_conflicts();
            ++conflict_id) {
        const Conflict& conflict = instance.conflict(conflict_id);
        clique_cover.push_back({conflict.item_1_id, conflict.item_2_id});
        const Item& item_1 = instance.item(conflict.item_1_id);
        const Item& item_2 = instance.item(conflict.item_2_id);
        clique_candidates.clear();
        for (const ItemConflict& item_conflict: item_1.neighbors)
            clique_candidates.add(item_conflict.item_id);
        neighbors_tmp.clear();
        for (const ItemConflict& item_conflict: item_2.neighbors)
            neighbors_tmp.add(item_conflict.item_id);
        for (auto it = clique_candidates.begin(); it != clique_candidates.end();) {
            bool remove = true;
            if (!neighbors_tmp.contains(*it)) {
                clique_candidates.remove(*it);
            } else {
                it++;
            }
        }
        while (!clique_candidates.empty()) {
            // Find the neighbors with the best profit/weight ratio.
            ItemId item_best_id = -1;
            double ratio_best = 0;
            for (ItemId item_id: clique_candidates) {
                const Item& item = instance.item(item_id);
                double ratio = (double)item.profit / item.weight;
                if (item_best_id == -1
                        || ratio_best < ratio) {
                    item_best_id = item_id;
                }
            }
            clique_cover.back().push_back(item_best_id);
            const Item& item_best = instance.item(item_best_id);
            neighbors_tmp.clear();
            for (const ItemConflict& item_conflict: item_best.neighbors)
                neighbors_tmp.add(item_conflict.item_id);
            for (auto it = clique_candidates.begin(); it != clique_candidates.end();) {
                bool remove = true;
                if (!neighbors_tmp.contains(*it)) {
                    clique_candidates.remove(*it);
                } else {
                    it++;
                }
            }
        }
        std::sort(clique_cover.begin(), clique_cover.end());
    }

    // Remove duplicates.
    std::sort(clique_cover.begin(), clique_cover.end());
    clique_cover.erase(
            unique(clique_cover.begin(), clique_cover.end()),
            clique_cover.end());

    return clique_cover;
}

std::vector<std::vector<ItemId>> knapsackwithconflictssolver::compute_clique_partition(const Instance& instance)
{
    std::vector<std::vector<ItemId>> cliques;

    std::vector<ItemId> sorted_items = compute_sorted_items(instance);

    optimizationtools::IndexedSet item_neighbors(instance.number_of_conflicts());
    for (ItemId item_pos = 0;
            item_pos < instance.number_of_items();
            ++item_pos) {
        ItemId item_id = sorted_items[item_pos];
        const Item& item = instance.item(item_id);

        item_neighbors.clear();
        for (const ItemConflict& item_conflict: item.neighbors)
            item_neighbors.add(item_conflict.item_id);

        ItemId clique_id_best = -1;
        for (ItemId clique_id = 0;
                clique_id < (ItemId)cliques.size();
                ++clique_id) {
            const std::vector<ItemId>& clique = cliques[clique_id];

            bool ok = true;
            for (ItemId clique_item_id: clique) {
                if (!item_neighbors.contains(clique_item_id)) {
                    ok = false;
                    break;
                }
            }
            if (!ok)
                continue;

            if (clique_id_best == -1
                    || cliques[clique_id_best].size() < clique.size()) {
                clique_id_best = clique_id;
            }
        }

        if (clique_id_best != -1) {
            cliques[clique_id_best].push_back(item_id);
        } else {
            cliques.push_back({item_id});
        }
    }
    return cliques;
}

CliquePartitionAndCoverOutput knapsackwithconflictssolver::compute_clique_partition_and_cover(const Instance& instance)
{
    CliquePartitionAndCoverOutput output;

    std::vector<ItemId> sorted_items = compute_sorted_items(instance);
    // TODO

    return output;
}
