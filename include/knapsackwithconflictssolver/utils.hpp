#pragma once

#include "knapsackwithconflictssolver/instance.hpp"

namespace knapsackwithconflictssolver
{

std::vector<ItemId> compute_sorted_items(const Instance& instance);

std::vector<ConflictId> compute_sorted_conflicts(const Instance& instance);

std::vector<std::vector<ItemId>> compute_clique_cover(const Instance& instance);

std::vector<std::vector<ItemId>> compute_clique_partition(const Instance& instance);

struct CliquePartitionAndCoverOutput
{
    std::vector<std::vector<ItemId>> clique_partition;
    std::vector<std::vector<ItemId>> clique_cover;
};

CliquePartitionAndCoverOutput compute_clique_partition_and_cover(const Instance& instance);

}
