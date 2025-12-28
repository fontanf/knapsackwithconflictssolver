#pragma once

#include "knapsackwithconflictssolver/instance.hpp"

namespace knapsackwithconflictssolver
{

std::vector<ItemId> compute_sorted_items(const Instance& instance);

std::vector<ConflictId> compute_sorted_conflicts(const Instance& instance);

}
