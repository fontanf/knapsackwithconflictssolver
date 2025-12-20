#pragma once

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

namespace knapsackwithconflictssolver
{

Output fractional_knapsack_bound(
        const Instance& instance,
        const Parameters& parameters = {});

Output binary_knapsack_bound(
        const Instance& instance,
        const Parameters& parameters = {});

Output fractional_multiple_choice_knapsack_bound(
        const Instance& instance,
        const Parameters& parameters = {});

Output binary_multiple_choice_knapsack_bound(
        const Instance& instance,
        const Parameters& parameters = {});

}
