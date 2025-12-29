#pragma once

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

namespace knapsackwithconflictssolver
{

struct CpSatOrtoolsParameters: Parameters
{
    /** Initial solution. */
    const Solution* initial_solution = NULL;

    /**
     * Strategy to handle conflicts:
     * - 0: one constraint for each conflict
     * - 1: edge clique partition pre-process
     */
    int conflicts_strategy = 0;
};

Output cp_sat_ortools(
        const Instance& instance,
        const CpSatOrtoolsParameters& parameters = {});

}
