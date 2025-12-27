#pragma once

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

#include "mathoptsolverscmake/common.hpp"

namespace knapsackwithconflictssolver
{

struct LagrangianRelaxationParameters: Parameters
{
    /** Solver. */
    mathoptsolverscmake::SolverName solver = mathoptsolverscmake::SolverName::Dlib;
};

struct LagrangianRelaxationOutput: Output
{
    LagrangianRelaxationOutput(
            const Instance& instance):
        Output(instance) { }


    /** vector of size instance.alternative_number(). */
    std::vector<std::vector<double>> x;

    /** vector of size instance.number_of_items() */
    std::vector<double> multipliers;
};

LagrangianRelaxationOutput lagrangian_relaxation(
        const Instance& instance,
        std::vector<double>* initial_multipliers = NULL,
        const LagrangianRelaxationParameters& parameters = {});

}
