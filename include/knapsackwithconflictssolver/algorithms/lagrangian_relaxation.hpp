#pragma once

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

#include "mathoptsolverscmake/common.hpp"

namespace knapsackwithconflictssolver
{

struct LagrangianRelaxationParameters: Parameters
{
    /** Solver. */
    mathoptsolverscmake::SolverName solver = mathoptsolverscmake::SolverName::Dlib;

    /**
     * Strategy to handle conflicts:
     * - 0: one constraint for each conflict
     * - 1: edge clique partition pre-process
     * - 2: dynamic clique constraints
     */
    int conflicts_strategy = 2;
};

struct LagrangianRelaxationOutput: Output
{
    LagrangianRelaxationOutput(
            const Instance& instance):
        Output(instance),
        x(instance.number_of_items(), 0) { }


    /** Cliques. */
    std::vector<std::vector<ItemId>> cliques;

    /** Vector of size `instance.number_of_items()`. */
    std::vector<double> x;

    /** Vector of size `this->cliques.size()`. */
    std::vector<double> multipliers;


    virtual int format_width() const override { return 31; }

    virtual void format(std::ostream& os) const override
    {
        Output::format(os);
        int width = format_width();
        os
            << std::setw(width) << std::left << "Number of cliques: " << this->cliques.size() << std::endl
            ;
    }

    virtual nlohmann::json to_json() const override
    {
        nlohmann::json json = Output::to_json();
        json.merge_patch({
                {"NumberOfCliques", this->cliques.size()},
                });
        return json;
    }
};

LagrangianRelaxationOutput lagrangian_relaxation(
        const Instance& instance,
        std::vector<double>* initial_multipliers = NULL,
        const LagrangianRelaxationParameters& parameters = {});

}
