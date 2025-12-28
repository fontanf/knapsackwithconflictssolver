#pragma once

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

#include "columngenerationsolver/commons.hpp"

namespace knapsackwithconflictssolver
{

struct ColumnGenerationParameters: Parameters
{
    /** Linear programming solver. */
    columngenerationsolver::SolverName linear_programming_solver = columngenerationsolver::SolverName::Highs;

    /**
     * Strategy to handle conflicts:
     * - 0: one constraint for each conflict
     * - 1: edge clique partition pre-process
     * - 2: dynamic clique constraints
     */
    int conflicts_strategy = 2;
};

struct ColumnGenerationOutput: Output
{
    ColumnGenerationOutput(
            const Instance& instance):
        Output(instance),
        x(instance.number_of_items(), 0) { }


    /** Cliques. */
    std::vector<std::vector<ItemId>> cliques;

    /** Vector of size `instance.number_of_items()`. */
    std::vector<double> x;

    /** Columns. */
    std::vector<std::vector<std::pair<ItemId, double>>> columns;


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

ColumnGenerationOutput column_generation(
        const Instance& instance,
        const ColumnGenerationParameters& parameters = {});

}
