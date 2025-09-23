#pragma once

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

namespace knapsackwithconflictssolver
{

struct TreeSearchParameters: Parameters
{

    virtual int format_width() const override { return 37; }

    virtual void format(std::ostream& os) const override
    {
        Parameters::format(os);
        int width = format_width();
        //os
        //    << std::setw(width) << std::left << "Max. # of iterations: " << maximum_number_of_iterations << std::endl
        //    ;
    }

    virtual nlohmann::json to_json() const override
    {
        nlohmann::json json = Parameters::to_json();
        //json.merge_patch({
        //        {"MaximumNumberOfIterations", maximum_number_of_iterations},
        //        });
        return json;
    }
};

struct TreeSearchOutput: Output
{
    TreeSearchOutput(
            const Instance& instance):
        Output(instance) { }


    virtual int format_width() const override { return 36; }

    virtual void format(std::ostream& os) const override
    {
        Output::format(os);
        int width = format_width();
        //os
        //    << std::setw(width) << std::left << "Number of genetic iterations: " << number_of_genetic_iterations << std::endl
        //    ;
    }

    virtual nlohmann::json to_json() const override
    {
        nlohmann::json json = Output::to_json();
        //json.merge_patch({
        //        {"NumberOfGeneticIterations", this->number_of_genetic_iterations},
        //        });
        return json;
    }
};

const TreeSearchOutput tree_search(
        const Instance& instance,
        const TreeSearchParameters& parameters = {});

}
