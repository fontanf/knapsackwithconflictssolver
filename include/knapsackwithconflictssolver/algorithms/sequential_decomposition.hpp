#pragma once

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

namespace knapsackwithconflictssolver
{

struct SequentialDecompositionParameters: Parameters
{
    int stable_weight_strategy = 0;

    virtual int format_width() const override { return 37; }

    virtual void format(std::ostream& os) const override
    {
        Parameters::format(os);
        int width = format_width();
        os
            << std::setw(width) << std::left << "Stable weight strategy: " << stable_weight_strategy << std::endl
            ;
    }

    virtual nlohmann::json to_json() const override
    {
        nlohmann::json json = Parameters::to_json();
        json.merge_patch({
                {"StableWeightStrategy", stable_weight_strategy},
                });
        return json;
    }
};

Output sequential_decomposition(
        const Instance& instance,
        const SequentialDecompositionParameters& parameters = {});

}
