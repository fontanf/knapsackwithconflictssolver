#pragma once

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

#include "mathoptsolverscmake/common.hpp"

namespace knapsackwithconflictssolver
{

struct Milp3Parameters: Parameters
{
    /** MILP solver. */
    mathoptsolverscmake::SolverName solver = mathoptsolverscmake::SolverName::Highs;


    virtual int format_width() const override { return 37; }

    virtual void format(std::ostream& os) const override
    {
        Parameters::format(os);
        int width = format_width();
    }

    virtual nlohmann::json to_json() const override
    {
        nlohmann::json json = Parameters::to_json();
        json.merge_patch({
                });
        return json;
    }
};

struct Milp3Output: Output
{
    Milp3Output(
            const Instance& instance):
        Output(instance) { }


    /** Number of iterations. */
    Counter number_of_iterations = 0;


    virtual int format_width() const override { return 31; }

    virtual void format(std::ostream& os) const override
    {
        Output::format(os);
        int width = format_width();
        os
            << std::setw(width) << std::left << "Number of iterations: " << number_of_iterations << std::endl
            ;
    }

    virtual nlohmann::json to_json() const override
    {
        nlohmann::json json = Output::to_json();
        json.merge_patch({
                {"NumberOfIterations", number_of_iterations},
                });
        return json;
    }
};

Milp3Output milp_3(
        const Instance& instance,
        const Milp3Parameters& parameters = {});


struct Milp3LinearRelaxationOutput: Output
{
    Milp3LinearRelaxationOutput(
            const Instance& instance):
        Output(instance) { }


    /** Solution of the linear relaxation. */
    std::vector<double> relaxation_solution;
};

Milp3LinearRelaxationOutput milp_3_linear_relaxation(
        const Instance& instance,
        const Milp3Parameters& parameters = {});


void write_mps_3(
        const Instance& instance,
        mathoptsolverscmake::SolverName solver,
        const std::string& output_path);

}
