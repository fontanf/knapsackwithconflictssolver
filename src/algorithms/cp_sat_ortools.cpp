#include "knapsackwithconflictssolver/algorithms/cp_sat_ortools.hpp"

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

#include "optimizationtools/graph/clique.hpp"

#include "ortools/sat/cp_model.h"

using namespace knapsackwithconflictssolver;

Output knapsackwithconflictssolver::cp_sat_ortools(
        const Instance& instance,
        const CpSatOrtoolsParameters& parameters)
{
    Output output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("CP-SAT (OR-Tools)");
    algorithm_formatter.print_header();

    operations_research::sat::CpModelBuilder cp;

    // Variables.
    std::vector<operations_research::sat::BoolVar> x;
    x.reserve(instance.number_of_items());
    for (ItemId item_id = 0; item_id < instance.number_of_items(); ++item_id)
        x.push_back(cp.NewBoolVar());

    // Objective: maximize total profit
    operations_research::sat::LinearExpr objective;
    for (ItemId item_id = 0; item_id < instance.number_of_items(); ++item_id) {
        const Item& item = instance.item(item_id);
        objective += item.profit * x[item_id];
    }
    cp.Maximize(objective);

    // Capacity constraint.
    operations_research::sat::LinearExpr expr;
    for (ItemId item_id = 0; item_id < instance.number_of_items(); ++item_id) {
        const Item& item = instance.item(item_id);
        expr += item.weight * x[item_id];
        cp.AddLessOrEqual(expr, instance.capacity());
    }

    // Conflict constraints.
    if (parameters.conflicts_strategy == 0) {
        for (ConflictId conflict_id = 0;
                conflict_id < instance.number_of_conflicts();
                ++conflict_id) {
            const Conflict& conflict = instance.conflict(conflict_id);
            operations_research::sat::LinearExpr expr;
            expr += x[conflict.item_1_id];
            expr += x[conflict.item_2_id];
            cp.AddLessOrEqual(expr, 1);
        }
    } else if (parameters.conflicts_strategy == 1) {
        optimizationtools::AdjacencyListGraphBuilder graph_builder;
        for (ItemId item_id = 0;
                item_id < instance.number_of_items();
                ++item_id) {
            const Item& item = instance.item(item_id);
            optimizationtools::Weight weight = (double)item.profit / item.weight;
            graph_builder.add_vertex(weight);
        }
        for (ConflictId conflict_id = 0;
                conflict_id < instance.number_of_conflicts();
                ++conflict_id) {
            const Conflict& conflict = instance.conflict(conflict_id);
            graph_builder.add_edge(
                    conflict.item_1_id,
                    conflict.item_2_id);
        }
        optimizationtools::AdjacencyListGraph graph = graph_builder.build();
        auto cliques = optimizationtools::edge_clique_partition(graph);
        for (const std::vector<ItemId>& clique: cliques) {
            operations_research::sat::LinearExpr expr;
            for (ItemId item_id: clique)
                expr += x[item_id];
            cp.AddLessOrEqual(expr, 1);
        }
    }

    operations_research::sat::SatParameters cp_parameters;

    // Write solver output to file.
    // TODO
    cp_parameters.set_log_search_progress(true);

    // Item time limit.
    cp_parameters.set_max_time_in_seconds(parameters.timer.remaining_time());

    // Solve
    operations_research::sat::CpSolverResponse resp = operations_research::sat::SolveWithParameters(cp.Build(), cp_parameters);
    if (resp.status() == operations_research::sat::CpSolverStatus::OPTIMAL
            || resp.status() == operations_research::sat::CpSolverStatus::FEASIBLE) {
        Solution solution(instance);
        for (ItemId item_id = 0; item_id < instance.number_of_items(); ++item_id)
            if (SolutionIntegerValue(resp, x[item_id]) > 0)
                solution.add(item_id);
        algorithm_formatter.update_solution(solution, "");
        if (resp.status() == operations_research::sat::CpSolverStatus::OPTIMAL)
            algorithm_formatter.update_bound(solution.profit(), "");
    }

    algorithm_formatter.end();
    return output;
}
