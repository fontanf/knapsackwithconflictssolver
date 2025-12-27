#include "knapsackwithconflictssolver/algorithms/lagrangian_relaxation.hpp"

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

#include "optimizationtools/graph/clique.hpp"

#include "mathoptsolverscmake/box_constrained_nlp.hpp"

using namespace knapsackwithconflictssolver;

LagrangianRelaxationOutput knapsackwithconflictssolver::lagrangian_relaxation(
        const Instance& instance,
        std::vector<double>* initial_multipliers,
        const LagrangianRelaxationParameters& parameters)
{
    LagrangianRelaxationOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Lagrangian relaxation");
    algorithm_formatter.print_header();

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
    //std::vector<std::vector<optimizationtools::VertexId>> clique_partition
    //    = optimizationtools::edge_clique_cover(graph);
    std::vector<std::vector<optimizationtools::VertexId>> clique_partition
        = optimizationtools::edge_clique_partition(graph);

    std::vector<std::vector<ConflictId>> items_conflicts(instance.number_of_items());
    for (ConflictId clique_id = 0;
            clique_id < (ConflictId)clique_partition.size();
            ++clique_id) {
        const std::vector<ConflictId>& clique = clique_partition[clique_id];
        for (ItemId item_id: clique)
            items_conflicts[item_id].push_back(clique_id);
    }

    mathoptsolverscmake::BoxConstrainedNlpModel model;

    model.objective_direction = mathoptsolverscmake::ObjectiveDirection::Minimize;
    model.objective_function = [&instance, &clique_partition, &items_conflicts](
            const std::vector<double>& multipliers)
    {
        mathoptsolverscmake::BoxConstrainedNlpFunctionOutput output;

        // Initialize bound and gradient;
        output.objective_value = 0;
        for (ConflictId clique_id = 0;
                clique_id < (ConflictId)clique_partition.size();
                ++clique_id) {
            output.objective_value += multipliers[clique_id];
        }
        output.gradient = std::vector<double>(instance.number_of_conflicts(), 1);

        std::vector<double> items_profits(instance.number_of_items(), 0);
        std::vector<ItemId> sorted_items;
        for (ItemId item_id = 0;
                item_id < instance.number_of_items();
                ++item_id) {
            const Item& item = instance.item(item_id);
            double profit = item.profit;
            for (ConflictId clique_id: items_conflicts[item_id])
                profit -= multipliers[clique_id];
            items_profits[item_id] = profit;
            if (profit > 0)
                sorted_items.push_back(item_id);
        }
        std::sort(
                sorted_items.begin(),
                sorted_items.end(),
                [&instance, &items_profits](ItemId item_1_id, ItemId item_2_id)
                {
                    const Item& item_1 = instance.item(item_1_id);
                    const Item& item_2 = instance.item(item_2_id);
                    return items_profits[item_1_id] / item_1.weight
                        > items_profits[item_2_id] / item_2.weight;
                });
        Weight weight = 0;
        std::vector<std::pair<ItemId, double>> selected_items;
        for (ItemId item_id: sorted_items) {
            const Item& item = instance.item(item_id);
            if (weight + item.weight > instance.capacity()) {
                double x = (double)(instance.capacity() - weight) / item.weight;
                for (ConflictId clique_id: items_conflicts[item_id])
                    output.gradient[clique_id] -= x;
                output.objective_value += x * items_profits[item_id];
                selected_items.push_back({item_id, x});
                break;
            }
            weight += item.weight;
            for (ConflictId clique_id: items_conflicts[item_id])
                output.gradient[clique_id]--;
            output.objective_value += items_profits[item_id];
            selected_items.push_back({item_id, 1});
        }

        return output;
    };

    for (ConflictId clique_id = 0;
            clique_id < (ConflictId)clique_partition.size();
            ++clique_id) {
        model.variables_lower_bounds.push_back(0);
        model.variables_upper_bounds.push_back(+std::numeric_limits<double>::infinity());
    }

    // Initialize multipliers
    if (initial_multipliers != NULL) {
        model.variables_initial_values = *initial_multipliers;
    } else {
        model.variables_initial_values = std::vector<double>(model.number_of_variables(), 0);
    }

    // Solve.
    double bcnlp_bound = 0;
#if DLIB_FOUND
    if (parameters.solver == mathoptsolverscmake::SolverName::Dlib) {
        mathoptsolverscmake::BoxConstrainedNlpDlibOutput dlib_output = mathoptsolverscmake::solve_dlib(model);
        bcnlp_bound = dlib_output.objective_value;
        output.multipliers = dlib_output.solution;
    }
#endif
#if CONICBUNDLE_FOUND
    if (parameters.solver == mathoptsolverscmake::SolverName::ConicBundle) {
        mathoptsolverscmake::BoxConstrainedNlpConicBundleOutput conicbundle_output = mathoptsolverscmake::solve_conicbundle(model);
        bcnlp_bound = conicbundle_output.objective_value;
        output.multipliers = conicbundle_output.solution;
    }
#endif

    // Fill output.
    algorithm_formatter.update_bound(std::floor(bcnlp_bound + FFOT_TOL), "");

    algorithm_formatter.end();
    return output;
}
