#include "knapsackwithconflictssolver/algorithms/lagrangian_relaxation.hpp"

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

#include "optimizationtools/graph/clique.hpp"

#include "mathoptsolverscmake/box_constrained_nlp.hpp"

using namespace knapsackwithconflictssolver;

namespace
{

void update_items_conflicts(
        const std::vector<std::vector<optimizationtools::VertexId>>& cliques,
        std::vector<std::vector<ConflictId>>& items_conflicts)
{
    for (ItemId item_id = 0;
            item_id < (ItemId)items_conflicts.size();
            ++item_id) {
        items_conflicts[item_id].clear();
    }
    for (ConflictId clique_id = 0;
            clique_id < (ConflictId)cliques.size();
            ++clique_id) {
        const std::vector<ConflictId>& clique = cliques[clique_id];
        for (ItemId item_id: clique)
            items_conflicts[item_id].push_back(clique_id);
    }
}

std::vector<std::vector<ItemId>> find_violated_cliques(
        const Instance& instance,
        const std::vector<double>& x)
{
    std::vector<optimizationtools::VertexId> kpc_to_graph(instance.number_of_items(), -2);
    for (ItemId item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        if (x[item_id] == 0)
            kpc_to_graph[item_id] = -1;
    }
    optimizationtools::AdjacencyListGraphBuilder graph_builder;
    std::vector<ItemId> clique_to_kpc;
    for (ItemId item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        if (kpc_to_graph[item_id] == -1)
            continue;
        const Item& item = instance.item(item_id);
        ItemId vertex_id = graph_builder.add_vertex(x[item_id]);
        clique_to_kpc.push_back(item_id);
        kpc_to_graph[item_id] = vertex_id;
    }
    for (ConflictId conflict_id = 0;
            conflict_id < instance.number_of_conflicts();
            ++conflict_id) {
        const Conflict& conflict = instance.conflict(conflict_id);
        optimizationtools::VertexId vertex_1_id = kpc_to_graph[conflict.item_1_id];
        if (vertex_1_id < 0)
            continue;
        optimizationtools::VertexId vertex_2_id = kpc_to_graph[conflict.item_2_id];
        if (vertex_2_id < 0)
            continue;
        graph_builder.add_edge(vertex_1_id, vertex_2_id);
    }
    optimizationtools::AdjacencyListGraph graph = graph_builder.build();

    // Solve.
    std::vector<std::vector<optimizationtools::VertexId>> cliques = optimizationtools::vertex_clique_partition_2(graph);

    // Retrieve solution.
    std::vector<std::vector<ItemId>> violated_cliques;
    for (const std::vector<optimizationtools::VertexId>& clique: cliques) {
        std::vector<ItemId> violated_clique;
        double weight = 0;
        for (optimizationtools::VertexId vertex_id: clique) {
            ItemId item_id = clique_to_kpc[vertex_id];
            weight += x[item_id];
            violated_clique.push_back(item_id);
        }
        if (weight > 1.0) {
            //std::cout << "Found violated clique of size " << violated_clique.size()
            //    << " and weight " << weight << std::endl;
            violated_cliques.push_back(violated_clique);
        }
    }
    return violated_cliques;
}

}

LagrangianRelaxationOutput knapsackwithconflictssolver::lagrangian_relaxation(
        const Instance& instance,
        std::vector<double>* initial_multipliers,
        const LagrangianRelaxationParameters& parameters)
{
    LagrangianRelaxationOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Lagrangian relaxation");
    algorithm_formatter.print_header();

    std::vector<std::vector<ConflictId>> items_conflicts(instance.number_of_items());
    auto f = [&instance, &output, &items_conflicts](
            const std::vector<double>& multipliers)
    {
        mathoptsolverscmake::BoxConstrainedNlpFunctionOutput bcnlp_output;

        // Initialize bound and gradient;
        bcnlp_output.objective_value = 0;
        for (ConflictId clique_id = 0;
                clique_id < (ConflictId)output.cliques.size();
                ++clique_id) {
            bcnlp_output.objective_value += multipliers[clique_id];
        }
        bcnlp_output.gradient = std::vector<double>(output.cliques.size(), 1);

        std::vector<double> items_profits(instance.number_of_items(), 0);
        std::vector<ItemId> sorted_items;
        for (ItemId item_id = 0;
                item_id < instance.number_of_items();
                ++item_id) {
            const Item& item = instance.item(item_id);
            output.x[item_id] = 0;
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
        for (ItemId item_id: sorted_items) {
            const Item& item = instance.item(item_id);
            if (weight + item.weight > instance.capacity()) {
                double value = (double)(instance.capacity() - weight) / item.weight;
                output.x[item_id] = value;
                break;
            }
            weight += item.weight;
            output.x[item_id] = 1;
        }

        for (ItemId item_id = 0;
                item_id < instance.number_of_items();
                ++item_id) {
            double value = output.x[item_id];
            if (value == 0)
                continue;
            for (ConflictId clique_id: items_conflicts[item_id])
                bcnlp_output.gradient[clique_id] -= value;
            bcnlp_output.objective_value += value * items_profits[item_id];
            output.x[item_id] = value;
        }

        return bcnlp_output;
    };

    for (Counter iteration = 0;; ++iteration) {
        // Choose conflicts.

        std::vector<double> initial_multipliers_cur;
        if (parameters.conflicts_strategy == 0) {
            for (ConflictId conflict_id = 0;
                    conflict_id < instance.number_of_conflicts();
                    ++conflict_id) {
                const Conflict& conflict = instance.conflict(conflict_id);
                output.cliques.push_back({conflict.item_1_id, conflict.item_2_id});
            }
            initial_multipliers_cur = std::vector<double>(output.cliques.size(), 0);

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
            //output.conflicts = optimizationtools::edge_clique_cover(graph);
            output.cliques = optimizationtools::edge_clique_partition(graph);
            initial_multipliers_cur = std::vector<double>(output.cliques.size(), 0);

        } else if (parameters.conflicts_strategy == 2) {
            // Compute initial x.
            f(output.multipliers);

            initial_multipliers_cur = output.multipliers;

            // Add violated conflicts.
            std::vector<std::vector<ItemId>> new_cliques = find_violated_cliques(
                    instance,
                    output.x);
            bool new_cliques_found = false;
            for (std::vector<ItemId>& clique: new_cliques) {
                std::sort(clique.begin(), clique.end());
                // Check if the clique is already selected.
                bool ok = true;
                for (const std::vector<ItemId>& current_clique: output.cliques) {
                    if (clique == current_clique) {
                        ok = false;
                        break;
                    }
                }
                if (!ok)
                    continue;
                std::cout << "new clique of size " << clique.size() << std::endl;
                output.cliques.push_back(clique);
                initial_multipliers_cur.push_back(0);
                new_cliques_found = true;
            }
            if (!new_cliques_found) {
                std::cout << "No violated cliques found." << std::endl;
                break;
            }
        }

        update_items_conflicts(output.cliques, items_conflicts);

        // Set-up box-constrained nonlinear model.
        mathoptsolverscmake::BoxConstrainedNlpModel model;
        model.objective_direction = mathoptsolverscmake::ObjectiveDirection::Minimize;
        model.objective_function = f;
        for (ConflictId clique_id = 0;
                clique_id < (ConflictId)output.cliques.size();
                ++clique_id) {
            model.variables_lower_bounds.push_back(0);
            model.variables_upper_bounds.push_back(+std::numeric_limits<double>::infinity());
        }

        // Initialize multipliers
        if (iteration == 0 && initial_multipliers != NULL) {
            model.variables_initial_values = *initial_multipliers;
        } else {
            model.variables_initial_values = initial_multipliers_cur;
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

        if (parameters.timer.needs_to_end())
            break;

        if (parameters.conflicts_strategy != 2)
            break;
    }

    algorithm_formatter.end();
    return output;
}
