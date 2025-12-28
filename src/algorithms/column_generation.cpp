#include "knapsackwithconflictssolver/algorithms/column_generation.hpp"

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

#include "columngenerationsolver/algorithms/column_generation.hpp"

//#include "knapsackwithconflictssolver/utils.hpp"

//#include "knapsacksolver/instance_builder.hpp"
//#include "knapsacksolver/algorithms/dynamic_programming_primal_dual.hpp"

//#include "multiplechoiceknapsacksolver/instance_builder.hpp"
//#include "multiplechoiceknapsacksolver/algorithms/dynamic_programming_bellman.hpp"

//#include "stablesolver/clique/algorithms/milp.hpp"
#include "optimizationtools/graph/clique.hpp"

/**
 * The linear programming formulation of the problem based on Dantzig–Wolfe
 * decomposition is written as follows:
 *
 * Variables
 *
 * - \f$ y^k \in \{ 0, 1 \} \f$ representing a knapsack satisfying the capacity
 *   constraint but not necessarily satisfying the conflict constraints.
 *   \f$ y^k = 1 \f$ iff the corresponding set of items is taken in the solution.
 *   \f$ X_j^k = 1 \f$ iff $y^k$ contains item \f$ j \f$ otherwise \f$ 0 \f$.
 *
 * Objective
 *
 * Maximize the profit of the selected items.
 * \f[
 * \min \sum_k \sum_j P_j X_j^k y^k
 * \f]
 *
 * Constraints
 *
 * A single knapsack must be selected:
 * \f[
 * 1 \le ∑_k X_j^k y^k \le 1
 * \f]
 * Dual variables: u
 *
 * Conflict constraints:
 * \f[
 * \forall c \in C \qquad
 * \sum_k \sum_{j \in c} X_j^k y^k \le 1
 * \f]
 * Dual variables: \f$ v_c \f$
 *
 * Pricing problem
 *
 * The pricing problem consists in finding a variable of negative reduced cost.
 * The reduced cost of a variable \f$ y^k \f$ is given by:
 * \f[
 * r(y_i^k) = \sum_j P_j X_j^k - u - \sum_c \sum_k \sum_{j \in c} X_j^k v_c
 * \f]
 *
 * Therefore, finding a variable of minium reduced cost reduces to solving
 * a 0-1 knapsack problem with items with profits
 * \f[
 * p_j - \sum_c \sum_k \sum_{j \in c} X_j^k v_c
 * \f]
 *
 */

using namespace knapsackwithconflictssolver;

using RowIdx = columngenerationsolver::RowIdx;
using ColIdx = columngenerationsolver::ColIdx;
using Value = columngenerationsolver::Value;
using Column = columngenerationsolver::Column;

namespace
{

class PricingSolver: public columngenerationsolver::PricingSolver
{

public:

    PricingSolver(
            const Instance& instance,
            const std::vector<std::vector<ItemId>>& conflicts):
        instance_(instance),
        conflicts_(conflicts),
        conflicts_number_of_items_(conflicts_.size())
    {
        this->items_conflicts_ = std::vector<std::vector<ConflictId>>(instance.number_of_items());
        for (ConflictId conflict_id = 0;
                conflict_id < (ConflictId)conflicts.size();
                ++conflict_id) {
            const std::vector<ConflictId>& conflict = conflicts[conflict_id];
            for (ItemId item_id: conflict)
                this->items_conflicts_[item_id].push_back(conflict_id);
        }
    }

    virtual std::vector<std::shared_ptr<const Column>> initialize_pricing(
            const std::vector<std::pair<std::shared_ptr<const Column>, Value>>& fixed_columns);

    virtual columngenerationsolver::PricingSolver::PricingOutput solve_pricing(
            const std::vector<Value>& duals);

    Column solution_to_column(
            const std::vector<std::pair<ItemId, double>>& solution) const;

private:

    const Instance& instance_;

    const std::vector<std::vector<ItemId>>& conflicts_;

    std::vector<std::vector<ConflictId>> items_conflicts_;

    mutable optimizationtools::IndexedMap<ItemId> conflicts_number_of_items_;
};

columngenerationsolver::Model get_model(
        const Instance& instance,
        const std::vector<std::vector<ItemId>>& conflicts)
{
    columngenerationsolver::Model model;

    model.objective_sense = optimizationtools::ObjectiveDirection::Maximize;

    columngenerationsolver::Row row;
    row.lower_bound = 1;
    row.upper_bound = 1;
    row.coefficient_lower_bound = 0;
    row.coefficient_upper_bound = 1;
    model.rows.push_back(row);

    for (ConflictId conflict_id = 0;
            conflict_id < conflicts.size();
            ++conflict_id) {
        columngenerationsolver::Row row;
        row.lower_bound = -std::numeric_limits<double>::infinity();
        row.upper_bound = 1;
        row.coefficient_lower_bound = 0;
        row.coefficient_upper_bound = instance.number_of_items();
        model.rows.push_back(row);
    }

    // Pricing solver.
    model.pricing_solver = std::unique_ptr<columngenerationsolver::PricingSolver>(
            new PricingSolver(instance, conflicts));

    return model;
}

std::vector<std::shared_ptr<const Column>> PricingSolver::initialize_pricing(
            const std::vector<std::pair<std::shared_ptr<const Column>, Value>>& fixed_columns)
{
    return {};
}

struct ColumnExtra
{
    std::vector<std::pair<ItemId, double>> items;
};

Column PricingSolver::solution_to_column(
        const std::vector<std::pair<ItemId, double>>& solution) const
{
    Column column;
    column.objective_coefficient = 0;
    ColumnExtra extra;
    extra.items = solution;
    column.objective_coefficient = 0;
    {
        columngenerationsolver::LinearTerm element;
        element.row = 0;
        element.coefficient = 1;
        column.elements.push_back(element);
    }
    conflicts_number_of_items_.clear();
    for (const auto& p: solution) {
        ItemId item_id = p.first;
        double value = p.second;
        const Item& item = instance_.item(item_id);
        column.objective_coefficient += value * item.profit;
        for (ConflictId conflict_id: items_conflicts_[item_id]) {
            conflicts_number_of_items_.set(
                    conflict_id,
                    conflicts_number_of_items_[conflict_id] + 1);
        }
    }
    for (const auto& p: conflicts_number_of_items_) {
        columngenerationsolver::LinearTerm element;
        element.row = 1 + p.first;
        element.coefficient = p.second;
        column.elements.push_back(element);
    }

    column.extra = std::shared_ptr<void>(new ColumnExtra(extra));
    //std::cout << column << std::endl;
    return column;
}

columngenerationsolver::PricingSolver::PricingOutput PricingSolver::solve_pricing(
            const std::vector<Value>& duals)
{
    PricingOutput output;
    Value reduced_cost_bound = 0.0;

    std::vector<double> items_profits(instance_.number_of_items(), 0);
    std::vector<ItemId> sorted_items;
    for (ItemId item_id = 0;
            item_id < instance_.number_of_items();
            ++item_id) {
        const Item& item = instance_.item(item_id);
        double profit = item.profit;
        for (ConflictId conflict_id: items_conflicts_[item_id])
            profit -= duals[1 + conflict_id];
        items_profits[item_id] = profit;
        if (profit > 0)
            sorted_items.push_back(item_id);
    }
    std::sort(
            sorted_items.begin(),
            sorted_items.end(),
            [this, &items_profits](ItemId item_1_id, ItemId item_2_id)
            {
                const Item& item_1 = this->instance_.item(item_1_id);
                const Item& item_2 = this->instance_.item(item_2_id);
                return items_profits[item_1_id] / item_1.weight
                    > items_profits[item_2_id] / item_2.weight;
            });
    Profit profit = 0;
    Weight weight = 0;
    std::vector<std::pair<ItemId, double>> selected_items;
    for (ItemId item_id: sorted_items) {
        const Item& item = this->instance_.item(item_id);
        if (weight + item.weight > this->instance_.capacity()) {
            double x = (double)(this->instance_.capacity() - weight) / item.weight;
            profit += x * item.profit;;
            selected_items.push_back({item_id, x});
            break;
        }
        profit += item.profit;
        weight += item.weight;
        selected_items.push_back({item_id, 1});
    }

    // Retrieve column.
    output.columns.push_back(std::shared_ptr<const columngenerationsolver::Column>(new columngenerationsolver::Column(this->solution_to_column(selected_items))));
    output.overcost = std::max(0.0, columngenerationsolver::compute_reduced_cost(*output.columns.front(), duals));
    return output;
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

ColumnGenerationOutput knapsackwithconflictssolver::column_generation(
        const Instance& instance,
        const ColumnGenerationParameters& parameters)
{
    ColumnGenerationOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Column generation");
    algorithm_formatter.print_header();

    for (Counter iteration = 0;; ++iteration) {
        // Choose conflicts.

        if (parameters.conflicts_strategy == 0) {
            for (ConflictId conflict_id = 0;
                    conflict_id < instance.number_of_conflicts();
                    ++conflict_id) {
                const Conflict& conflict = instance.conflict(conflict_id);
                output.cliques.push_back({conflict.item_1_id, conflict.item_2_id});
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
            //output.cliques = optimizationtools::edge_clique_cover(graph);
            output.cliques = optimizationtools::edge_clique_partition(graph);

        } else if (parameters.conflicts_strategy == 2) {
            // Compute initial x.
            if (iteration == 0) {
                columngenerationsolver::Model model = get_model(instance, output.cliques);
                auto pricing_output = model.pricing_solver->solve_pricing({0});
                std::shared_ptr<ColumnExtra> extra
                    = std::static_pointer_cast<ColumnExtra>(pricing_output.columns[0]->extra);
                for (const auto& p: extra->items)
                    output.x[p.first] += p.second;
            }

            // Add violated conflicts.
            std::vector<std::vector<ItemId>> new_cliques = find_violated_cliques(
                    instance,
                    output.x);
            if (new_cliques.empty()) {
                std::cout << "No violated cliques found." << std::endl;
                break;
            }

            // Remove inactive conflicts.
            for (ConflictId clique_id = 0;
                    clique_id < (ConflictId)output.cliques.size();
                    ++clique_id) {
                double value = 0;
                const std::vector<ItemId>& clique = output.cliques[clique_id];
                for (ItemId item_id: clique)
                    value += output.x[item_id];
                if (value <= 0.99)
                    continue;
                new_cliques.push_back(clique);
            }

            output.cliques = new_cliques;
        }

        // Column generation.
        columngenerationsolver::Model model = get_model(instance, output.cliques);
        columngenerationsolver::ColumnGenerationParameters cgs_parameters;
        cgs_parameters.verbosity_level = 0;
        cgs_parameters.timer = parameters.timer;
        //cgs_parameters.self_adjusting_wentges_smoothing = true;
        //cgs_parameters.automatic_directional_smoothing = true;
        cgs_parameters.solver_name = parameters.linear_programming_solver;
        // Give last optimal solution as initial columns instead of restarting
        // the column generation from scratch.
        for (const std::vector<std::pair<ItemId, double>>& items: output.columns) {
            Column column = static_cast<const PricingSolver&>(*model.pricing_solver).solution_to_column(items);
            cgs_parameters.initial_columns.push_back(std::shared_ptr<const columngenerationsolver::Column>(new columngenerationsolver::Column(column)));
        }
        bool end = false;
        if (parameters.conflicts_strategy == 2) {
            cgs_parameters.timer.add_end_boolean(&end);
            cgs_parameters.iteration_callback = [&output, &end](
                    const columngenerationsolver::ColumnGenerationOutput& cg_output)
            {
                if (cg_output.number_of_column_generation_iterations > 100
                        && cg_output.bound < output.bound) {
                    double gap_1 = (output.bound - cg_output.bound) / cg_output.bound;
                    double gap_2 = (cg_output.bound - cg_output.relaxation_solution_value) / cg_output.relaxation_solution_value;
                    //std::cout << "gap_1 " << gap_1 << " gap_2 " << gap_2 << std::endl;
                    if (gap_2 < gap_1)
                        end = true;
                }
            };
        }
        auto cgs_output = columngenerationsolver::column_generation(model, cgs_parameters);

        if (cgs_output.bound != std::numeric_limits<double>::infinity())
            algorithm_formatter.update_bound(cgs_output.bound, "");

        // Update x and columns.
        output.columns.clear();
        std::fill(output.x.begin(), output.x.end(), 0);
        for (const auto& colval: cgs_output.relaxation_solution.columns()) {
            std::shared_ptr<ColumnExtra> extra
                = std::static_pointer_cast<ColumnExtra>(colval.first->extra);
            output.columns.push_back(extra->items);
            for (const auto& p: extra->items)
                output.x[p.first] += p.second * colval.second;
        }

        if (parameters.timer.needs_to_end())
            break;

        if (parameters.conflicts_strategy != 2)
            break;
    }

    algorithm_formatter.end();
    return output;
}
