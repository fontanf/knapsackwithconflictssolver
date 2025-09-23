#include "knapsackwithconflictssolver/algorithms/tree_search.hpp"

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

#include "treesearchsolver/best_first_search.hpp"
#include "treesearchsolver/anytime_column_search.hpp"
#include "treesearchsolver/iterative_beam_search.hpp"

using namespace knapsackwithconflictssolver;

namespace
{

using NodeId = int64_t;
using GuideId = int64_t;

class BranchingScheme
{

public:

    struct Parameters
    {
        GuideId guide_id = 0;
    };

    struct Node
    {
        /** Parent node. */
        std::shared_ptr<Node> parent = nullptr;

        /** Array indicating for each item, if it still available. */
        std::vector<bool> available_items;

        ItemId item_pos = -1;

        /** Last item added to the partial solution. */
        ItemId item_id = -1;

        /** Number of items in the partial solution. */
        ItemId number_of_items = 0;

        /** Number of remaining available items. */
        ItemId number_of_remaining_items = -1;

        /** Weight of the remaining available items. */
        Profit remaining_weight = 0;

        /** Profit of the remaining available items. */
        Profit remaining_profit = 0;

        /** Weight of the partial solution. */
        Weight weight = 0;

        /** Profit of the partial solution. */
        Profit profit = 0;

        /** Guide. */
        double guide = 0;

        /** Next child to generate. */
        ItemId next_child_pos = 0;

        /** Unique id of the node. */
        NodeId id = -1;
    };

    BranchingScheme(
            const Instance& instance,
            AlgorithmFormatter& algorithm_formatter,
            const Parameters& parameters):
        instance_(instance),
        algorithm_formatter_(algorithm_formatter),
        parameters_(parameters)
    {
        sorted_items_ = std::vector<ItemId>(instance.number_of_items(), -1);
        std::iota(sorted_items_.begin(), sorted_items_.end(), 0);
        std::sort(
                sorted_items_.begin(),
                sorted_items_.end(),
                [&instance](ItemId item_id_1, ItemId item_id_2)
                {
                    const Item& item_1 = instance.item(item_id_1);
                    const Item& item_2 = instance.item(item_id_2);
                    return (double)item_1.profit / item_1.weight
                        > (double)item_2.profit / item_2.weight;
                });
    }

    inline const std::shared_ptr<Node> root() const
    {
        auto r = std::shared_ptr<Node>(new BranchingScheme::Node());
        r->id = node_id_;
        node_id_++;
        r->available_items.resize(instance_.number_of_items(), true);
        r->number_of_remaining_items = instance_.number_of_items();
        for (ItemId item_id = 0;
                item_id < instance_.number_of_items();
                ++item_id) {
            r->remaining_weight += instance_.item(item_id).weight;
            r->remaining_profit += instance_.item(item_id).profit;
        }
        return r;
    }

    inline std::shared_ptr<Node> next_child(
            const std::shared_ptr<Node>& parent) const
    {
        //std::cout << "parent " << parent->id
        //    << " guide " << parent->guide
        //    << " pos " << parent->item_pos
        //    << " item_id " << parent->item_id
        //    << " # " << parent->number_of_items
        //    << " w " << parent->weight
        //    << " p " << parent->profit
        //    << " ri " << parent->number_of_remaining_items
        //    << " rp " << parent->remaining_profit
        //    << std::endl;

        // Get the next item to add.
        ItemId item_id_next = sorted_items_[parent->next_child_pos];

        // Update parent
        parent->next_child_pos++;

        // Check if the item is still available.
        if (!parent->available_items[item_id_next])
            return nullptr;

        // Check if the item fit in the knapsack.
        if (parent->weight + instance_.item(item_id_next).weight > instance_.capacity())
            return nullptr;

        // Compute new child.
        auto child = std::shared_ptr<Node>(new BranchingScheme::Node());
        child->id = node_id_;
        node_id_++;
        child->parent = parent;
        child->item_pos = parent->next_child_pos - 1;
        child->next_child_pos = parent->next_child_pos;
        child->item_id = item_id_next;
        child->number_of_items = parent->number_of_items + 1;
        child->available_items = parent->available_items;
        child->available_items[item_id_next] = false;
        child->number_of_remaining_items = parent->number_of_remaining_items - 1;
        child->remaining_weight = parent->remaining_weight - instance_.item(item_id_next).weight;
        child->remaining_profit = parent->remaining_profit - instance_.item(item_id_next).profit;
        for (ItemId pos = parent->item_pos + 1; pos < child->item_pos; ++pos) {
            ItemId item_id = sorted_items_[pos];
            if (child->available_items[item_id]) {
                child->available_items[item_id] = false;
                child->number_of_remaining_items--;
                child->remaining_weight -= instance_.item(item_id).weight;
                child->remaining_profit -= instance_.item(item_id).profit;
            }
        }
        for (const ItemConflict& conflict: instance_.item(item_id_next).neighbors) {
            if (child->available_items[conflict.item_id]) {
                child->available_items[conflict.item_id] = false;
                child->number_of_remaining_items--;
                child->remaining_weight -= instance_.item(conflict.item_id).weight;
                child->remaining_profit -= instance_.item(conflict.item_id).profit;
            }
        }
        child->weight = parent->weight + instance_.item(item_id_next).weight;
        child->profit = parent->profit + instance_.item(item_id_next).profit;

        parent->guide = parent->profit;
        Weight w = parent->weight;
        for (ItemId pos = child->next_child_pos; pos < instance_.number_of_items(); ++pos) {
            ItemId item_id = sorted_items_[pos];
            if (parent->available_items[item_id] == 0)
                continue;
            const Item& item = instance_.item(item_id);
            if (w + item.weight > instance_.capacity()) {
                parent->guide += (double)item.profit * (instance_.capacity() - w) / item.weight;
                break;
            }
            parent->guide += item.profit;
            w += item.weight;
        }

        child->guide = child->profit;
        w = child->weight;
        for (ItemId pos = child->next_child_pos; pos < instance_.number_of_items(); ++pos) {
            ItemId item_id = sorted_items_[pos];
            if (child->available_items[item_id] == 0)
                continue;
            const Item& item = instance_.item(item_id);
            if (w + item.weight > instance_.capacity()) {
                child->guide += (double)item.profit * (instance_.capacity() - w) / item.weight;
                break;
            }
            child->guide += item.profit;
            w += item.weight;
        }

        Solution solution(instance_);
        std::vector<uint8_t> available_items(instance_.number_of_items(), 1);
        for (auto node_tmp = child;
                node_tmp->parent != nullptr;
                node_tmp = node_tmp->parent) {
            ItemId item_id = node_tmp->item_id;
            const Item& item = instance_.item(item_id);
            solution.add(item_id);
            available_items[item_id] = 0;
            for (const ItemConflict& conflict: item.neighbors)
                available_items[conflict.item_id] = 0;
        }
        for (ItemId pos = child->next_child_pos; pos < instance_.number_of_items(); ++pos) {
            ItemId item_id = sorted_items_[pos];
            if (available_items[item_id] == 0)
                continue;
            const Item& item = instance_.item(item_id);
            if (solution.weight() + item.weight > instance_.capacity()) {
                solution.add(item_id);
                ItemId item_best_id = -1;
                Profit profit_best = 0;
                for (ItemId item_id: solution.items()) {
                    const Item& item = instance_.item(item_id);
                    Profit profit = solution.profit() - item.profit;
                    Weight weight = solution.weight() - item.weight;
                    if (weight > instance_.capacity())
                        continue;
                    if (item_best_id == -1
                            || profit_best < profit) {
                        item_best_id = item_id;
                        profit_best = profit;
                    }
                }
                solution.remove(item_best_id);
                break;
            }
            solution.add(item_id);
            available_items[item_id] = 0;
            for (const ItemConflict& conflict: item.neighbors)
                available_items[conflict.item_id] = 0;
        }
        //std::cout << "solution profit " << solution.profit()
        //    << " weight " << solution.weight()
        //    << " # conflicts " << solution.number_of_conflicts()
        //    << std::endl;
        algorithm_formatter_.update_solution(solution, "");

        //child->guide =
        //    (parameters_.guide_id == 0)? (double)child->weight / child->profit:
        //    (parameters_.guide_id == 1)? (double)child->weight / child->profit / child->remaining_profit:
        //                                 (double)1.0 / (child->profit + child->remaining_profit);
        return child;
    }

    inline treesearchsolver::Depth depth(
            const std::shared_ptr<Node>& node) const
    {
        return instance_.number_of_items() - node->number_of_remaining_items;
    }

    inline bool infertile(
            const std::shared_ptr<Node>& node) const
    {
        return (node->next_child_pos == instance_.number_of_items());
    }

    inline bool operator()(
            const std::shared_ptr<Node>& node_1,
            const std::shared_ptr<Node>& node_2) const
    {
        if (node_1->guide != node_2->guide)
            return node_1->guide > node_2->guide;
        return node_1->id < node_2->id;
    }

    inline bool leaf(
            const std::shared_ptr<Node>& node) const
    {
        return node->number_of_items == instance_.number_of_items();
    }

    bool bound(
            const std::shared_ptr<Node>& node_1,
            const std::shared_ptr<Node>& node_2) const
    {
        return node_1->profit + node_1->remaining_profit <= node_2->profit;
    }

    /*
     * Solution pool.
     */

    bool better(
            const std::shared_ptr<Node>& node_1,
            const std::shared_ptr<Node>& node_2) const
    {
        return node_1->profit > node_2->profit;
    }

    std::shared_ptr<Node> goal_node(double value) const
    {
        auto node = std::shared_ptr<Node>(new BranchingScheme::Node());
        node->profit = value;
        return node;
    }

    bool equals(
            const std::shared_ptr<Node>& node_1,
            const std::shared_ptr<Node>& node_2) const
    {
        if (node_1->number_of_items != node_2->number_of_items)
            return false;
        std::vector<bool> v(instance_.number_of_items(), false);
        for (auto node_tmp = node_1; node_tmp->parent != nullptr; node_tmp = node_tmp->parent)
            v[node_tmp->item_id] = true;
        for (auto node_tmp = node_1; node_tmp->parent != nullptr; node_tmp = node_tmp->parent)
            if (!v[node_tmp->item_id])
                return false;
        return true;
    }

    std::string display(const std::shared_ptr<Node>& node) const
    {
        std::stringstream ss;
        ss << node->profit
            << " (n" << node->number_of_items << "/" << instance_.number_of_items()
            << " w" << node->weight << "/" << instance_.capacity()
            << ")";
        return ss.str();
    }

    /*
     * Dominances.
     */

    inline bool comparable(
            const std::shared_ptr<Node>&) const
    {
        return true;
    }

    struct NodeHasher
    {
        std::hash<std::vector<bool>> hasher;

        inline bool operator()(
                const std::shared_ptr<Node>& node_1,
                const std::shared_ptr<Node>& node_2) const
        {
            return node_1->available_items == node_2->available_items;
        }

        inline std::size_t operator()(
                const std::shared_ptr<Node>& node) const
        {
            size_t hash = hasher(node->available_items);
            return hash;
        }
    };

    inline NodeHasher node_hasher() const { return NodeHasher(); }

    inline bool dominates(
            const std::shared_ptr<Node>& node_1,
            const std::shared_ptr<Node>& node_2) const
    {
        if (node_1->profit >= node_2->profit
                && node_1->weight <= node_2->weight)
            return true;
        return false;
    }

private:

    /** Instance. */
    const Instance& instance_;

    /** Parameters. */
    Parameters parameters_;

    AlgorithmFormatter& algorithm_formatter_;

    std::vector<ItemId> sorted_items_;

    mutable NodeId node_id_ = 0;

};

}

const TreeSearchOutput knapsackwithconflictssolver::tree_search(
        const Instance& instance,
        const TreeSearchParameters& parameters)
{
    TreeSearchOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Tree search");

    if (parameters.timer.needs_to_end()) {
        algorithm_formatter.end();
        return output;
    }

    algorithm_formatter.print_header();

    BranchingScheme::Parameters branching_scheme_parameters;
    BranchingScheme branching_scheme(instance, algorithm_formatter, branching_scheme_parameters);
    treesearchsolver::BestFirstSearchParameters<BranchingScheme> bfs_parameters;
    //treesearchsolver::AnytimeColumnSearchParameters<BranchingScheme> bfs_parameters;
    //treesearchsolver::IterativeBeamSearchParameters<BranchingScheme> bfs_parameters;
    bfs_parameters.timer = parameters.timer;
    bfs_parameters.verbosity_level = 0;
    bfs_parameters.new_solution_callback
        = [&instance, &algorithm_formatter](
                const treesearchsolver::Output<BranchingScheme>& ts_output)
        {
            const auto& bfs_output = static_cast<const treesearchsolver::BestFirstSearchOutput<BranchingScheme>&>(ts_output);
            //const auto& bfs_output = static_cast<const treesearchsolver::AnytimeColumnSearchOutput<BranchingScheme>&>(ts_output);
            //const auto& bfs_output = static_cast<const treesearchsolver::IterativeBeamSearchOutput<BranchingScheme>&>(ts_output);
            Solution solution(instance);
            auto node = bfs_output.solution_pool.best();
            for (auto node_tmp = node;
                    node_tmp->parent != nullptr;
                    node_tmp = node_tmp->parent) {
                solution.add(node_tmp->item_id);
            }
            std::stringstream ss;
            ss << "node " << bfs_output.number_of_nodes;
            algorithm_formatter.update_solution(solution, ss.str());
        };
    treesearchsolver::best_first_search(branching_scheme, bfs_parameters);
    //treesearchsolver::anytime_column_search(branching_scheme, bfs_parameters);
    //treesearchsolver::iterative_beam_search(branching_scheme, bfs_parameters);

    algorithm_formatter.end();
    return output;
}
