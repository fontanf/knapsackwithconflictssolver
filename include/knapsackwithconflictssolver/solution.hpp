#pragma once

#include "knapsackwithconflictssolver/instance.hpp"

#include "optimizationtools/containers/indexed_set.hpp"

namespace knapsackwithconflictssolver
{

class Solution
{

public:

    struct Item
    {
        /**
         * neighbor_weight = w iff the sum of the weights of the neighbors of j
         * which are in the solution is equal to w.
         */
        Weight neighbor_weight = 0;

        /**
         * neighbor_profit = p iff the sum of the profits of the neighbors of j
         * which are in the solution is equal to p.
         */
        Profit neighbor_profit = 0;
    };

    /*
     * Constructors and destructor
     */

    /** Constructor to build an instance manually. */
    Solution(const Instance& instance):
        instance_(&instance),
        items_set_(instance.number_of_items()),
        items_(instance.number_of_items()),
        conflicts_(instance.number_of_conflicts()) { }

    Solution(
            const Instance& instance,
            std::string certificate_path);

    /*
     * Getters
     */

    /** Get the instance. */
    inline const Instance& instance() const { return *instance_; }

    /** Get the number of items in the solution. */
    inline ItemId number_of_items() const { return items_set_.size(); }

    /** Get an item. */
    inline const Item& item(ItemId item_id) const { return items_[item_id]; }

    /** Get the items set. */
    inline const optimizationtools::IndexedSet& items() const { return items_set_; }

    /** Get the total profit of the solution. */
    inline Profit profit() const { return total_profit_; }

    /** Get the total weight of the solution. */
    inline Weight weight() const { return total_weight_; }

    /** Get the number of conflicts in the solution. */
    inline ItemId number_of_conflicts() const { return conflicts_.size(); }

    /** Get the conflicts. */
    inline const optimizationtools::IndexedSet& conflicts() const { return conflicts_; }

    /** Check if the solution contains a given item. */
    inline bool contains(ItemId item_id) const { return items_set_.contains(item_id); }

    /** Get the objective value. */
    inline Profit objective_value() const { return profit(); }

    /** Check if the solution is feasible. */
    inline bool feasible() const { return weight() <= instance().capacity() && number_of_conflicts() == 0; }

    /*
     * Setters
     */

    /** Add an item to the solution. */
    inline void add(ItemId item_id);

    /** Remove an item from the solution. */
    inline void remove(ItemId item_id);

    /*
     * Export
     */

    /** Write the solution to a file. */
    void write(
            const std::string& certificate_path,
            const std::string& format = "") const;

    /** Write an instance file in 'standard' format. */
    void write_standard(
            std::ostream& file) const;

    /** Export solution characteristics to a JSON structure. */
    nlohmann::json to_json() const;

    /** Write a formatted output of the instance to a stream. */
    void format(
            std::ostream& os,
            int verbosity_level = 1) const;

private:

    /*
     * Private methods
     */

    /*
     * Private attributes
     */

    /** Instance. */
    const Instance* instance_;

    /** Items set. */
    optimizationtools::IndexedSet items_set_;

    /** Items. */
    std::vector<Solution::Item> items_;

    /** Conflicts. */
    optimizationtools::IndexedSet conflicts_;

    /** Total profit of the solution. */
    Profit total_profit_ = 0;

    /** Total weight of the solution. */
    Weight total_weight_ = 0;

    friend class SolutionBuilder;
};

void Solution::add(ItemId item_id)
{
    // Checks.
    if (contains(item_id)) {
        throw std::invalid_argument(
                "knapsackwithconflictssolver::Solution::add: "
                "added item must not already be in the solution; "
                "item_id: " + std::to_string(item_id) + ".");
    }

    const knapsackwithconflictssolver::Item& item = instance().item(item_id);
    for (const ItemConflict& conflict: item.neighbors) {
        if (this->contains(conflict.item_id))
            conflicts_.add(conflict.conflict_id);
        items_[conflict.item_id].neighbor_weight += item.weight;
        items_[conflict.item_id].neighbor_profit += item.profit;
    }
    items_set_.add(item_id);
    total_weight_ += item.weight;
    total_profit_ += item.profit;
}

void Solution::remove(ItemId item_id)
{
    // Checks.
    if (!this->contains(item_id)) {
        throw std::invalid_argument(
                "knapsackwithconflictssolver::Solution::remove: "
                "removed item must already be in the solution; "
                "item_id: " + std::to_string(item_id) + ".");
    }

    const knapsackwithconflictssolver::Item& item = instance().item(item_id);
    for (const ItemConflict& conflict: item.neighbors) {
        conflicts_.remove(conflict.conflict_id);
        items_[conflict.item_id].neighbor_weight -= item.weight;
        items_[conflict.item_id].neighbor_profit -= item.profit;
    }
    items_set_.remove(item_id);
    total_weight_ -= item.weight;
    total_profit_ -= item.profit;
}

}
