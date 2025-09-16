#pragma once

#include "nlohmann//json.hpp"

#include <cstdint>
#include <vector>
#include <iostream>

namespace knapsackwithconflictssolver
{

using ItemId = int64_t;
using ConflictId = int64_t;
using Weight = int64_t;
using Profit = int64_t;
using Counter = int64_t;
using Seed = int64_t;

/**
 * Structure that stores the information of a neighbor for a considered item.
 */
struct ItemConflict
{
    /** Id of the conflict. */
    ConflictId conflict_id;

    /** Id of the neighbor. */
    ItemId item_id;
};

/*
 * Structure for an item.
 */
struct Item
{
    /** Weight of the item. */
    Weight weight = 0;

    /** Profit of the item. */
    Profit profit = 0;

    /** Conflicting items. */
    std::vector<ItemConflict> neighbors;
};

/**
 * Structure that stores the information for a conflict.
 */
struct Conflict
{
    /** Id of the first item of the conflict. */
    ItemId item_1_id;

    /** Id of the second item of the conflict. */
    ItemId item_2_id;
};

/**
 * Instance class for a 'knapsack_with_conflicts' problem.
 */
class Instance
{

public:

    /*
     * Getters
     */

    /** Get the number of items. */
    inline ItemId number_of_items() const { return items_.size(); }

    /** Get an item. */
    inline const Item& item(ItemId item_id) const { return items_[item_id]; }

    /** Get the capacity of the knapsack. */
    inline Weight capacity() const { return capacity_; }

    /** Get the number of conflicts. */
    inline ConflictId number_of_conflicts() const { return conflicts_.size(); }

    /** Get a conflict. */
    inline const Conflict& conflict(ConflictId conflict_id) const { return conflicts_[conflict_id]; }

    /** Get the total weight of the items. */
    inline Weight total_weight() const { return total_weight_; }

    /** Get the total profit of the items. */
    inline Weight total_profit() const { return total_profit_; }

    /*
     * Outputs
     */

    /** Print the instance. */
    void format(
            std::ostream& os,
            int verbosity_level = 1) const;

private:

    /*
     * Private methods
     */

    /** Constructor to build an instance manually. */
    Instance() { }

    /*
     * Private attributes
     */

    /** Items. */
    std::vector<Item> items_;

    /** Capacity. */
    Weight capacity_ = 0;

    /** Conflicts. */
    std::vector<Conflict> conflicts_;

    /*
     * Computed attributes
     */

    /** Total weight of the items. */
    Weight total_weight_ = 0;

    /** Total profit of the items. */
    Profit total_profit_ = 0;

    friend class InstanceBuilder;
};

}
