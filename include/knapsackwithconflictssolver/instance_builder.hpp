#pragma once

#include "knapsackwithconflictssolver/instance.hpp"

#include <fstream>

namespace knapsackwithconflictssolver
{

class InstanceBuilder
{

public:

    /** Constructor. */
    InstanceBuilder() { }

    /** Add an item. */
    void add_item(
            Weight weight,
            Profit profit)
    {
        Item item;
        item.weight = weight;
        item.profit = profit;
        instance_.items_.push_back(item);
    }

    /** Set the weight of an item. */
    void set_weight(
            ItemId item_id,
            Weight weight)
    {
        instance_.items_[item_id].weight = weight;
    }

    /** Set the profit of an item. */
    void set_profit(
            ItemId item_id,
            Profit profit)
    {
        instance_.items_[item_id].profit = profit;
    }

    /** Add a conflict between two items. */
    ConflictId add_conflict(
            ItemId item_1_id,
            ItemId item_2_id)
    {
        ConflictId conflict_id = instance_.conflicts_.size();
        instance_.items_[item_1_id].neighbors.push_back({conflict_id, item_2_id});
        instance_.items_[item_2_id].neighbors.push_back({conflict_id, item_1_id});
        instance_.conflicts_.push_back({item_1_id, item_2_id});
        return conflict_id;
    }

    /** Set the capacity of the knapsack. */
    void set_capacity(Weight capacity) { instance_.capacity_ = capacity; }

    /** Build an instance from a file. */
    void read(
            const std::string& instance_path,
            const std::string& format = "");

    /*
     * Build
     */

    /** Build the instance. */
    Instance build();

private:

    /*
     * Private methods
     */

    /** Read an instance from a file in 'hifi2006' format. */
    void read_hifi2006(std::ifstream& file);

    /** Read an instance from a file in 'bettinelli2017' format. */
    void read_bettinelli2017(std::ifstream& file);

    /*
     * Private attributes
     */

    /** Instance. */
    Instance instance_;

};

}
