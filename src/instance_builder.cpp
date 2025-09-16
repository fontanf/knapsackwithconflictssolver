#include "knapsackwithconflictssolver/instance_builder.hpp"

using namespace knapsackwithconflictssolver;

void InstanceBuilder::read(
        const std::string& instance_path,
        const std::string& format)
{
    std::ifstream file(instance_path);
    if (!file.good()) {
        throw std::runtime_error(
                "Unable to open file \"" + instance_path + "\".");
    }
    if (format == "" || format == "default" || format == "hifi2006") {
        read_hifi2006(file);
    } else if (format == "bettinelli2017") {
        read_bettinelli2017(file);
    } else {
        throw std::invalid_argument(
                "Unknown instance format \"" + format + "\".");
    }
    file.close();
}

void InstanceBuilder::read_hifi2006(std::ifstream& file)
{
    ItemId number_of_items = -1;
    file >> number_of_items;

    ConflictId number_of_conflicts = -1;
    file >> number_of_conflicts;

    Weight capacity = -1;
    file >> capacity;
    set_capacity(capacity);

    Profit profit = -1;
    for (ItemId item_id = 0; item_id < number_of_items; ++item_id) {
        file >> profit;
        add_item(0, profit);
    }

    Weight weight = -1;
    for (ItemId item_id = 0; item_id < number_of_items; ++item_id) {
        file >> weight;
        set_weight(item_id, weight);
    }

    ItemId item_1_id = -1;
    ItemId item_2_id = -1;
    for (ConflictId conflict_id = 0; conflict_id < number_of_conflicts; ++conflict_id) {
        file >> item_1_id >> item_2_id;
        add_conflict(item_1_id - 1, item_2_id - 1);
    }
}

void InstanceBuilder::read_bettinelli2017(std::ifstream& file)
{
    ItemId number_of_items = -1;
    Weight capacity = -1;
    std::string tmp;
    file >> tmp >> tmp >> tmp >> number_of_items >> tmp;
    file >> tmp >> tmp >> tmp >> capacity >> tmp;
    set_capacity(capacity);
    if (tmp == ";")
        file >> tmp;
    file >> tmp >> tmp >> tmp >> tmp >> tmp >> tmp;

    Weight weight = -1;
    Profit profit = -1;
    for (ItemId item_id = 0; item_id < number_of_items; ++item_id) {
        file >> tmp >> profit >> weight;
        add_item(weight, profit);
    }

    file >> tmp >> tmp >> tmp >> tmp;
    ItemId item_1_id = -1;
    ItemId item_2_id = -1;
    for (;;) {
        file >> item_1_id >> item_2_id;
        if (!file)
            break;
        add_conflict(item_1_id, item_2_id);
    }
}

Instance InstanceBuilder::build()
{
    // Compute total weight.
    instance_.total_weight_ = 0;
    for (ItemId item_id = 0;
            item_id < instance_.number_of_items();
            ++item_id) {
        const Item& item = instance_.item(item_id);
        instance_.total_weight_ += item.weight;
        instance_.total_profit_ += item.profit;
    }

    return std::move(instance_);
}
