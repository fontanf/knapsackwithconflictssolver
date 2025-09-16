#include "knapsackwithconflictssolver/instance.hpp"

#include <iomanip>

using namespace knapsackwithconflictssolver;

void Instance::format(
        std::ostream& os,
        int verbosity_level) const
{
    if (verbosity_level >= 1) {
        os
            << "Number of items:         " << this->number_of_items() << std::endl
            << "Capacity:                " << this->capacity() << std::endl
            << "Total weight:            " << this->total_weight() << std::endl
            << "Weight ratio:            " << (double)this->total_weight() / this->capacity() << std::endl
            << "Total profit:            " << this->total_profit() << std::endl
            << "Number of conflicts:     " << this->number_of_conflicts() << std::endl
            << "Average # of conflicts:  " << (double)this->number_of_conflicts() * 2 / this->number_of_items() << std::endl
            ;
    }

    // Print items.
    if (verbosity_level >= 2) {
        os << std::endl
            << std::setw(12) << "Item"
            << std::setw(12) << "Profit"
            << std::setw(12) << "Weight"
            << std::setw(12) << "Efficiency"
            << std::setw(12) << "# conflicts"
            << std::endl
            << std::setw(12) << "----"
            << std::setw(12) << "------"
            << std::setw(12) << "------"
            << std::setw(12) << "----------"
            << std::setw(12) << "-----------"
            << std::endl;
        for (ItemId item_id = 0; item_id < number_of_items(); ++item_id) {
            const Item& item = this->item(item_id);
            os
                << std::setw(12) << item_id
                << std::setw(12) << item.profit
                << std::setw(12) << item.weight
                << std::setw(12) << (double)item.profit / item.weight
                << std::setw(12) << item.neighbors.size()
                << std::endl;
        }
    }

    // Print conflicts.
    if (verbosity_level >= 3) {
        os << std::endl
            << std::setw(12) << "Conflict"
            << std::setw(12) << "Item 1"
            << std::setw(12) << "Item 2"
            << std::endl
            << std::setw(12) << "--------"
            << std::setw(12) << "------"
            << std::setw(12) << "------"
            << std::endl;
        for (ConflictId conflict_id = 0;
                conflict_id < this->number_of_conflicts();
                ++conflict_id) {
            const Conflict& conflict = this->conflict(conflict_id);
            os
                << std::setw(12) << conflict_id
                << std::setw(12) << conflict.item_1_id
                << std::setw(12) << conflict.item_2_id
                << std::endl;
        }
    }
}
