#include "knapsackwithconflictssolver/solution.hpp"

#include "optimizationtools/utils/utils.hpp"

#include <fstream>
#include <iomanip>

using namespace knapsackwithconflictssolver;

Solution::Solution(
        const Instance& instance,
        std::string certificate_path):
    Solution(instance)
{
    if (certificate_path.empty())
        return;
    std::ifstream file(certificate_path);
    if (!file.good()) {
        throw std::runtime_error(
                "Unable to open file \"" + certificate_path + "\".");
    }

    ItemId number_of_items;
    ItemId item_id;
    file >> number_of_items;
    for (ItemId pos = 0; pos < number_of_items; ++pos) {
        file >> item_id;
        add(item_id);
    }
}

void Solution::write(
        const std::string& certificate_path,
        const std::string& format) const
{
    if (certificate_path.empty())
        return;
    std::ofstream file(certificate_path);
    if (!file.good()) {
        throw std::runtime_error(
                "Unable to open file \"" + certificate_path + "\".");
    }

    if (format == "" || format == "standard") {
        write_standard(file);
    } else {
        throw std::invalid_argument(
                "Unknown instance format \"" + format + "\".");
    }
    file.close();
}

void Solution::write_standard(
        std::ostream& file) const
{
    file << number_of_items() << std::endl;
    for (ItemId item_id: items_set_)
        file << item_id << " ";
}

void Solution::format(
        std::ostream& os,
        int verbosity_level) const
{
    if (verbosity_level >= 1) {
        os
            << "Number of items:      " << optimizationtools::Ratio<ItemId>(number_of_items(), instance().number_of_items()) << std::endl
            << "Weight:               " << optimizationtools::Ratio<Weight>(weight(), instance().capacity()) << std::endl
            << "Number of conflicts:  " << number_of_conflicts() << std::endl
            << "Feasible:             " << feasible() << std::endl
            << "Profit:               " << profit() << std::endl
            ;
    }

    if (verbosity_level >= 2) {
        os << std::right << std::endl
            << std::setw(12) << "Item"
            << std::setw(12) << "Profit"
            << std::setw(12) << "Weight"
            << std::endl
            << std::setw(12) << "---"
            << std::setw(12) << "------"
            << std::setw(12) << "------"
            << std::endl;
        for (ItemId item_id: items_set_) {
            const knapsackwithconflictssolver::Item& item = instance().item(item_id);
            os
                << std::setw(12) << item_id
                << std::setw(12) << item.weight
                << std::setw(12) << item.profit
                << std::endl;
        }
    }
}

nlohmann::json Solution::to_json() const
{
    return nlohmann::json {
        {"NumberOfItems", number_of_items()},
        {"NumberOfConflicts", number_of_conflicts()},
        {"Weight", weight()},
        {"Feasible", feasible()},
        {"Profit", profit()},
    };
}
