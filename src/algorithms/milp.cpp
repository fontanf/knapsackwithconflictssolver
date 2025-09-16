#include "knapsackwithconflictssolver/algorithms/milp.hpp"

#include "knapsackwithconflictssolver/algorithm_formatter.hpp"

using namespace knapsackwithconflictssolver;

namespace
{

mathoptsolverscmake::MilpModel create_milp_model(
        const Instance& instance)
{
    int number_of_variables = instance.number_of_items();
    int number_of_constraints = 1 + instance.number_of_conflicts();
    int number_of_elements = instance.number_of_items() + 2 * instance.number_of_conflicts();

    mathoptsolverscmake::MilpModel model(
            number_of_variables,
            number_of_constraints,
            number_of_elements);

    // Variable and objective.
    model.objective_direction = mathoptsolverscmake::ObjectiveDirection::Maximize;
    for (ItemId item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        const Item& item = instance.item(item_id);
        model.variables_lower_bounds[item_id] = 0;
        model.variables_upper_bounds[item_id] = 1;
        model.variables_types[item_id] = mathoptsolverscmake::VariableType::Binary;
        model.objective_coefficients[item_id] = item.profit;
    }

    // Constraints.
    int element_id = 0;
    int constraints_id = 0;

    // Constraint: knapsack constraint.
    model.constraints_starts[constraints_id] = element_id;
    for (ItemId item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        const Item& item = instance.item(item_id);
        model.elements_variables[element_id] = item_id;
        model.elements_coefficients[element_id] = item.weight;
        element_id++;
    }
    model.constraints_upper_bounds[constraints_id] = instance.capacity();
    constraints_id++;

    // Constraints:conflict constraints.
    for (ItemId item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        const Item& item = instance.item(item_id);
        for (const ItemConflict& conflict: item.neighbors) {
            if (item_id > conflict.item_id)
                continue;
            model.constraints_starts[constraints_id] = element_id;

            model.elements_variables[element_id] = item_id;
            model.elements_coefficients[element_id] = 1.0;
            element_id++;

            model.elements_variables[element_id] = conflict.item_id;
            model.elements_coefficients[element_id] = 1.0;
            element_id++;

            model.constraints_upper_bounds[constraints_id] = 1;
            constraints_id++;
        }
    }

    return model;
}

Solution retrieve_solution(
        const Instance& instance,
        const std::vector<double>& milp_solution)
{
    Solution solution(instance);
    for (ItemId item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        if (milp_solution[item_id] > 0.5)
            solution.add(item_id);
    }
    return solution;
}

}

namespace
{

#ifdef CBC_FOUND

class EventHandler: public CbcEventHandler
{

public:

    virtual CbcAction event(CbcEvent which_event);

    EventHandler(
            const Instance& instance,
            const MilpParameters& parameters,
            MilpOutput& output,
            AlgorithmFormatter& algorithm_formatter):
        CbcEventHandler(),
        instance_(instance),
        parameters_(parameters),
        output_(output),
        algorithm_formatter_(algorithm_formatter) { }

    virtual ~EventHandler() { }

    EventHandler(const EventHandler &rhs):
        CbcEventHandler(rhs),
        instance_(rhs.instance_),
        parameters_(rhs.parameters_),
        output_(rhs.output_),
        algorithm_formatter_(rhs.algorithm_formatter_) { }

    virtual CbcEventHandler* clone() const { return new EventHandler(*this); }

private:

    const Instance& instance_;
    const MilpParameters& parameters_;
    MilpOutput& output_;
    AlgorithmFormatter& algorithm_formatter_;

};

CbcEventHandler::CbcAction EventHandler::event(CbcEvent which_event)
{
    // Not in subtree.
    if ((model_->specialOptions() & 2048) != 0)
        return noAction;
    const CbcModel& cbc_model = *model_;

    // Retrieve solution.
    double milp_objective_value = -mathoptsolverscmake::get_solution_value(cbc_model);
    if (output_.solution.profit() < milp_objective_value) {
        std::vector<double> milp_solution = mathoptsolverscmake::get_solution(cbc_model);
        Solution solution = retrieve_solution(instance_, milp_solution);
        algorithm_formatter_.update_solution(solution, "");
    }

    // Retrieve bound.
    double bound = -mathoptsolverscmake::get_bound(cbc_model);
    algorithm_formatter_.update_bound(bound, "");

    // Check end.
    if (parameters_.timer.needs_to_end())
        return stop;
    if (parameters_.goal >= 0
            && output_.solution.objective_value() >= parameters_.goal) {
        return stop;
    }

    return noAction;
}

#endif

#ifdef XPRESS_FOUND

struct XpressCallbackUser
{
    const Instance& instance;
    const MilpParameters& parameters;
    MilpOutput& output;
    AlgorithmFormatter& algorithm_formatter;
};

void xpress_callback(
        XPRSprob xpress_model,
        void* user,
        int*)
{
    const XpressCallbackUser& d = *(const XpressCallbackUser*)(user);

    // Retrieve solution.
    double milp_objective_value = mathoptsolverscmake::get_solution_value(xpress_model);
    if (d.output.solution.profit() < milp_objective_value) {
        std::vector<double> milp_solution = mathoptsolverscmake::get_solution(xpress_model);
        Solution solution = retrieve_solution(d.instance, milp_solution);
        d.algorithm_formatter.update_solution(solution, "");
    }

    // Retrieve bound.
    double bound = mathoptsolverscmake::get_bound(xpress_model);
    d.algorithm_formatter.update_bound(bound, "");

    // Check end.
    if (d.parameters.timer.needs_to_end())
        XPRSinterrupt(xpress_model, XPRS_STOP_USER);
    if (d.parameters.goal >= 0
            && d.output.solution.objective_value() >= d.parameters.goal) {
        XPRSinterrupt(xpress_model, XPRS_STOP_USER);
    }
};

#endif

}

MilpOutput knapsackwithconflictssolver::milp(
        const Instance& instance,
        const MilpParameters& parameters)
{
    MilpOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("MILP");

    algorithm_formatter.print_header();

    mathoptsolverscmake::MilpModel milp_model = create_milp_model(instance);
    std::vector<double> milp_solution;
    double milp_bound = instance.total_profit();

    if (parameters.solver == mathoptsolverscmake::SolverName::Cbc) {
#ifdef CBC_FOUND
        OsiCbcSolverInterface osi_solver;
        CbcModel cbc_model(osi_solver);
        mathoptsolverscmake::reduce_printout(cbc_model);
        mathoptsolverscmake::set_time_limit(cbc_model, parameters.timer.remaining_time());
        mathoptsolverscmake::load(cbc_model, milp_model);
        EventHandler cbc_event_handler(instance, parameters, output, algorithm_formatter);
        cbc_model.passInEventHandler(&cbc_event_handler);
        mathoptsolverscmake::solve(cbc_model);
        milp_solution = mathoptsolverscmake::get_solution(cbc_model);
        milp_bound = mathoptsolverscmake::get_bound(cbc_model);
#else
        throw std::invalid_argument("");
#endif

    } else if (parameters.solver == mathoptsolverscmake::SolverName::Highs) {
#ifdef HIGHS_FOUND
        Highs highs;
        mathoptsolverscmake::reduce_printout(highs);
        mathoptsolverscmake::set_time_limit(highs, parameters.timer.remaining_time());
        mathoptsolverscmake::set_log_file(highs, "highs.log");
        mathoptsolverscmake::load(highs, milp_model);
        highs.setCallback([
                &instance,
                &parameters,
                &output,
                &algorithm_formatter](
                    const int,
                    const std::string& message,
                    const HighsCallbackOutput* highs_output,
                    HighsCallbackInput* highs_input,
                    void*)
                {
                    // Retrieve solution.
                    double milp_objective_value = highs_output->mip_primal_bound;
                    if (output.solution.profit() < milp_objective_value) {
                        Solution solution = retrieve_solution(instance, highs_output->mip_solution);
                        algorithm_formatter.update_solution(solution, "");
                    }

                    // Retrieve bound.
                    double bound = highs_output->mip_dual_bound;
                    if (bound != std::numeric_limits<double>::infinity())
                        algorithm_formatter.update_bound(bound, "");

                    // Check end.
                    if (parameters.timer.needs_to_end())
                        highs_input->user_interrupt = 1;
                    if (parameters.goal >= 0
                            && output.solution.objective_value() >= parameters.goal) {
                        highs_input->user_interrupt = 1;
                    }
                },
                nullptr);
        HighsStatus highs_status = highs.startCallback(HighsCallbackType::kCallbackMipImprovingSolution);
        mathoptsolverscmake::solve(highs);
        milp_solution = mathoptsolverscmake::get_solution(highs);
        milp_bound = mathoptsolverscmake::get_bound(highs);
#else
        throw std::invalid_argument("");
#endif

    } else if (parameters.solver == mathoptsolverscmake::SolverName::Xpress) {
#ifdef XPRESS_FOUND
        XPRSprob xpress_model;
        XPRScreateprob(&xpress_model);
        mathoptsolverscmake::set_time_limit(xpress_model, parameters.timer.remaining_time());
        mathoptsolverscmake::set_log_file(xpress_model, "xpress.log");
        mathoptsolverscmake::load(xpress_model, milp_model);
        //mathoptsolverscmake::write_mps(xpress_model, "kpc.mps");
        XpressCallbackUser xpress_callback_user{instance, parameters, output, algorithm_formatter};
        XPRSaddcbprenode(xpress_model, xpress_callback, (void*)&xpress_callback_user, 0);
        mathoptsolverscmake::solve(xpress_model);
        milp_solution = mathoptsolverscmake::get_solution(xpress_model);
        milp_bound = mathoptsolverscmake::get_bound(xpress_model);
        XPRSdestroyprob(xpress_model);
#else
        throw std::invalid_argument("");
#endif

    } else {
        throw std::invalid_argument("");
    }

    // Retrieve solution.
    Solution solution = retrieve_solution(instance, milp_solution);
    algorithm_formatter.update_solution(solution, "");

    // Retrieve bound.
    algorithm_formatter.update_bound(milp_bound, "");

    algorithm_formatter.end();
    return output;
}
