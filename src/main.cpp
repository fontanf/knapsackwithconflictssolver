#include "knapsackwithconflictssolver/instance_builder.hpp"

#include "knapsackwithconflictssolver/solution.hpp"
#include "knapsackwithconflictssolver/algorithms/greedy.hpp"
#include "knapsackwithconflictssolver/algorithms/sequential_decomposition.hpp"
#include "knapsackwithconflictssolver/algorithms/greedy_best.hpp"
#include "knapsackwithconflictssolver/algorithms/bounds.hpp"
#include "knapsackwithconflictssolver/algorithms/milp.hpp"
#include "knapsackwithconflictssolver/algorithms/milp_2.hpp"
#include "knapsackwithconflictssolver/algorithms/milp_3.hpp"
#include "knapsackwithconflictssolver/algorithms/lagrangian_relaxation.hpp"

#ifdef XPRESS_FOUND
#include "xprs.h"
#endif

#include <boost/program_options.hpp>

using namespace knapsackwithconflictssolver;

namespace po = boost::program_options;

void read_args(
        Parameters& parameters,
        const po::variables_map& vm)
{
    parameters.timer.set_sigint_handler();
    parameters.messages_to_stdout = true;
    if (vm.count("time-limit"))
        parameters.timer.set_time_limit(vm["time-limit"].as<double>());
    if (vm.count("verbosity-level"))
        parameters.verbosity_level = vm["verbosity-level"].as<int>();
    if (vm.count("log"))
        parameters.log_path = vm["log"].as<std::string>();
    parameters.log_to_stderr = vm.count("log-to-stderr");
    bool only_write_at_the_end = vm.count("only-write-at-the-end");
    if (!only_write_at_the_end) {

        std::string certificate_path;
        if (vm.count("certificate"))
            certificate_path = vm["certificate"].as<std::string>();

        std::string certificate_format;
        if (vm.count("certificate-format"))
            certificate_format = vm["certificate-format"].as<std::string>();

        std::string json_output_path;
        if (vm.count("output"))
            json_output_path = vm["output"].as<std::string>();

        parameters.new_solution_callback = [
            json_output_path,
            certificate_path,
            certificate_format](
                    const Output& output,
                    const std::string&)
        {
            if (!json_output_path.empty())
                output.write_json_output(json_output_path);
            if (!certificate_path.empty())
                output.solution.write(certificate_path, certificate_format);
        };
    }
}

Output run(
        const Instance& instance,
        const po::variables_map& vm)
{
    std::mt19937_64 generator(vm["seed"].as<Seed>());
    Solution solution = (vm.count("initial-solution"))?
        Solution(instance, vm["initial-solution"].as<std::string>()):
        Solution(instance);

    // Run algorithm.
    std::string algorithm = vm["algorithm"].as<std::string>();
    if (algorithm == "greedy") {
        Parameters parameters;
        read_args(parameters, vm);
        return greedy(instance, parameters);

    } else if (algorithm == "sequential-decomposition") {
        SequentialDecompositionParameters parameters;
        read_args(parameters, vm);
        if (vm.count("stable-weight-strategy")) {
            parameters.stable_weight_strategy
                = vm["stable-weight-strategy"].as<int>();
        }
        return sequential_decomposition(instance, parameters);

    } else if (algorithm == "greedy-best") {
        Parameters parameters;
        read_args(parameters, vm);
        return greedy_best(instance, parameters);

    } else if (algorithm == "fractional-knapsack-bound") {
        Parameters parameters;
        read_args(parameters, vm);
        return fractional_knapsack_bound(instance, parameters);
    } else if (algorithm == "binary-knapsack-bound") {
        Parameters parameters;
        read_args(parameters, vm);
        return binary_knapsack_bound(instance, parameters);
    } else if (algorithm == "fractional-multiple-choice-knapsack-bound") {
        Parameters parameters;
        read_args(parameters, vm);
        return fractional_multiple_choice_knapsack_bound(instance, parameters);
    } else if (algorithm == "binary-multiple-choice-knapsack-bound") {
        Parameters parameters;
        read_args(parameters, vm);
        return binary_multiple_choice_knapsack_bound(instance, parameters);

    } else if (algorithm == "milp") {
#ifdef XPRESS_FOUND
        XPRSinit(NULL);
#endif
        MilpParameters parameters;
        read_args(parameters, vm);
        if (vm.count("solver")) {
            parameters.solver
                = vm["solver"].as<mathoptsolverscmake::SolverName>();
        }
        auto milp_output = milp(instance, parameters);
#ifdef XPRESS_FOUND
        XPRSfree();
#endif
        return milp_output;
    } else if (algorithm == "milp-linear-relaxation") {
#ifdef XPRESS_FOUND
        XPRSinit(NULL);
#endif
        MilpParameters parameters;
        read_args(parameters, vm);
        auto milp_linear_reaxation_output = milp_linear_relaxation(instance, parameters);
#ifdef XPRESS_FOUND
        XPRSfree();
#endif
        return milp_linear_reaxation_output;

    } else if (algorithm == "milp-2") {
#ifdef XPRESS_FOUND
        XPRSinit(NULL);
#endif
        Milp2Parameters parameters;
        read_args(parameters, vm);
        if (vm.count("solver")) {
            parameters.solver
                = vm["solver"].as<mathoptsolverscmake::SolverName>();
        }
        auto milp_output = milp_2(instance, parameters);
#ifdef XPRESS_FOUND
        XPRSfree();
#endif
        return milp_output;
    } else if (algorithm == "milp-2-linear-relaxation") {
#ifdef XPRESS_FOUND
        XPRSinit(NULL);
#endif
        Milp2Parameters parameters;
        read_args(parameters, vm);
        auto milp_2_linear_reaxation_output = milp_2_linear_relaxation(instance, parameters);
#ifdef XPRESS_FOUND
        XPRSfree();
#endif
        return milp_2_linear_reaxation_output;

    } else if (algorithm == "milp-3") {
#ifdef XPRESS_FOUND
        XPRSinit(NULL);
#endif
        Milp3Parameters parameters;
        read_args(parameters, vm);
        if (vm.count("solver")) {
            parameters.solver
                = vm["solver"].as<mathoptsolverscmake::SolverName>();
        }
        auto milp_output = milp_3(instance, parameters);
#ifdef XPRESS_FOUND
        XPRSfree();
#endif
        return milp_output;
    } else if (algorithm == "milp-3-linear-relaxation") {
#ifdef XPRESS_FOUND
        XPRSinit(NULL);
#endif
        Milp3Parameters parameters;
        read_args(parameters, vm);
        auto milp_3_linear_reaxation_output = milp_3_linear_relaxation(instance, parameters);
#ifdef XPRESS_FOUND
        XPRSfree();
#endif
        return milp_3_linear_reaxation_output;

    } else if (algorithm == "lagrangian-relaxation") {
        LagrangianRelaxationParameters parameters;
        read_args(parameters, vm);
        return lagrangian_relaxation(instance, nullptr, parameters);

    } else {
        throw std::invalid_argument(
                "Unknown algorithm \"" + algorithm + "\".");
    }
    return Output(instance);
}

int main(int argc, char *argv[])
{
    // Parse program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("algorithm,a", po::value<std::string>()->default_value("local-search"), "set algorithm")
        ("input,i", po::value<std::string>()->required(), "set input file (required)")
        ("format,f", po::value<std::string>()->default_value(""), "set input file format (default: standard)")
        ("certificate-format,", po::value<std::string>()->default_value(""), "set certificate file format (default: standard)")
        ("output,o", po::value<std::string>(), "set JSON output file")
        ("initial-solution,", po::value<std::string>(), "")
        ("certificate,c", po::value<std::string>(), "set certificate file")
        ("goal,", po::value<Profit>(), "")
        ("seed,s", po::value<Seed>()->default_value(0), "set seed")
        ("time-limit,t", po::value<double>(), "set time limit in seconds")
        ("verbosity-level,v", po::value<int>(), "set verbosity level")
        ("only-write-at-the-end,e", "only write output and certificate files at the end")
        ("log,l", po::value<std::string>(), "set log file")
        ("log-to-stderr", "write log to stderr")

        ("stable-weight-strategy,", po::value<int>(), "set stable weight strategy (sequential-decomposition)")
        ("solver,", po::value<mathoptsolverscmake::SolverName>(), "set solver (milp)")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help")) {
        std::cout << desc << std::endl;;
        return 1;
    }
    try {
        po::notify(vm);
    } catch (const po::required_option& e) {
        std::cout << desc << std::endl;;
        return 1;
    }

    // Build instance.
    InstanceBuilder instance_builder;
    instance_builder.read(
            vm["input"].as<std::string>(),
            vm["format"].as<std::string>());
    const Instance instance = instance_builder.build();

    // Run.
    Output output = run(instance, vm);

    // Write outputs.
    if (vm.count("certificate")) {
        std::string certificate_format = "";
        if (vm.count("certificate-format"))
            certificate_format = vm["certificate-format"].as<std::string>();
        output.solution.write(
                vm["certificate"].as<std::string>(),
                certificate_format);
    }
    if (vm.count("output"))
        output.write_json_output(vm["output"].as<std::string>());

    return 0;
}
