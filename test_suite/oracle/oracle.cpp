// Bit-exact comparison of rsruckig with upstream Ruckig (https://github.com/pantor/ruckig).
//
// 1. Dump the random inputs and results of rsruckig:
//      RSRUCKIG_ORACLE_DUMP=/tmp/dump.txt cargo test --release --test tests_random
// 2. Build this program against an upstream checkout and feed it the dump:
//      g++ -std=c++20 -O2 -I<ruckig>/include oracle.cpp \
//          $(ls <ruckig>/src/ruckig/*.cpp | grep -v cloud) -o oracle
//      ./oracle < /tmp/dump.txt
//    The optional argument is the number of mismatches of each kind to print
//    (default 20).
//
// Every line of the dump holds: DoFs, control interface (0 position, 1
// velocity), synchronization, minimum duration (negative: none), then per DoF
// the 11 values p0 v0 a0 pf vf af v_max a_max j_max v_min a_min, and finally
// the result code and duration of rsruckig. All floating-point numbers are the hex bit patterns of doubles.

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include <ruckig/ruckig.hpp>

using namespace ruckig;

static double read_double(std::istringstream& stream) {
    std::string token;
    stream >> token;
    const uint64_t bits = std::stoull(token, nullptr, 16);
    double value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

int main(int argc, char** argv) {
    const size_t print_limit = (argc > 1) ? std::strtoull(argv[1], nullptr, 10) : 20;
    size_t total {0}, result_mismatches {0}, bit_mismatches {0}, large_mismatches {0};
    double max_difference {0.0};

    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream stream {line};
        size_t dofs;
        int interface, synchronization;
        stream >> dofs >> interface >> synchronization;
        const double minimum_duration = read_double(stream);

        Ruckig<DynamicDOFs> otg {dofs, 0.005};
        InputParameter<DynamicDOFs> input {dofs};
        Trajectory<DynamicDOFs> trajectory {dofs};

        input.control_interface = interface == 1 ? ControlInterface::Velocity : ControlInterface::Position;
        input.synchronization = static_cast<Synchronization>(synchronization);
        if (minimum_duration >= 0.0) {
            input.minimum_duration = minimum_duration;
        }
        input.min_velocity = std::vector<double>(dofs);
        input.min_acceleration = std::vector<double>(dofs);
        for (size_t dof = 0; dof < dofs; ++dof) {
            input.current_position[dof] = read_double(stream);
            input.current_velocity[dof] = read_double(stream);
            input.current_acceleration[dof] = read_double(stream);
            input.target_position[dof] = read_double(stream);
            input.target_velocity[dof] = read_double(stream);
            input.target_acceleration[dof] = read_double(stream);
            input.max_velocity[dof] = read_double(stream);
            input.max_acceleration[dof] = read_double(stream);
            input.max_jerk[dof] = read_double(stream);
            (*input.min_velocity)[dof] = read_double(stream);
            (*input.min_acceleration)[dof] = read_double(stream);
        }
        int expected_result;
        stream >> expected_result;
        const double expected_duration = read_double(stream);

        const int result = static_cast<int>(otg.calculate(input, trajectory));
        const double duration = result == 0 ? trajectory.get_duration() : 0.0;

        total += 1;
        if (result != expected_result) {
            result_mismatches += 1;
            if (result_mismatches <= print_limit) {
                std::printf("RESULT upstream %d rsruckig %d (duration %.17g vs %.17g): %s\n", result, expected_result, duration, expected_duration, line.c_str());
            }
        } else if (std::memcmp(&duration, &expected_duration, sizeof(double)) != 0) {
            bit_mismatches += 1;
            const double difference = std::abs(duration - expected_duration);
            max_difference = std::max(max_difference, difference);
            if (difference > 1e-9 * std::max(1.0, std::abs(duration))) {
                large_mismatches += 1;
                if (large_mismatches <= print_limit) {
                    std::printf("DURATION upstream %.17g rsruckig %.17g: %s\n", duration, expected_duration, line.c_str());
                }
            }
        }
    }

    std::printf("%zu trajectories: %zu result mismatches, %zu durations not bit-identical (%zu differ by more than 1e-9 relative, largest difference %.3g)\n",
        total, result_mismatches, bit_mismatches, large_mismatches, max_difference);
    return (result_mismatches > 0 || large_mismatches > 0) ? 1 : 0;
}
