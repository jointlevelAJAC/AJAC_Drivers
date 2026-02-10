#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H
#include <string>
#include <cmath>

namespace MOTOR_8115 {
    const std::string name = "MOTOR_8115";
    constexpr double rotor_inertia = 0.007;
    constexpr double Ls = 0.000105;
    constexpr double Rs = 0.1625;
    constexpr double flux_linkage = 0.005356815;
    constexpr double gr = 9.75;
    constexpr double alpha = 0.9; //recommended value
    constexpr double Input_source = 36; // 24v input
    constexpr double v_bound = alpha * Input_source * 0.56; //TODO fix this with dynamic bounds, for now slightly smaller than 1/sqrt(3) [SVPWM]
    constexpr double kt = 1.346148; //NOTE: this kt directly corresponds to iq, depending on the omp. gain and gr.
    constexpr double npp = 21;
    constexpr double step = 10.0;
    constexpr double control_dt = 0.001; // real control dt
}

namespace MOTOR_6210{
    const std::string name = "MOTOR_6210";
    constexpr double rotor_inertia = 0.003;
    constexpr double Ls = 0.000108832;
    constexpr double Rs = 0.1423715;
    constexpr double flux_linkage = 0.005257336;
    constexpr double gr = 10.75;
    constexpr double alpha = 0.9; //recommended value
    constexpr double Input_source = 24; // 24v input
    constexpr double v_bound = alpha * Input_source * 0.56; //TODO fix this with dynamic bounds, for now slightly smaller than 1/sqrt(3) [SVPWM]
    constexpr double kt = 0.939298115; //NOTE: this kt directly corresponds to iq, depending on the omp. gain and gr.
    constexpr double npp = 14;
    constexpr double step = 10.;
    constexpr double control_dt = 0.001; // real control dt
}

namespace VBAC {
    constexpr double delta_t = 0.05; // increase delta_t to increase the robustness
    constexpr double q_max = 2. * M_PI;
    constexpr double q_min = -2. * M_PI;
    constexpr double dq_max = 180000000.; // mor tune this parameter to adjust the vel limits
    // constexpr double dq_max = 10; // mor tune this parameter to adjust the vel limits
    // constexpr double dq_max = 180000; // vats
    constexpr double ddq_max_default = 6428.6; // 40 / 0.007 = 6428.6 (8115) aggressive one
    // constexpr double ddq_max_default = 3200.; // conservative one
}

// magic number
constexpr double inertia_gain_z = 4.;


#endif //MOTOR_CONFIG_H
