#ifndef MOTOR_SAFE_CTRL_H
#define MOTOR_SAFE_CTRL_H
#include <map>
#include <string>

// Constants for motor 8115
constexpr double battery_8115 = 36;
constexpr double GR_8115 = 9.75;
constexpr double Phi_8115 = 0.00147095;
constexpr double nominal_torque_8115 = 25; // Nm
constexpr double Npp_8115 = 21;
constexpr double Inverse_KT_OUT_8115 = 1.8488;
constexpr double Rs_8115 = 0.1625;
constexpr double Ls_8115 = 0.000105;
constexpr double K_Sim_8115 = 66;
constexpr double J_8115 = 0.0007;

typedef enum {
    nothing = 0,
    motor_8115 = 1,
} Motor_Type;

class Motor_Torque_Bound {
public:
    explicit Motor_Torque_Bound(const double working_voltage, Motor_Type motor_type) {
        if (motor_type == motor_8115) {
            motor_parameters_["battery_voltage"] = working_voltage;
            motor_parameters_["nominal_torque"] = nominal_torque_8115;
            motor_parameters_["gear_ratio"] = GR_8115;
            motor_parameters_["phi_m"] = Phi_8115;
            motor_parameters_["Npp"] = Npp_8115;
            motor_parameters_["inverse_KT_OUT"] = Inverse_KT_OUT_8115;
            motor_parameters_["Rs"] = Rs_8115;
            motor_parameters_["Ls"] = Ls_8115;
            motor_parameters_["K_Sim"] = K_Sim_8115;
            motor_parameters_["Inertial"] = J_8115;
            motor_parameters_["alpha"] = Rs_8115 / Ls_8115 * K_Sim_8115 * Phi_8115;
            motor_parameters_["beta"] = (Npp_8115 + K_Sim_8115) * Phi_8115;
        }
    }

    ~Motor_Torque_Bound() = default;

    void get_torque_cmd();

    double run_saturate_torque_reference();

private:
    std::map<std::string, double> motor_parameters_;
};

#endif
