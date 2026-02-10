#include "driver/beast_usb2can.h"
#include "AJAC/Actuator_Config.h"
#include <random>
#include "AJAC/AJAC.h"

constexpr uint16_t beast_usb2can_vendor_id = 0x1111;
constexpr uint16_t beast_usb2can_product_id = 0x2222;
constexpr uint8_t motors_ep_in = 0x81;
constexpr uint8_t motors_ep_out = 0x01;
USB_Cmd_T *usb_cmd_from_controller;
USB_Data_T *usb_data_to_controller;
AJAC *ajac_handle;
double sign = 1;

constexpr double peak = 30.;
int peak_iter = 0;

void control_motors() {
    static int iter;
    static int start;

    if (start && iter > 1000) {
        // Step input
        if (iter % 1500 == 0) {
            if (sign > 0)
                sign = -1;
            else if (sign < 0)
                sign = 1;
        }
        const double fake_ctrl_tff = peak * sign;
        const double q = usb_data_to_controller->usb_chip_data_[0].data_pack[0].p_data_;
        const double uq = usb_data_to_controller->usb_chip_data_[0].data_pack[0].uq_;
        const double ud = usb_data_to_controller->usb_chip_data_[0].data_pack[0].ud_;
        const double qd = usb_data_to_controller->usb_chip_data_[0].data_pack[0].v_data_;
        const double tau = usb_data_to_controller->usb_chip_data_[0].data_pack[0].t_data_;
        // high-level encoder for experimental convenience
        ajac_handle->inside_encoder(q);

        ajac_handle->update_data(qd, uq, ud, tau / MOTOR_8115::kt, 0.);
        const double clipped_tau_vats = ajac_handle->clip_tau_by_ajac(fake_ctrl_tff); // cliped by VATS
        // const double clipped_tau_mor = vats_handle->clip_tau_by_MOR(qd, fake_ctrl_tff); // clipped by MOR
        usb_cmd_from_controller->usb_chip_cmd_[0].cmd_pack[0].t_ff_ = static_cast<float>(clipped_tau_vats);
        usb_cmd_from_controller->usb_chip_cmd_[1].cmd_pack[0].p_cmd_ = static_cast<float>(ajac_handle->vbac_.ub_[0]);
        usb_cmd_from_controller->usb_chip_cmd_[1].cmd_pack[0].v_cmd_ = static_cast<float>(ajac_handle->vbac_.ub_[1]);
        usb_cmd_from_controller->usb_chip_cmd_[1].cmd_pack[0].kp_ = static_cast<float>(ajac_handle->vbac_.ub_[2]);
        usb_cmd_from_controller->usb_chip_cmd_[1].cmd_pack[0].kd_ = static_cast<float>(ajac_handle->vbac_.ub_[3]);
        usb_cmd_from_controller->usb_chip_cmd_[1].cmd_pack[0].t_ff_ = static_cast<float>(ajac_handle->ddq_max_k_vats_);
        usb_cmd_from_controller->usb_chip_cmd_[1].cmd_pack[1].p_cmd_ = static_cast<float>(ajac_handle->inside_encoder_);
        usb_cmd_from_controller->usb_chip_cmd_[1].cmd_pack[1].v_cmd_ = static_cast<float>(ajac_handle->st_tau_lb_k_);
        usb_cmd_from_controller->usb_chip_cmd_[1].cmd_pack[1].kp_ = static_cast<float>(ajac_handle->st_tau_ub_k_);
        usb_cmd_from_controller->usb_chip_cmd_[1].cmd_pack[1].kd_ = static_cast<float>(sqrtf(uq * uq + ud * ud));
        usb_cmd_from_controller->usb_chip_cmd_[1].cmd_pack[1].t_ff_ = static_cast<float>(fake_ctrl_tff);
        usb_cmd_from_controller->usb_chip_cmd_[1].cmd_pack[2].p_cmd_ = static_cast<float>(ajac_handle->tr_tau_lb_k_);
        usb_cmd_from_controller->usb_chip_cmd_[1].cmd_pack[2].v_cmd_ = static_cast<float>(ajac_handle->tr_tau_ub_k_);
    }
    iter++;
    if (iter < 100) {
        usb_cmd_from_controller->usb_chip_cmd_[0].chip_flag[0] = (0x01 << 1);
    } else {
        usb_cmd_from_controller->usb_chip_cmd_[0].chip_flag[0] = (0x01) | (0x01 << 2);
        start = 1;
    }
    // if (iter == 3500)
    // std::exit(EXIT_SUCCESS);
}

[[noreturn]] int main() {
    usb_cmd_from_controller = new USB_Cmd_T();
    usb_data_to_controller = new USB_Data_T();
    motor_typ motor_8115;
    motor_8115.name = "motor_8115";
    motor_8115.flux_linkage_ = MOTOR_8115::flux_linkage;
    motor_8115.gr_ = MOTOR_8115::gr;
    motor_8115.ls_ = MOTOR_8115::Ls;
    motor_8115.rs_ = MOTOR_8115::Rs;
    motor_8115.v_bound_ = MOTOR_8115::v_bound;
    motor_8115.rotor_inertia_ = MOTOR_8115::rotor_inertia;
    motor_8115.npp = MOTOR_8115::npp;
    motor_8115.kt_ = MOTOR_8115::kt;
    motor_8115.step_ = MOTOR_8115::step;
    motor_8115.control_dt_ = MOTOR_8115::control_dt;

    // motor_typ motor_6210;
    // motor_6210.name = MOTOR_6210::name;
    // motor_6210.flux_linkage_ = MOTOR_6210::flux_linkage;
    // motor_6210.gr_ = MOTOR_6210::gr;
    // motor_6210.ls_ = MOTOR_6210::Ls;
    // motor_6210.rs_ = MOTOR_6210::Rs;
    // motor_6210.v_bound_ = MOTOR_6210::v_bound;
    // motor_6210.rotor_inertia_ = MOTOR_6210::rotor_inertia;
    // motor_6210.npp = MOTOR_6210::npp;
    // motor_6210.kt_ = MOTOR_6210::kt;
    // motor_6210.step_ = MOTOR_6210::step;
    // motor_6210.control_dt_ = MOTOR_6210::control_dt;

    ajac_handle = new AJAC(motor_8115);
    ajac_handle->set_kinematic_constraints(VBAC::q_max, VBAC::q_min, VBAC::dq_max, VBAC::ddq_max_default, VBAC::delta_t);
    // ajac_handle->set_kinematic_only();
    ajac_handle->set_ajac();

    int complete = 0;
    timeval timestru{};
    timestru.tv_sec = 0;
    timestru.tv_usec = 1000;

    Beast_USB2CAN spUSB2CAN(beast_usb2can_vendor_id, //
                            beast_usb2can_product_id,
                            motors_ep_in,
                            motors_ep_out);
    spUSB2CAN.USB2CAN_SetBuffer(usb_cmd_from_controller, usb_data_to_controller);
    spUSB2CAN.register_ctrl_func(control_motors);
    spUSB2CAN.set_write_variable();
    spUSB2CAN.set_lcm_publish();
    spUSB2CAN.start_transfer_ansy();

    while (true) {
        libusb_handle_events_timeout_completed(spUSB2CAN.ctx, &timestru, &complete);
    }
}
