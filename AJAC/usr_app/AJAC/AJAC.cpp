#include "AJAC.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <ostream>
#include <utility>
#include <algorithm>
#include "Actuator_Config.h"

AJAC::AJAC(motor_typ motor): motor_(std::move(motor)) {
    std::cout << "*******************************************\n"
            "VATS Motor Parameter: \n" <<
            motor_.name << "\n" <<
            "GR = " << motor_.gr_ << "\n" <<
            "control dt: " << motor_.control_dt_ << "\n" <<
            "V limit: " << motor_.v_bound_ << " V\n" <<
            "npp: " << motor_.npp << "\n" <<
            "step: " << motor_.step_ << " steps/rev\n" <<
            "kt: " << motor_.kt_ << " Nm/A\n";
    std::cout << "*******************************************\n";
    steps_ = motor_.step_;
    vbac_.lb_.resize(4);
    vbac_.ub_.resize(4);
}

/// For convinience, state k predict state k+1
/// @param dq flange speed
/// @param uq q-axis voltage
/// @param ud d-axis voltage
/// @param iq tau/kt
/// @param tau_ff tau feedforward from the controller, normally equals to -C(q,\dot{q})+J_c^Tf
void AJAC::update_data(const double dq, const double uq, const double ud, const double iq,
                       const double tau_ff) {
    // Update motor data from usb_data_to_controller
    dq_k_ = dq;
    uq_k_ = uq;
    ud_k_ = ud;
    dq_k_ = dq;
    tau_ff_k_ = tau_ff;
    iq_k_ = iq;

    // update res voltage first
    double res_q_filter = 0.8;
    double res_ud_k_filter = 0.8;
    const double omega_e_k_ = dq * motor_.gr_ * motor_.npp;
    res_uq_k_ = (uq - (motor_.rs_ * iq + omega_e_k_ * motor_.flux_linkage_)) *
                res_q_filter + (1 - res_q_filter) * res_uq_k_1_;
    res_ud_k_ = (ud - (-omega_k_1_ * motor_.ls_ * iq)) * res_ud_k_filter + (1 - res_ud_k_filter) * res_ud_k_1_;
    // res_ud_k_ = 0;
    uq_k1 = res_uq_k_ + (motor_.rs_ * iq + omega_e_k_ * motor_.flux_linkage_);
    ud_k1 = res_ud_k_ + (-omega_e_k_ * motor_.ls_ * iq);

    Ik_transient();
    Ik_steady_predict();

    fi_tau_lb_k_ = st_tau_lb_k_ > tr_tau_lb_k_ ? st_tau_lb_k_ : tr_tau_lb_k_;
    fi_tau_ub_k_ = st_tau_ub_k_ < tr_tau_ub_k_ ? st_tau_ub_k_ : tr_tau_ub_k_;

    res_uq_k_1_ = res_uq_k_;
    res_ud_k_1_ = res_ud_k_;
}

void AJAC::Ik_transient() {
    // let ud as feetforward
    const double omega_e_k_ = dq_k_ * motor_.gr_ * motor_.npp;
    // std::cout << "omega_e_k_: " << omega_e_k_ << std::endl;
    const double m = motor_.rs_ + motor_.ls_ / motor_.control_dt_;
    // std::cout << "m: " << m << std::endl;
    const double n = omega_e_k_ * motor_.flux_linkage_ - motor_.ls_ / motor_.control_dt_ * iq_k_ +
                     res_uq_k_;
    // std::cout << "n: " << n << std::endl;
    const double p = -omega_e_k_ * motor_.ls_;
    // std::cout << "p: " << p << std::endl;

    const double epsilon = (m * m + p * p) * motor_.v_bound_ * motor_.v_bound_ + 2 * m * n * p * res_ud_k_ -
                           p * p * n * n - m * m * res_ud_k_ * res_ud_k_;
    if (epsilon >= 0) {
        tr_iq_lb_k_ = (-(m * n + p * res_ud_k_) - std::sqrt(epsilon)) / (m * m + p * p);
        tr_iq_ub_k_ = (-(m * n + p * res_ud_k_) + std::sqrt(epsilon)) / (m * m + p * p);
    } else {
        tr_iq_lb_k_ = -(m * n + p * res_ud_k_) / (m * m + p * p);
        tr_iq_ub_k_ = -(m * n + p * res_ud_k_) / (m * m + p * p);
    }

    constexpr double asym_filter = 0.5;
    if (tr_iq_ub_k_ > tr_iq_ub_k_1_) {
        tr_iq_ub_k_ = tr_iq_ub_k_ * asym_filter + (1 - asym_filter) * tr_iq_ub_k_1_;
    }
    if (tr_iq_lb_k_ < tr_iq_lb_k_1_) {
        tr_iq_lb_k_ = tr_iq_lb_k_ * asym_filter + (1 - asym_filter) * tr_iq_lb_k_1_;
    }

    constexpr double dead_zone = 0.0;
    // std::cerr << "iq bounds[LB/UB]: " << iq_lb_k_ << " " << iq_ub_k_ << std::endl;
    tr_iq_lb_k_ = tr_iq_lb_k_ > -dead_zone ? -dead_zone : tr_iq_lb_k_;
    tr_iq_ub_k_ = tr_iq_ub_k_ < dead_zone ? dead_zone : tr_iq_ub_k_;

    tr_tau_lb_k_ = tr_iq_lb_k_ * motor_.kt_;
    tr_tau_ub_k_ = tr_iq_ub_k_ * motor_.kt_;

    omega_k_1_ = omega_e_k_;
    tr_iq_lb_k_1_ = tr_iq_lb_k_;
    tr_iq_ub_k_1_ = tr_iq_ub_k_;
}

void AJAC::Ik_steady_predict() {
    // mapping the real acc
    const double acc_constant = (motor_.gr_ * motor_.npp * motor_.control_dt_ *
                                 motor_.flux_linkage_) / (motor_.rotor_inertia_* inertia_gain_z);
    const double omega_e_k_ = dq_k_ * motor_.gr_ * motor_.npp;
    const double m = motor_.rs_ + motor_.kt_ * acc_constant * steps_;
    const double n = omega_e_k_ * motor_.flux_linkage_ - tau_ff_k_ * acc_constant * steps_ + res_uq_k_;
    const double p = -omega_e_k_ * motor_.ls_;

    const double epsilon = (m * m + p * p) * motor_.v_bound_ * motor_.v_bound_ + 2 * m * n * p * res_ud_k_ -
                           p * p * n * n - m * m * res_ud_k_ * res_ud_k_;

    if (epsilon >= 0) {
        st_iq_lb_k_ = (-(m * n + p * res_ud_k_) - std::sqrt(epsilon)) / (m * m + p * p);
        st_iq_ub_k_ = (-(m * n + p * res_ud_k_) + std::sqrt(epsilon)) / (m * m + p * p);
    } else {
        // std::cerr << "no solution! " << std::endl;
        st_iq_lb_k_ = -(m * n + p * res_ud_k_) / (m * m + p * p);
        st_iq_ub_k_ = -(m * n + p * res_ud_k_) / (m * m + p * p);
    }

    constexpr double asym_filter = 0.5;
    if (st_iq_ub_k_ > st_iq_ub_k_1_) {
        st_iq_ub_k_ = st_iq_ub_k_ * asym_filter + (1 - asym_filter) * st_iq_ub_k_1_;
    }
    if (st_iq_lb_k_ < st_iq_lb_k_1_) {
        st_iq_lb_k_ = st_iq_lb_k_ * asym_filter + (1 - asym_filter) * st_iq_lb_k_1_;
    }

    constexpr double dead_zone = 0.0;
    st_iq_lb_k_ = st_iq_lb_k_ > -dead_zone ? -dead_zone : st_iq_lb_k_;
    st_iq_ub_k_ = st_iq_ub_k_ < dead_zone ? dead_zone : st_iq_ub_k_;

    st_tau_lb_k_ = st_iq_lb_k_ * motor_.kt_;
    st_tau_ub_k_ = st_iq_ub_k_ * motor_.kt_;

    // note: the ddq_max for Kinematic Envelope is the declaration acc
    if (dq_k_ >= 0) {
        ddq_max_k_vats_ = (std::fabs(st_tau_lb_k_ - tau_ff_k_)) / motor_.rotor_inertia_;
    } else {
        ddq_max_k_vats_ = (std::fabs(st_tau_ub_k_ - tau_ff_k_)) / motor_.rotor_inertia_;
    }
    omega_k_1_ = omega_e_k_;
    st_iq_lb_k_1_ = st_iq_lb_k_;
    st_iq_ub_k_1_ = st_iq_ub_k_;
}

void AJAC::update_kinematic_limit() {
    const double omega_e_k_ = dq_k_ * motor_.gr_ * motor_.npp;

    // let ud as feetforward
    const double m = motor_.rs_;
    const double n = omega_e_k_ * motor_.flux_linkage_ + res_uq_k_;
    const double p = -omega_e_k_ * motor_.ls_;
    const double V = motor_.v_bound_;
    const double epsilon = 2 * m * n * p * res_ud_k_ + m * m * V * V + p * p * V * V - p * p * n * n - m * m * res_ud_k_
                           * res_ud_k_;
    if (epsilon >= 0) {
        ajac_.dpv_iq_lb_ = (-m * n - p * res_ud_k_ - std::sqrt(epsilon)) / (m * m + p * p);
        ajac_.dpv_iq_ub_ = (-m * n - p * res_ud_k_ + std::sqrt(epsilon)) / (m * m + p * p);
    } else {
        // std::cerr << "no solution! " << std::endl;
        ajac_.dpv_iq_lb_ = (-m * n - p * res_ud_k_) / (m * m + p * p);
        ajac_.dpv_iq_ub_ = (-m * n - p * res_ud_k_) / (m * m + p * p);
    }

    constexpr double asym_filter = 0.5;
    if (ajac_.dpv_iq_ub_ > ajac_.dpv_iq_ub_last_) {
        ajac_.dpv_iq_ub_ = ajac_.dpv_iq_ub_ * asym_filter + (1 - asym_filter) * ajac_.dpv_iq_ub_last_;
    }
    if (ajac_.dpv_iq_lb_ < ajac_.dpv_iq_lb_last_) {
        ajac_.dpv_iq_lb_ = ajac_.dpv_iq_lb_ * asym_filter + (1 - asym_filter) * ajac_.dpv_iq_lb_last_;
    }

    constexpr double dead_zone = 0.0;
    ajac_.dpv_iq_lb_ = ajac_.dpv_iq_lb_ > -dead_zone ? -dead_zone : ajac_.dpv_iq_lb_;
    ajac_.dpv_iq_ub_ = ajac_.dpv_iq_ub_ < dead_zone ? dead_zone : ajac_.dpv_iq_ub_;

    ajac_.dpv_tau_lb_ = ajac_.dpv_iq_lb_ * motor_.kt_;
    ajac_.dpv_tau_ub_ = ajac_.dpv_iq_ub_ * motor_.kt_;

    // note: the ddq_max for DPV is the declaration acc
    if (dq_k_ >= 0) {
        ddq_max_k_vats_ = (std::fabs(ajac_.dpv_tau_lb_ - tau_ff_k_)) / motor_.rotor_inertia_;
    } else {
        ddq_max_k_vats_ = (std::fabs(ajac_.dpv_tau_ub_ - tau_ff_k_)) / motor_.rotor_inertia_;
    }

}

/// run update_vats and DEl_Prete_VK first
/// @param tau_ref
/// @return
double AJAC::clip_tau_by_ajac(const double tau_ref) {
    double tau_ref_temp = tau_ref;
    const int frequency_flg = 1000 / (static_cast<int>(1 / vbac_.delta_t_));
    vats_counter_++;
    if (use_kinematic_only) {
        if (vats_counter_ == frequency_flg) {
            // convert tau_ref to acc_ref
            const double acc_ref_temp = (tau_ref_temp - tau_ff_k_) / motor_.rotor_inertia_;

            // this->Del_Prete_VK(inside_encoder_, dq_k_, ddq_max_k_vats_);
            this->Kinematic_Envelop(inside_encoder_, dq_k_, 0);
            // std::cout << "Low: " << dpv_.ddq_minmax_output_.lb << " Upper: " << dpv_.ddq_minmax_output_.ub << "\n";
            // std::cout << "ddq_max_k_vats_: " << ddq_max_k_vats_ << std::endl;
            const double acc_ref_clipped = std::max(std::min(acc_ref_temp, vbac_.ddq_minmax_output_.ub),
                                                    vbac_.ddq_minmax_output_.lb);
            // convert acc_ref back to tau_ref, this procedure will crease some error, so use vats or mor to ensure safety
            tau_ref_temp = acc_ref_clipped * motor_.rotor_inertia_ + tau_ff_k_;
            vats_counter_ = 0;
            dpv_last_tau_ = tau_ref_temp;
        } else {
            tau_ref_temp = dpv_last_tau_;
        }
    }

    if (use_ajac) {
        // convert tau_ref to acc_ref
        const double acc_ref_temp = (tau_ref_temp - tau_ff_k_) / motor_.rotor_inertia_;
        this->Kinematic_Envelop(inside_encoder_, dq_k_, ddq_max_k_vats_);
        const double acc_ref_clipped = std::max(std::min(acc_ref_temp, vbac_.ddq_minmax_output_.ub),
                                                vbac_.ddq_minmax_output_.lb);
        tau_ref_temp = acc_ref_clipped * motor_.rotor_inertia_ + tau_ff_k_;
        vats_counter_ = 0;
        dpv_last_tau_ = tau_ref_temp;

        tau_ref_temp = tau_ref_temp > fi_tau_ub_k_ ? fi_tau_ub_k_ : tau_ref_temp;
        tau_ref_temp = tau_ref_temp < fi_tau_lb_k_ ? fi_tau_lb_k_ : tau_ref_temp;
    }

    return tau_ref_temp;
}

/// MOR by shin
/// @param dq
/// @param tau_ref
/// @return
double AJAC::clip_tau_by_MOR(const double dq, const double tau_ref) {
    // The original equation should be tau = -4.5622 * dq + 62.562969;
    // To match the speed of VATS (under \alpha=1, and V=12.8568V ), we scale the equation
    // std::cout <<"ref: " << tau_ref << std::endl;
    double tau_ub_ = 0;
    double tau_lb_ = 0;
    double tau_out = tau_ref;
    if (use_ajac) {
        // convert tau_ref to acc_ref
        const double acc_ref_temp = (tau_ref - tau_ff_k_) / motor_.rotor_inertia_;
        // use default ddq_max
        this->Kinematic_Envelop(inside_encoder_, dq_k_, 0);
        const double acc_ref_clipped = std::max(std::min(acc_ref_temp, vbac_.ddq_minmax_output_.ub),
                                                vbac_.ddq_minmax_output_.lb);
        // convert acc_ref back to tau_ref, this procedure will crease some error, so use vats or mor to ensure safety
        tau_out = acc_ref_clipped * motor_.rotor_inertia_ + tau_ff_k_;
    }
    // 24v
    if (motor_.name == MOTOR_8115::name) {
        if (dq >= 0) {
            tau_ub_ = -4.5622 * dq + 56.31;
            tau_lb_ = -35.0;
            tau_out = tau_out > tau_ub_ ? tau_ub_ : tau_out;
            tau_out = tau_out < tau_lb_ ? tau_lb_ : tau_out;
        }
        if (dq < 0) {
            tau_lb_ = -4.5622 * dq - 56.31;
            tau_ub_ = 35.0;
            tau_out = tau_out > tau_ub_ ? tau_ub_ : tau_out;
            tau_out = tau_out < tau_lb_ ? tau_lb_ : tau_out;
        }
        // 36V this original bounds: 93.844535, for safety shrink this bound to 84.46
        // if (dq >= 0) {
        //     tau_ub_ = -4.5622 * dq + 84.46;
        //     tau_lb_ = -35.0;
        //     tau_out = tau_ref > tau_ub_ ? tau_ub_ : tau_ref;
        //     tau_out = tau_out < tau_lb_ ? tau_lb_ : tau_out;
        // }
        // if (dq < 0) {
        //     tau_lb_ = -4.5622 * dq - 84.46;
        //     tau_ub_ = 35.0;
        //     tau_out = tau_ref > tau_ub_ ? tau_ub_ : tau_ref;
        //     tau_out = tau_out < tau_lb_ ? tau_lb_ : tau_out;
        // }
    }

    if (motor_.name == MOTOR_6210::name) {
        // 48V: original line: tau = -2.1749 * dq + 74.92, match the alha 0.9
        // 24v: 74.92 * (24 / 48) * 0.9 = 33.714
        // std::cout << "sdsa" << std::endl;
        if (dq >= 0) {
            tau_ub_ = -2.1749 * dq + 33.714;
            tau_ub_ = tau_ub_ < 0 ? 0 : tau_ub_;
            tau_lb_ = -20.0;
            tau_out = tau_out > tau_ub_ ? tau_ub_ : tau_out;
            tau_out = tau_out < tau_lb_ ? tau_lb_ : tau_out;
        }
        if (dq < 0) {
            tau_lb_ = -2.1749 * dq - 33.714;
            tau_lb_ = tau_lb_ > 0 ? 0.0 : tau_lb_;
            tau_ub_ = 20.0;
            tau_out = tau_out > tau_ub_ ? tau_ub_ : tau_out;
            tau_out = tau_out < tau_lb_ ? tau_lb_ : tau_out;
        }
        // 36V
        // if (dq >= 0) {
        //     tau_ub_ = -2.1749 * dq + 50.571;
        //     tau_ub_ = tau_ub_ < 0 ? 0 : tau_ub_;
        //     tau_lb_ = -20.0;
        //     tau_out = tau_out > tau_ub_ ? tau_ub_ : tau_out;
        //     tau_out = tau_out < tau_lb_ ? tau_lb_ : tau_out;
        // }
        // if (dq < 0) {
        //     tau_lb_ = -2.1749 * dq - 50.571;
        //     tau_lb_ = tau_lb_ > 0 ? 0.0 : tau_lb_;
        //     tau_ub_ = 20.0;
        //     tau_out = tau_out > tau_ub_ ? tau_ub_ : tau_out;
        //     tau_out = tau_out < tau_lb_ ? tau_lb_ : tau_out;
        // }
    }
    // std::cout << "MOR tau bounds[LB/UB]: " << tau_lb_ << " " << tau_ub_ << std::endl;
    // std::cout << "Torque out: " << tau_out << std::endl;

    return tau_out;
}

void AJAC::Kinematic_Envelop(const double q, const double dq, double ddq_max) {
    if (ddq_max == 0) {
        vbac_.ddq_max_ = vbac_.ddq_max_default_;
    } else {
        vbac_.ddq_max_ = ddq_max;
    }
    // acc bounds from pos limits
    const double ddq_M1 = -dq / vbac_.delta_t_;
    const double ddq_M2 = -dq * dq / (2 * (vbac_.q_max_ - q));
    const double ddq_M3 = 2 * (vbac_.q_max_ - q - dq * vbac_.delta_t_) / (vbac_.delta_t_ * vbac_.delta_t_);
    const double ddq_m2 = dq * dq / (2 * (q - vbac_.q_min_));
    const double ddq_m3 = 2 * (vbac_.q_min_ - q - dq * vbac_.delta_t_) / (vbac_.delta_t_ * vbac_.delta_t_);
    double ddq_ub_pos = 0;
    double ddq_lb_pos = 0;
    if (dq >= 0) {
        ddq_lb_pos = ddq_m3;
        if (ddq_M3 > ddq_M1) {
            ddq_ub_pos = ddq_M3;
        } else {
            ddq_ub_pos = std::min(ddq_M1, ddq_M2);
        }
    } else {
        ddq_ub_pos = ddq_M3;
        if (ddq_m3 < ddq_M1) {
            ddq_lb_pos = ddq_m3;
        } else {
            ddq_lb_pos = std::max(ddq_M1, ddq_m2);
        }
    }

    // acc bounds from vel limits
    const double a = vbac_.delta_t_ * vbac_.delta_t_;
    double b = vbac_.delta_t_ * (2 * dq + vbac_.delta_t_ * vbac_.ddq_max_);
    double c = dq * dq - 2 * vbac_.ddq_max_ * (vbac_.q_max_ - q - dq * vbac_.delta_t_);
    const double ddq_1 = -dq / vbac_.delta_t_;
    double Delta = b * b - 4 * a * c;
    double ddq_ub_vel = 0;
    double ddq_lb_vel = 0;
    if (Delta >= 0) {
        ddq_ub_vel = std::max(ddq_1, (-b + std::sqrt(Delta)) / (2 * a));
    } else {
        ddq_ub_vel = ddq_1;
    }
    // negative speed
    b = 2 * vbac_.delta_t_ * dq - vbac_.ddq_max_ * vbac_.delta_t_ * vbac_.delta_t_;
    c = dq * dq - 2 * vbac_.ddq_max_ * (q + vbac_.delta_t_ * dq - vbac_.q_min_);
    Delta = b * b - 4 * a * c;
    if (Delta >= 0) {
        ddq_lb_vel = std::min(ddq_1, (-b - std::sqrt(Delta)) / (2 * a));
    } else {
        ddq_lb_vel = ddq_1;
    }

    vbac_.ub_[0] = vbac_.ddq_max_;
    vbac_.ub_[1] = ddq_ub_pos;
    vbac_.ub_[2] = ddq_ub_vel;
    vbac_.ub_[3] = (vbac_.dq_max_ - dq) / (vbac_.delta_t_);
    vbac_.lb_[0] = -vbac_.ddq_max_;
    vbac_.lb_[1] = ddq_lb_pos;
    vbac_.lb_[2] = ddq_lb_vel;
    vbac_.lb_[3] = (-vbac_.dq_max_ - dq) / (vbac_.delta_t_);

    vbac_.ddq_minmax_output_.lb = *std::max_element(vbac_.lb_.begin(), vbac_.lb_.end());
    vbac_.ddq_minmax_output_.ub = *std::min_element(vbac_.ub_.begin(), vbac_.ub_.end());
}

void AJAC::inside_encoder(const double q) {
    if (first_run_) {
        encoder_offset_ = q + 8 * M_PI;
        q_k_ = q_k_last_ = q + 8 * M_PI;
        first_run_ = false;
    }
    q_k_ = q + 8 * M_PI;
    // my encoder range: +- 4 pi,
    if ((q_k_ - q_k_last_) > (4 * M_PI))
        encoder_loop_ -= 1;
    else if (q_k_ - q_k_last_ < (-4 * M_PI))
        encoder_loop_ += 1;
    inside_encoder_ = q_k_ - encoder_offset_ + encoder_loop_ * (8 * M_PI);
    q_k_last_ = q_k_;
}
