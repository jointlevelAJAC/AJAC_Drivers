#ifndef MOTOR_SAFE_CTRL_H
#define MOTOR_SAFE_CTRL_H
#include <string>
#include <vector>

// Constants for motor 8115
typedef struct motor_struct {
    std::string name;
    double ls_;
    double rs_;
    double flux_linkage_;
    double gr_;
    double v_bound_;
    double rotor_inertia_;
    double npp;
    double kt_; //Note: this kt_ directly corresponds to iq, depending on the omp. gain and gr.
    double step_;
    double control_dt_;
} motor_typ;

typedef struct bounds {
    double lb;
    double ub;
} bound_typ;

typedef struct del_prete_vk {
    double delta_t_;
    double q_max_;
    double q_min_;
    double dq_max_;
    double ddq_max_;
    double ddq_max_default_;
    std::vector<double> ub_;
    std::vector<double> lb_;
    bound_typ ddq_minmax_output_;
} vbac_typ;

typedef struct vats_pre_acc {
    double dpv_tau_ub_;
    double dpv_tau_lb_;
    double dpv_iq_ub_;
    double dpv_iq_ub_last_;
    double dpv_iq_lb_;
    double dpv_iq_lb_last_;
} ajac_typ;

class AJAC {
public:
    explicit AJAC(motor_typ motor);

    ~AJAC() = default;

    void update_data(double dq, double uq, double ud, double iq, double tau_ff);

    void Ik_transient();

    void Ik_steady_predict();

    void update_kinematic_limit();

    double clip_tau_by_ajac(double tau_ref);

    double clip_tau_by_MOR(double dq, double tau_ref);

    void Kinematic_Envelop(double q, double dq, double ddq);

    void set_ajac() { use_kinematic_only = false; use_ajac = true;};
    void set_kinematic_only() { use_kinematic_only = true; use_ajac = false; };
    void stop_ajac() { use_kinematic_only = false; use_ajac = false; };

    void inside_encoder(double q);

    void set_ajac_steps(const double step) { steps_ = step; };
    void set_ajac_control_dt(const double dt) { control_dt_ = dt; };

    void set_kinematic_constraints(const double q_max, const double q_min, const double dq_max, const double ddq_max_default,
                 const double delta_t) {
        vbac_.q_max_ = q_max;
        vbac_.q_min_ = q_min;
        vbac_.dq_max_ = dq_max;
        vbac_.ddq_max_default_ = ddq_max_default;
        vbac_.delta_t_ = delta_t;
    };
    double tr_iq_lb_k_{}, tr_iq_ub_k_{}, tr_iq_lb_k_1_{}, tr_iq_ub_k_1_{};
    double st_iq_lb_k_{}, st_iq_ub_k_{}, st_iq_lb_k_1_{}, st_iq_ub_k_1_{};
    double uq_k1{}, ud_k1{};
    double st_tau_lb_k_{}, st_tau_ub_k_{}, res_uq_k_{}, res_ud_k_{};
    double tr_tau_lb_k_{}, tr_tau_ub_k_{}, fi_tau_lb_k_{}, fi_tau_ub_k_{};
    double ddq_max_k_vats_{};
    double inside_encoder_{};
    vbac_typ vbac_{};

private:
    motor_typ motor_{};
    double dq_k_{}, omega_k_1_{};
    double q_k_{}, q_k_last_{};
    double tau_ff_k_{}, iq_k_{};
    double uq_k_{}, ud_k_{};
    double res_uq_k_1_{}, res_ud_k_1_{};
    double steps_{0.};
    double control_dt_{0.001};
    bool use_kinematic_only{true};
    bool use_ajac{false};

    double encoder_loop_{};
    double encoder_offset_{};
    bool first_run_{true};
    ajac_typ ajac_{};
    int vats_counter_{0};
    double dpv_last_tau_{};
    // double iq_lb_k_{}, iq_ub_k_{};
};

#endif
