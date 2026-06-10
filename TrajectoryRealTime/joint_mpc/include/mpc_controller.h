#pragma once
// Eigen-only joint-space MPC — no external QP solver.
//
// It is an *optimal reference generator*: per joint it models a double integrator
// on its OWN reference state [q_ref, v_ref] with acceleration input, and minimizes
//   J = sum_k w_q (q_k-goal)^2 + w_qf (q_N-goal)^2 + w_v v_k^2 + w_vf v_N^2 + w_u u_k^2
// over the input sequence. With no inequality constraints the condensed problem is
//   (Tq^T Wq Tq + Tv^T Wv Tv + w_u I) U = goal*(Aq*1) - (Aq Sq + Av Sv) x0,
// an SPD linear system solved once-factorized (LLT) and reused for all joints each
// tick (the joints are decoupled and share the same model + weights). The first
// optimal input is applied; the reference is integrated one control step and clamped
// to velocity/position limits (preventing wind-up). The clamped reference is the
// emitted setpoint. Hard in-QP limits are the OSQP upgrade noted in DESIGN.md.
//
// LIMITATION (feedforward only): after reset() seeds the reference from the measured
// state, compute() integrates ONLY its internal [q_ref, v_ref] and ignores the
// measured state thereafter. So the convergence guarantee is on the internal
// REFERENCE, not the measured arm: there is NO disturbance rejection and no
// steady-state-error correction. Against a clean plant the arm tracks the reference
// to the goal; against a persistent disturbance the measured arm settles offset.
// Closed-loop feedback MPC (re-anchor to measured q,dq each tick, or add integral
// action) is the M3+ upgrade. See DESIGN_M2.md.
//
// Same JointController seam as the trivial controllers; the SimBackend, queue, and
// driver are unchanged.
#include <algorithm>
#include "joint_controller.h"
#include "joint_types.h"

namespace jmpc {

struct MpcParams {
    int    N   = 100;      // horizon steps (0.2 s lookahead at dt=0.002)
    double dt  = 0.002;    // model/control timestep (s)
    double w_q  = 1.0;     // running position-tracking weight
    double w_qf = 50.0;    // terminal position weight
    double w_v  = 0.1;     // running velocity weight (damping — ~critically damped)
    double w_vf = 10.0;    // terminal velocity weight (arrive at rest)
    double w_u  = 0.01;    // input (acceleration) effort weight
};

class MpcJointController : public JointController {
public:
    MpcJointController(MpcParams p, JointLimits lim) : p_(p), lim_(std::move(lim)) {
        build();
    }

    int horizon_steps() const override { return p_.N; }

    // Bumpless transfer in BOTH position and velocity: seed the internal reference
    // from the measured state so a mid-motion switch-on continues at the current
    // velocity instead of snapping to rest (which would be a velocity discontinuity
    // on hardware).
    void reset(const JointState& s) override {
        q_ref_ = s.q;
        v_ref_ = s.dq;
        initialized_ = true;
    }

    JointSetpoint compute(const JointState& s, const Eigen::VectorXd& goal) override {
        if (!initialized_) reset(s);
        const int n = static_cast<int>(goal.size());
        const double dt = p_.dt;
        Eigen::VectorXd q_des(n), dq_des(n);

        for (int i = 0; i < n; ++i) {
            // Condensed solve for this joint: U = H^{-1} (g*M1ones - MSx*x0).
            Eigen::Vector2d x0(q_ref_(i), v_ref_(i));
            Eigen::VectorXd c = goal(i) * M1ones_ - MSx_ * x0;
            Eigen::VectorXd U = llt_.solve(c);
            const double u0 = U(0);

            // Apply the first input via semi-implicit (symplectic) Euler: clamp the
            // velocity first, then integrate position with the CLAMPED velocity, so the
            // per-tick position step is itself velocity-limited (|dq| <= dq_max*dt) and
            // the reference cannot wind up ahead of the limits.
            double v1 = v_ref_(i) + dt * u0;
            v1 = std::min(lim_.dq_max(i), std::max(-lim_.dq_max(i), v1));
            double q1 = q_ref_(i) + dt * v1;
            q1 = std::min(lim_.q_upper(i), std::max(lim_.q_lower(i), q1));

            q_ref_(i) = q1;
            v_ref_(i) = v1;
            q_des(i)  = q1;
            dq_des(i) = v1;
        }
        return JointSetpoint{ q_des, dq_des };
    }

private:
    // Precompute the condensed prediction/cost matrices (independent of joint, x0, goal).
    // Predicted q_k and v_k (k=1..N, row r=k-1) for the double integrator:
    //   q_k = Sq[r]*x0 + Tq[r,:]*U,   v_k = Sv[r]*x0 + Tv[r,:]*U.
    void build() {
        const int N = p_.N;
        const double dt = p_.dt;
        Eigen::MatrixXd Tq = Eigen::MatrixXd::Zero(N, N);
        Eigen::MatrixXd Tv = Eigen::MatrixXd::Zero(N, N);
        Eigen::MatrixXd Sq(N, 2), Sv(N, 2);
        for (int r = 0; r < N; ++r) {
            Sq(r, 0) = 1.0; Sq(r, 1) = (r + 1) * dt;   // q_k = q0 + k*dt*v0 + ...
            Sv(r, 0) = 0.0; Sv(r, 1) = 1.0;            // v_k = v0 + ...
            for (int j = 0; j <= r; ++j) {
                const int m = r - j;                   // = k-1-j
                Tq(r, j) = dt * dt * (0.5 + m);
                Tv(r, j) = dt;
            }
        }
        Eigen::VectorXd wq = Eigen::VectorXd::Constant(N, p_.w_q);  wq(N - 1) = p_.w_qf;
        Eigen::VectorXd wv = Eigen::VectorXd::Constant(N, p_.w_v);  wv(N - 1) = p_.w_vf;

        // Aq = Tq^T Wq, Av = Tv^T Wv. Cost minimized at:
        //   (Aq Tq + Av Tv + w_u I) U = g*(Aq 1) - (Aq Sq + Av Sv) x0   (velocity target 0).
        Eigen::MatrixXd Aq = Tq.transpose() * wq.asDiagonal();
        Eigen::MatrixXd Av = Tv.transpose() * wv.asDiagonal();
        Eigen::MatrixXd H  = Aq * Tq + Av * Tv + Eigen::MatrixXd::Identity(N, N) * p_.w_u;
        llt_.compute(H);
        M1ones_ = Aq * Eigen::VectorXd::Ones(N);       // N
        MSx_    = Aq * Sq + Av * Sv;                   // N x 2
    }

    MpcParams p_;
    JointLimits lim_;
    bool initialized_ = false;
    Eigen::VectorXd q_ref_, v_ref_;

    Eigen::LLT<Eigen::MatrixXd> llt_;
    Eigen::VectorXd M1ones_;
    Eigen::MatrixXd MSx_;
};

}  // namespace jmpc
