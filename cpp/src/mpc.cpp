#include "cartpole_mpc/mpc.hpp"

#include "osqp.h"

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <algorithm>
#include <stdexcept>
#include <vector>



namespace cartpole {
namespace {

constexpr int kNx = Mpc::kNumStates;
// OSQP's own sentinel for an infinite bound, NOT IEEE infinity. OSQP runs Ruiz
// equilibration over l and u during setup; a real inf poisons those norms and
// the solver then fails to converge at ANY tolerance -- which is exactly how
// this presented: max_iter_reached at eps 1e-3 through 1e-6 alike.
constexpr double kInf = OSQP_INFTY;

using RowMat4 = Eigen::Matrix<double, kNx, kNx, Eigen::RowMajor>;
using Triplet = Eigen::Triplet<double>;

// Copy an Eigen compressed sparse column matrix into OSQP's index type.
// Eigen stores int indices by default; OSQPInt here is long long, so the index
// arrays cannot simply be aliased.
void ToOsqpCsc(const Eigen::SparseMatrix<double>& src,
               std::vector<OSQPFloat>* values, std::vector<OSQPInt>* rows,
               std::vector<OSQPInt>* cols) {
  values->assign(src.valuePtr(), src.valuePtr() + src.nonZeros());
  rows->resize(src.nonZeros());
  cols->resize(src.cols() + 1);
  for (Eigen::Index k = 0; k < src.nonZeros(); ++k) {
    (*rows)[k] = static_cast<OSQPInt>(src.innerIndexPtr()[k]);
  }
  for (Eigen::Index k = 0; k <= src.cols(); ++k) {
    (*cols)[k] = static_cast<OSQPInt>(src.outerIndexPtr()[k]);
  }
}

}  // namespace

struct Mpc::Impl {
  MpcOptions options;
  int horizon = 0;
  bool use_slack = false;
  int num_vars = 0;
  int num_cons = 0;

  // Storage that OSQP's matrix views point into; must outlive the solver.
  std::vector<OSQPFloat> P_values, A_values, q, lower, upper;
  std::vector<OSQPInt> P_rows, P_cols, A_rows, A_cols;
  OSQPCscMatrix P{}, A{};

  OSQPSolver* solver = nullptr;
  OSQPSettings* settings = nullptr;
  int last_iterations = -1;
  int last_status = 0;

  // Variable layout: [x_0..x_N] [u_0..u_{N-1}] [s_0..s_{N-1}]
  int xi(int k, int j) const { return kNx * k + j; }
  int ui(int k) const { return kNx * (horizon + 1) + k; }
  int si(int k) const { return kNx * (horizon + 1) + horizon + k; }

  ~Impl() {
    if (solver) osqp_cleanup(solver);
    if (settings) free(settings);
  }
};

Mpc::Mpc(const double* Ad, const double* Bd, const double* Q, const double* R,
         const double* S, const MpcOptions& options)
    : impl_(std::make_unique<Impl>()) {
  if (!Ad || !Bd || !Q || !R || !S) throw std::invalid_argument("null matrix");
  if (options.horizon < 1) throw std::invalid_argument("horizon must be >= 1");
  if (options.dt <= 0.0) throw std::invalid_argument("dt must be > 0");

  Impl& impl = *impl_;
  impl.options = options;
  const int N = impl.horizon = options.horizon;
  const double dt = options.dt;
  impl.use_slack = options.x_max >= 0.0;

  Eigen::Map<const RowMat4> Ad_m(Ad), Q_m(Q), S_m(S);
  Eigen::Map<const Eigen::Matrix<double, kNx, 1>> Bd_m(Bd);
  const double R_scalar = *R;

  const int num_vars = kNx * (N + 1) + N + (impl.use_slack ? N : 0);
  //  4 rows pinning x_0, 4N dynamics, N input bounds, and when slack is in play
  //  N non-negativity rows plus 2N soft track rows.
  const int num_cons = 4 + kNx * N + N + (impl.use_slack ? 3 * N : 0);
  impl.num_vars = num_vars;
  impl.num_cons = num_cons;

  // ---- cost:  0.5 z' P z + q' z ----------------------------------------
  // OSQP wants the UPPER TRIANGLE of P only. The objective is
  //   sum_k dt*(x_k' Q x_k + u_k' R u_k) + x_N' S x_N + penalty * sum_k s_k
  // and since 0.5*P must equal those blocks, every block is doubled.
  std::vector<Triplet> p_triplets;
  for (int k = 0; k <= N; ++k) {
    const RowMat4& block = (k == N) ? S_m : Q_m;
    const double scale = (k == N) ? 2.0 : 2.0 * dt;
    for (int r = 0; r < kNx; ++r) {
      for (int c = r; c < kNx; ++c) {  // upper triangle
        const double v = scale * block(r, c);
        if (v != 0.0) p_triplets.emplace_back(impl.xi(k, r), impl.xi(k, c), v);
      }
    }
  }
  for (int k = 0; k < N; ++k) {
    p_triplets.emplace_back(impl.ui(k), impl.ui(k), 2.0 * dt * R_scalar);
  }
  Eigen::SparseMatrix<double> P_sparse(num_vars, num_vars);
  P_sparse.setFromTriplets(p_triplets.begin(), p_triplets.end());
  P_sparse.makeCompressed();
  ToOsqpCsc(P_sparse, &impl.P_values, &impl.P_rows, &impl.P_cols);

  impl.q.assign(num_vars, 0.0);
  if (impl.use_slack) {
    for (int k = 0; k < N; ++k) impl.q[impl.si(k)] = options.slack_penalty;
  }

  // ---- constraints:  l <= A z <= u -------------------------------------
  std::vector<Triplet> a_triplets;
  impl.lower.assign(num_cons, 0.0);
  impl.upper.assign(num_cons, 0.0);
  int row = 0;

  // x_0 = e0. These four rows are the ONLY thing that changes per solve.
  for (int j = 0; j < kNx; ++j, ++row) {
    a_triplets.emplace_back(row, impl.xi(0, j), 1.0);
    impl.lower[row] = impl.upper[row] = 0.0;  // overwritten by Solve()
  }

  // x_{k+1} = Ad x_k + Bd u_k  ->  Ad x_k + Bd u_k - x_{k+1} = 0
  for (int k = 0; k < N; ++k) {
    for (int r = 0; r < kNx; ++r, ++row) {
      for (int c = 0; c < kNx; ++c) {
        if (Ad_m(r, c) != 0.0) {
          a_triplets.emplace_back(row, impl.xi(k, c), Ad_m(r, c));
        }
      }
      if (Bd_m(r) != 0.0) a_triplets.emplace_back(row, impl.ui(k), Bd_m(r));
      a_triplets.emplace_back(row, impl.xi(k + 1, r), -1.0);
      impl.lower[row] = impl.upper[row] = 0.0;
    }
  }

  // |u_k| <= u_max. Hard: this is our own actuator, a promise we can keep.
  for (int k = 0; k < N; ++k, ++row) {
    a_triplets.emplace_back(row, impl.ui(k), 1.0);
    impl.lower[row] = -options.u_max;
    impl.upper[row] = options.u_max;
  }

  if (impl.use_slack) {
    // s_k >= 0
    for (int k = 0; k < N; ++k, ++row) {
      a_triplets.emplace_back(row, impl.si(k), 1.0);
      impl.lower[row] = 0.0;
      impl.upper[row] = kInf;
    }
    // Track limit, SOFT:  x_cart - s <= x_max  and  -x_cart - s <= x_max.
    // A disturbance can put the cart outside the track, and a hard constraint
    // would make the problem infeasible exactly when a control action matters
    // most. Violation is penalised instead of forbidden.
    for (int k = 0; k < N; ++k) {
      a_triplets.emplace_back(row, impl.xi(k + 1, 0), 1.0);
      a_triplets.emplace_back(row, impl.si(k), -1.0);
      impl.lower[row] = -kInf;
      impl.upper[row] = options.x_max;
      ++row;
      a_triplets.emplace_back(row, impl.xi(k + 1, 0), -1.0);
      a_triplets.emplace_back(row, impl.si(k), -1.0);
      impl.lower[row] = -kInf;
      impl.upper[row] = options.x_max;
      ++row;
    }
  }

  Eigen::SparseMatrix<double> A_sparse(num_cons, num_vars);
  A_sparse.setFromTriplets(a_triplets.begin(), a_triplets.end());
  A_sparse.makeCompressed();
  ToOsqpCsc(A_sparse, &impl.A_values, &impl.A_rows, &impl.A_cols);

  OSQPCscMatrix_set_data(&impl.P, num_vars, num_vars,
                         static_cast<OSQPInt>(impl.P_values.size()),
                         impl.P_values.data(), impl.P_rows.data(),
                         impl.P_cols.data());
  OSQPCscMatrix_set_data(&impl.A, num_cons, num_vars,
                         static_cast<OSQPInt>(impl.A_values.size()),
                         impl.A_values.data(), impl.A_rows.data(),
                         impl.A_cols.data());

  impl.settings = static_cast<OSQPSettings*>(malloc(sizeof(OSQPSettings)));
  if (!impl.settings) throw std::bad_alloc();
  osqp_set_default_settings(impl.settings);
  impl.settings->verbose = 0;
  impl.settings->warm_starting = 1;   // consecutive QPs differ in 4 bounds
  impl.settings->polishing = options.polishing ? 1 : 0;
  impl.settings->eps_abs = options.eps_abs;
  impl.settings->eps_rel = options.eps_rel;
  impl.settings->max_iter = options.max_iter;

  // Setup happens ONCE. The sparsity pattern and the factorisation are reused
  // for the life of the controller; per step we only move four bounds.
  const OSQPInt status =
      osqp_setup(&impl.solver, &impl.P, impl.q.data(), &impl.A,
                 impl.lower.data(), impl.upper.data(), num_cons, num_vars,
                 impl.settings);
  if (status != 0 || !impl.solver) {
    throw std::runtime_error("osqp_setup failed");
  }
}

Mpc::~Mpc() = default;

bool Mpc::Solve(const double* e0, double* u_out) {
  if (!e0 || !u_out) return false;
  Impl& impl = *impl_;

  for (int j = 0; j < kNx; ++j) {
    impl.lower[j] = impl.upper[j] = e0[j];
  }
  if (osqp_update_data_vec(impl.solver, nullptr, impl.lower.data(),
                           impl.upper.data()) != 0) {
    return false;
  }
  if (osqp_solve(impl.solver) != 0) return false;

  impl.last_iterations = static_cast<int>(impl.solver->info->iter);
  impl.last_status = static_cast<int>(impl.solver->info->status_val);
  if (impl.last_status != OSQP_SOLVED) return false;

  *u_out = impl.solver->solution->x[impl.ui(0)];
  return true;
}

int Mpc::last_iterations() const { return impl_->last_iterations; }

int Mpc::last_status() const { return impl_->last_status; }

}  // namespace cartpole
