#include "bot_dugma/m_step.hpp"
#include "bot_dugma/lrbfgs_wrapper.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <Eigen/Geometry>

namespace bot_dugma
{
    #include "generated/B.inl"
    #include "generated/grad_r_to_grad_q.inl"

    MStep::MStep()
    {

    }

    namespace
    {
        constexpr int kMaxNonFiniteDiagnostics {20};
        constexpr int kMaxBasisCoeffSummaries {20};
        int g_non_finite_diagnostics_printed {0};
        int g_basis_coeff_summaries_printed {0};

        template<std::size_t N>
        bool firstNonFinite(
            const std::array<float, N>& values,
            std::size_t& bad_index,
            float& bad_value
        )
        {
            for(std::size_t idx = 0; idx < N; ++idx)
            {
                if(!std::isfinite(values[idx]))
                {
                    bad_index = idx;
                    bad_value = values[idx];
                    return true;
                }
            }

            return false;
        }

        void printNonFiniteDiagnostic(
            const char* stage,
            const BasisCoeffsVec& A,
            const std::array<float, 4>& q,
            const std::array<float, 3>& t,
            const std::array<float, 9>& R_r,
            const std::array<float, 12>& u,
            float energy,
            const std::array<float, 12>& grad_u,
            const std::array<float, 4>& grad_q,
            const std::array<float, 3>& grad_t
        )
        {
            if(g_non_finite_diagnostics_printed >= kMaxNonFiniteDiagnostics)
            {
                return;
            }

            ++g_non_finite_diagnostics_printed;

            std::size_t idx {};
            float value {};

            std::printf(
                "[bot_dugma::m_step] non-finite diagnostic stage=%s energy=%.9e\n",
                stage,
                static_cast<double>(energy)
            );

            if(firstNonFinite(A.vals, idx, value))
            {
                std::printf(
                    "[bot_dugma::m_step] first non-finite A[%zu]=%.9e\n",
                    idx,
                    static_cast<double>(value)
                );
            }
            if(firstNonFinite(q, idx, value))
            {
                std::printf(
                    "[bot_dugma::m_step] first non-finite q[%zu]=%.9e\n",
                    idx,
                    static_cast<double>(value)
                );
            }
            if(firstNonFinite(t, idx, value))
            {
                std::printf(
                    "[bot_dugma::m_step] first non-finite t[%zu]=%.9e\n",
                    idx,
                    static_cast<double>(value)
                );
            }
            if(firstNonFinite(R_r, idx, value))
            {
                std::printf(
                    "[bot_dugma::m_step] first non-finite R_r[%zu]=%.9e\n",
                    idx,
                    static_cast<double>(value)
                );
            }
            if(firstNonFinite(u, idx, value))
            {
                std::printf(
                    "[bot_dugma::m_step] first non-finite u[%zu]=%.9e\n",
                    idx,
                    static_cast<double>(value)
                );
            }
            if(firstNonFinite(grad_u, idx, value))
            {
                std::printf(
                    "[bot_dugma::m_step] first non-finite grad_u[%zu]=%.9e\n",
                    idx,
                    static_cast<double>(value)
                );
            }
            if(firstNonFinite(grad_q, idx, value))
            {
                std::printf(
                    "[bot_dugma::m_step] first non-finite grad_q[%zu]=%.9e\n",
                    idx,
                    static_cast<double>(value)
                );
            }
            if(firstNonFinite(grad_t, idx, value))
            {
                std::printf(
                    "[bot_dugma::m_step] first non-finite grad_t[%zu]=%.9e\n",
                    idx,
                    static_cast<double>(value)
                );
            }
        }

        void printBasisCoeffSummary(
            const BasisCoeffsVec& A
        )
        {
            if(g_basis_coeff_summaries_printed >= kMaxBasisCoeffSummaries)
            {
                return;
            }

            ++g_basis_coeff_summaries_printed;

            std::size_t finite_count {0};
            std::size_t nonfinite_count {0};
            std::size_t zero_count {0};
            std::size_t first_nonfinite_idx {0};
            float first_nonfinite_value {0.0f};
            bool has_nonfinite {false};

            float min_value {std::numeric_limits<float>::infinity()};
            float max_value {-std::numeric_limits<float>::infinity()};
            float max_abs_value {0.0f};
            std::size_t max_abs_idx {0};
            double sum_abs {0.0};

            for(std::size_t idx = 0; idx < A.vals.size(); ++idx)
            {
                const float value = A.vals[idx];
                if(!std::isfinite(value))
                {
                    if(!has_nonfinite)
                    {
                        first_nonfinite_idx = idx;
                        first_nonfinite_value = value;
                        has_nonfinite = true;
                    }
                    ++nonfinite_count;
                    continue;
                }

                ++finite_count;
                if(value == 0.0f)
                {
                    ++zero_count;
                }

                if(value < min_value)
                {
                    min_value = value;
                }
                if(value > max_value)
                {
                    max_value = value;
                }

                const float abs_value = std::fabs(value);
                sum_abs += static_cast<double>(abs_value);
                if(abs_value > max_abs_value)
                {
                    max_abs_value = abs_value;
                    max_abs_idx = idx;
                }
            }

            const double mean_abs =
                finite_count > 0 ?
                sum_abs / static_cast<double>(finite_count) :
                0.0;

            std::printf(
                "[bot_dugma::m_step] A coeff summary finite=%zu nonfinite=%zu zeros=%zu "
                "min=%.9e max=%.9e max_abs=%.9e max_abs_idx=%zu mean_abs=%.9e\n",
                finite_count,
                nonfinite_count,
                zero_count,
                static_cast<double>(finite_count > 0 ? min_value : 0.0f),
                static_cast<double>(finite_count > 0 ? max_value : 0.0f),
                static_cast<double>(max_abs_value),
                max_abs_idx,
                mean_abs
            );

            if(has_nonfinite)
            {
                std::printf(
                    "[bot_dugma::m_step] first non-finite A coeff before optimize A[%zu]=%.9e\n",
                    first_nonfinite_idx,
                    static_cast<double>(first_nonfinite_value)
                );
            }
        }

    }

    std::array<float, 12> poseToU(
        const PoseEstimate& pose
    )
    {
        return {
            pose.m_t[0], pose.m_t[1], pose.m_t[2],
            pose.m_R[0], pose.m_R[1], pose.m_R[2],
            pose.m_R[3], pose.m_R[4], pose.m_R[5],
            pose.m_R[6], pose.m_R[7], pose.m_R[8]
        };
    }

    void quatToRFlat(
        const std::array<float, 4>& q,
        std::array<float, 9>& R_r
    )
    {
        const float qx = q[0];
        const float qy = q[1];
        const float qz = q[2];
        const float qw = q[3];

        Eigen::Quaternionf quat(qw, qx, qy, qz);
        quat.normalize();

        Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> R(
            R_r.data()
        );

        R = quat.toRotationMatrix();
    }

    void poseToQuatT(
        const PoseEstimate& pose,
        std::array<float, 4>& q,
        std::array<float, 3>& t
    )
    {
        t[0] = pose.m_t[0];
        t[1] = pose.m_t[1];
        t[2] = pose.m_t[2];

        Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> R(
            pose.m_R
        );

        Eigen::Quaternionf quat(R);
        quat.normalize();

        q[0] = quat.x();
        q[1] = quat.y();
        q[2] = quat.z();
        q[3] = quat.w();
    }

    float evaluateEnergyAndQuatTGradient(
        const BasisCoeffsVec& A,
        const std::array<float, 4>& q,
        const std::array<float, 3>& t,
        std::array<float, 4>& grad_q,
        std::array<float, 3>& grad_t
    )
    {
        std::array<float, 9> R_r {};
        quatToRFlat(
            q,
            R_r
        );

        const std::array<float, 12> u {
            t[0], t[1], t[2],
            R_r[0], R_r[1], R_r[2],
            R_r[3], R_r[4], R_r[5],
            R_r[6], R_r[7], R_r[8]
        };

        float energy = 0.0f;
        std::array<float, 12> grad_u {};

        computeEnergyAndEuclideanGradient(
            u,
            A,
            energy,
            grad_u
        );

        if(!std::isfinite(energy))
        {
            printNonFiniteDiagnostic(
                "after_compute_energy",
                A,
                q,
                t,
                R_r,
                u,
                energy,
                grad_u,
                grad_q,
                grad_t
            );
        }

        grad_t[0] = grad_u[0];
        grad_t[1] = grad_u[1];
        grad_t[2] = grad_u[2];

        const std::array<float, 9> grad_R_r {
            grad_u[3], grad_u[4], grad_u[5],
            grad_u[6], grad_u[7], grad_u[8],
            grad_u[9], grad_u[10], grad_u[11]
        };

        GradRFlatToGradQ(
            q[0],
            q[1],
            q[2],
            q[3],
            grad_R_r.data(),
            grad_q.data()
        );

        std::size_t idx {};
        float value {};
        if(firstNonFinite(grad_u, idx, value) ||
           firstNonFinite(grad_q, idx, value) ||
           firstNonFinite(grad_t, idx, value))
        {
            printNonFiniteDiagnostic(
                "after_gradient_conversion",
                A,
                q,
                t,
                R_r,
                u,
                energy,
                grad_u,
                grad_q,
                grad_t
            );
        }

        return energy;
    }

    

    PoseEstimate MStep::optimize(
        const BasisCoeffsVec& A,
        const PoseEstimate& initial_pose
    )
    {
        printBasisCoeffSummary(A);

        LRBFGSWrapper optimizer(A);
        return optimizer.optimize(initial_pose);
    }
}
