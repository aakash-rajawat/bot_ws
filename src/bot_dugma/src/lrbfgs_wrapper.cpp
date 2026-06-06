#include "bot_dugma/lrbfgs_wrapper.hpp"
#include "bot_dugma/m_step.hpp"

#include "Manifolds/Euclidean.h"
#include "Manifolds/MultiManifolds.h"
#include "Manifolds/Sphere.h"
#include "Solvers/LRBFGS.h"

#include <cmath>
#include <cstdio>

#include <array>
#include <cstddef>
#include <limits>

namespace
{
    constexpr int kQuatSize {4};
    constexpr int kTranslationSize {3};
    constexpr int kMaxIterations {50};
    constexpr int kHistorySize {10};
    constexpr double kGradTolerance {1.0e-6};
    constexpr double kSolverTimeBoundSeconds {2.0};

    template<std::size_t N>
    bool allFinite(
        const std::array<float, N>& values
    )
    {
        for(float value : values)
        {
            if(!std::isfinite(value))
            {
                return false;
            }
        }

        return true;
    }

    void variableToQuatT(
        const ROPTLIB::Variable& x,
        std::array<float, kQuatSize>& q,
        std::array<float, kTranslationSize>& t
    )
    {
        const realdp* q_data = x.GetElement(0).ObtainReadData();
        const realdp* t_data = x.GetElement(1).ObtainReadData();

        for(int idx = 0; idx < kQuatSize; ++idx)
        {
            q[idx] = static_cast<float>(q_data[idx]);
        }

        for(int idx = 0; idx < kTranslationSize; ++idx)
        {
            t[idx] = static_cast<float>(t_data[idx]);
        }
    }

    void writeQuatTToVariable(
        const std::array<float, kQuatSize>& q,
        const std::array<float, kTranslationSize>& t,
        ROPTLIB::Variable& x
    )
    {
        x.NewMemoryOnWrite();

        realdp* q_data = x.GetElement(0).ObtainWriteEntireData();
        realdp* t_data = x.GetElement(1).ObtainWriteEntireData();

        for(int idx = 0; idx < kQuatSize; ++idx)
        {
            q_data[idx] = static_cast<realdp>(q[idx]);
        }

        for(int idx = 0; idx < kTranslationSize; ++idx)
        {
            t_data[idx] = static_cast<realdp>(t[idx]);
        }
    }

    void writeGradientToVector(
        const std::array<float, kQuatSize>& grad_q,
        const std::array<float, kTranslationSize>& grad_t,
        ROPTLIB::Vector* result
    )
    {
        result->NewMemoryOnWrite();

        realdp* grad_q_data = result->GetElement(0).ObtainWriteEntireData();
        realdp* grad_t_data = result->GetElement(1).ObtainWriteEntireData();

        for(int idx = 0; idx < kQuatSize; ++idx)
        {
            grad_q_data[idx] = static_cast<realdp>(grad_q[idx]);
        }

        for(int idx = 0; idx < kTranslationSize; ++idx)
        {
            grad_t_data[idx] = static_cast<realdp>(grad_t[idx]);
        }
    }
}

namespace bot_dugma
{
    LRBFGSWrapper::LRBFGSWrapper(
        const BasisCoeffsVec& A
    )
        : m_A(A)
    {
        SetNumGradHess(false);
        SetUseGrad(true);
        SetUseHess(false);
    }

    double LRBFGSWrapper::f(
        const ROPTLIB::Variable& x
    ) const
    {
        std::array<float, kQuatSize> q {};
        std::array<float, kTranslationSize> t {};
        variableToQuatT(
            x,
            q,
            t
        );

        std::array<float, kQuatSize> grad_q {};
        std::array<float, kTranslationSize> grad_t {};

        const float energy = evaluateEnergyAndQuatTGradient(
            m_A,
            q,
            t,
            grad_q,
            grad_t
        );

        if(
            !std::isfinite(energy) ||
            !allFinite(grad_q) ||
            !allFinite(grad_t)
        )
        {
            m_saw_nonfinite_eval = true;
        }

        return static_cast<double>(energy);
    }

    ROPTLIB::Vector& LRBFGSWrapper::EucGrad(
        const ROPTLIB::Variable& x,
        ROPTLIB::Vector* result
    ) const
    {
        std::array<float, kQuatSize> q {};
        std::array<float, kTranslationSize> t {};
        variableToQuatT(
            x,
            q,
            t
        );

        std::array<float, kQuatSize> grad_q {};
        std::array<float, kTranslationSize> grad_t {};

        const float energy = evaluateEnergyAndQuatTGradient(
            m_A,
            q,
            t,
            grad_q,
            grad_t
        );

        if(
            !std::isfinite(energy) ||
            !allFinite(grad_q) ||
            !allFinite(grad_t)
        )
        {
            m_saw_nonfinite_eval = true;
        }

        writeGradientToVector(
            grad_q,
            grad_t,
            result
        );

        return *result;
    }

    static bool isFinitePose(const bot_dugma::PoseEstimate& p)
    {
        for (float v : p.m_R) {
            if (!std::isfinite(v)) {
                return false;
            }
        }

        for (float v : p.m_t) {
            if (!std::isfinite(v)) {
                return false;
            }
        }

        return std::isfinite(p.m_energy);
    }

    PoseEstimate LRBFGSWrapper::optimize(
        const PoseEstimate& initial_pose
    )
    {
        std::printf("[bot_dugma::LRBFGSWrapper] optimize: begin\n");
        m_saw_nonfinite_eval = false;

        std::array<float, kQuatSize> q0 {};
        std::array<float, kTranslationSize> t0 {};
        poseToQuatT(
            initial_pose,
            q0,
            t0
        );

        ROPTLIB::Sphere quat_domain(kQuatSize);
        quat_domain.ChooseParamsSet2();

        ROPTLIB::Euclidean translation_domain(kTranslationSize);

        ROPTLIB::ProductManifold domain(
            2,
            &quat_domain,
            1,
            &translation_domain,
            1
        );

        SetDomain(&domain);

        ROPTLIB::Variable x0 = domain.RandominManifold();
        writeQuatTToVariable(
            q0,
            t0,
            x0
        );

        ROPTLIB::LRBFGS solver(
            this,
            &x0
        );

        solver.Max_Iteration = kMaxIterations;
        solver.Tolerance = static_cast<realdp>(kGradTolerance);
        solver.LengthSY = kHistorySize;
        solver.LineSearch_LS = ROPTLIB::LSSM_ARMIJO;
        solver.TimeBound = static_cast<realdp>(kSolverTimeBoundSeconds);
        solver.Verbose = ROPTLIB::NOOUTPUT;

        solver.CheckParams();
        std::printf("[bot_dugma::LRBFGSWrapper] optimize: before solver.Run()\n");
        solver.Run();
        std::printf(
            "[bot_dugma::LRBFGSWrapper] optimize: after solver.Run() finalfun=%.9e iter=%d\n",
            static_cast<double>(solver.Getfinalfun()),
            static_cast<int>(solver.GetIter())
        );

        if(m_saw_nonfinite_eval || !std::isfinite(solver.Getfinalfun()))
        {
            std::printf(
                "[bot_dugma::LRBFGSWrapper] optimize: non-finite solver evaluation, returning non-converged pose\n"
            );

            PoseEstimate pose = initial_pose;
            pose.m_energy = m_saw_nonfinite_eval
                ? std::numeric_limits<float>::quiet_NaN()
                : static_cast<float>(solver.Getfinalfun());
            pose.m_iter = static_cast<int>(solver.GetIter());
            pose.m_is_converged = false;
            return pose;
        }

        std::printf("[bot_dugma::LRBFGSWrapper] optimize: before solver.GetXopt()\n");
        const ROPTLIB::Variable x_opt = solver.GetXopt();
        std::printf("[bot_dugma::LRBFGSWrapper] optimize: after solver.GetXopt()\n");

        std::array<float, kQuatSize> q_opt {};
        std::array<float, kTranslationSize> t_opt {};
        variableToQuatT(
            x_opt,
            q_opt,
            t_opt
        );
        std::printf(
            "[bot_dugma::LRBFGSWrapper] optimize: after variableToQuatT "
            "q=(%.6f, %.6f, %.6f, %.6f) t=(%.6f, %.6f, %.6f)\n",
            q_opt[0], q_opt[1], q_opt[2], q_opt[3],
            t_opt[0], t_opt[1], t_opt[2]
        );

        std::array<float, 9> R_r {};
        quatToRFlat(
            q_opt,
            R_r
        );
        std::printf("[bot_dugma::LRBFGSWrapper] optimize: after quatToRFlat\n");

        PoseEstimate pose {};
        for(int idx = 0; idx < 9; ++idx)
        {
            pose.m_R[idx] = R_r[idx];
        }

        for(int idx = 0; idx < kTranslationSize; ++idx)
        {
            pose.m_t[idx] = t_opt[idx];
        }

        pose.m_energy = static_cast<float>(solver.Getfinalfun());
        pose.m_iter = static_cast<int>(solver.GetIter());
        pose.m_is_converged =
            !m_saw_nonfinite_eval &&
            isFinitePose(pose) &&
            solver.GetIter() < solver.Max_Iteration;

        std::printf(
            "[bot_dugma::LRBFGSWrapper] optimize: end converged=%s energy=%.9e iter=%d\n",
            pose.m_is_converged ? "true" : "false",
            static_cast<double>(pose.m_energy),
            pose.m_iter
        );

        return pose;
    }
}
