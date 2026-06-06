#ifndef LRBFGS_WRAPPER_HPP
#define LRBFGS_WRAPPER_HPP

#include "bot_dugma/types.hpp"
#include "bot_dugma/e_step.hpp"
#include "Problems/Problem.h"

namespace bot_dugma
{
    class LRBFGSWrapper : public ROPTLIB::Problem
    {
    public:
        explicit LRBFGSWrapper(
            const BasisCoeffsVec& A
        );

        double f(
            const ROPTLIB::Variable& x
        ) const override;

        ROPTLIB::Vector& EucGrad(
            const ROPTLIB::Variable& x,
            ROPTLIB::Vector* result
        ) const override;

        PoseEstimate optimize(
            const PoseEstimate& initial_pose
        );

    private:
        mutable bool m_saw_nonfinite_eval {false};
        const BasisCoeffsVec& m_A;
    };
}


#endif
