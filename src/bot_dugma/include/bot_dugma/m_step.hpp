#ifndef M_STEP_HPP
#define M_STEP_HPP

#include "bot_dugma/types.hpp"
#include "bot_dugma/e_step.hpp"

#include <array>
#include <vector>

namespace bot_dugma
{
    std::array<float, 12> poseToU(
        const PoseEstimate& pose
    );

    void quatToRFlat(
        const std::array<float, 4>& q,
        std::array<float, 9>& R_r
    );

    void poseToQuatT(
        const PoseEstimate& pose,
        std::array<float, 4>& q,
        std::array<float, 3>& t
    );

    float evaluateEnergyAndQuatTGradient(
        const BasisCoeffsVec& A,
        const std::array<float, 4>& q,
        const std::array<float, 3>& t,
        std::array<float, 4>& grad_q,
        std::array<float, 3>& grad_t
    );

    
    class MStep
    {
    public:
        explicit MStep();

        PoseEstimate optimize(
            const BasisCoeffsVec& A,
            const PoseEstimate& initial_pose
        );

    private:
        
    };
}

#endif
