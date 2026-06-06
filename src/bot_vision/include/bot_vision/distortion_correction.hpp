#ifndef DISTORTION_CORRECTION_HPP
#define DISTORTION_CORRECTION_HPP

#include "bot_vision/feature_tracks.hpp"
#include "bot_vision/camera_context.hpp"
#include <vector>


namespace bot_vision::distortion_correction
{
    feature_tracks::CurrentFrameObservation uncertaintyAwareDistortionCorrection(
        const std::vector<camera_context::CameraCalibration>& cam_info_vector,
        const feature_tracks::CurrentFrameObservation& observations
    );
}


#endif
