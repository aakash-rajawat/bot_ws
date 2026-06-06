#include "bot_dugma/types.hpp"

namespace bot_dugma
{
    PointCloudView makePointCloudView(const PointCloud& pt_cloud)
    {
        PointCloudView view {};

        view.m_x = pt_cloud.m_x;
        view.m_y = pt_cloud.m_y;
        view.m_z = pt_cloud.m_z;

        view.m_Sxx = pt_cloud.m_Sxx;
        view.m_Sxy = pt_cloud.m_Sxy;
        view.m_Sxz = pt_cloud.m_Sxz;
        view.m_Syy = pt_cloud.m_Syy;
        view.m_Syz = pt_cloud.m_Syz;
        view.m_Szz = pt_cloud.m_Szz;

        view.m_timestamp_sec = pt_cloud.m_timestamp_sec;
        view.m_frame_id = pt_cloud.m_frame_id;

        return view;
    }
}
