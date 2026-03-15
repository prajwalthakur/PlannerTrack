#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "mpl_geometry_utils/geometry.hpp"
#include "project_utils/msg/path_point.hpp"
#include "project_utils/msg/trajectory.hpp"


namespace mpl::path_sampler
{
    namespace geometry_utils
    {
        template <typename T1, typename T2>
        bool isSamePoint(const T1& t1, const T2& t2)
        {
            const auto p1 =  mpl_geometry_utils::get_point(t1);
            const auto p2 = mpl_geometry_utils::get_point(t2);
            constexpr double epsilon = 1e-6;
            if(epsilon < std::abs(p1.x-p2.x) && epsilon < std::abs(p1.y-p2.y))
            {
                return true;
            }
            return false;

        }
        template <typename T1, typename T2>
        bool isSamePointEuclid(const T1& t1, const T2& t2)
        {
            const auto p1 =  mpl_geometry_utils::get_point(t1);
            const auto p2 = mpl_geometry_utils::get_point(t2);
            constexpr double epsilon = 1e-6;
            double dx = p1.x-p2.x;
            double dy = p1.y-p2.y;
            if(dx*dx + dy*dy < epsilon*epsilon)
                return true;
            return false;
        }
    }// namespace geometry_utils
}//namespace mpl::path_sampler