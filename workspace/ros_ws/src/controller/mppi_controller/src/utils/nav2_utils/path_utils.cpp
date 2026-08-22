
#include "mppi_controller/utils/nav2_utils/path_utils.hpp"

/** \file
 * \brief Path-inversion/in-place-rotation detection helpers implementation.
 */

namespace controller::mppi_controller::utils
{
    size_t removePosesAfterFirstConstraint(nav_msgs::msg::Path& path,
        bool enforcePathInversion, 
        float rotationThreshold)
    {
        nav_msgs::msg::Path croppedPath  = path; // why copy ?
        size_t firstAfterConstraint = findFirstPathConstraint(path,enforcePathInversion,rotationThreshold);
        if(firstAfterConstraint==croppedPath.poses.size())
            return 0u;
        croppedPath.poses.erase(croppedPath.poses.begin()+firstAfterConstraint,croppedPath.poses.end());
        path = croppedPath;
        return firstAfterConstraint;
    }

   size_t findFirstPathConstraint(
    nav_msgs::msg::Path & path,
    bool enforcePathInversion,
    float rotationThreshold)
    {
        size_t pathSize = path.poses.size();
         // if the path has less than 3 points, we can not check for rotation or inversion  
        if(pathSize<3){
            return pathSize;
        }
        
        const bool checkRotation = fabs(rotationThreshold) < 1e-6 ? false : true;
        size_t rotationIdx  =  pathSize;
        size_t inversionIdx = pathSize;

        float prevDx = 0.0f;
        float prevDy = 0.0f;

        // Iteration through the path to determine
        // the translational inversion and rotational inversion

        for(size_t idx=0; idx < pathSize; ++idx)
        {
            float dx = path.poses[idx+1].pose.position.x - path.poses[idx].pose.position.x;
            float dy = path.poses[idx+1].pose.position.y - path.poses[idx].pose.position.y;
            float trans = hypot(dx,dy);
            
            // Translation always grow,
            // If already found that there is rotation_idx <= idx +1
            if(rotationIdx <= idx+1){
                break;
            }

            // check translational inversion
            if(enforcePathInversion && ( trans > 1e-4) ){
                if(idx>=1){
                    // check for cusp
                    float dotProduct = prevDx*dx + prevDy*dy;
                    // if the angle between two intermediate vectors is more
                    // than 90(degree), we call it discontinuity
                    if(dotProduct < 0.0f){
                        inversionIdx = idx+1;
                        break;
                    }
                }
                prevDx = dx;
                prevDy = dy;
            }

            // check for in place rotations
            // Geometric global path from path-planners has the
            // information the information about the geometric information also,
            // if there's inplace rotation required then consicutive poses will have the same euclidean point, but different rotations
            if( checkRotation && trans < 1e-4 && rotationIdx == path.poses.size())
            {
                float accumulatedRotation = 0.0f;
                size_t endIdx = idx;
                // Continue checking
                while(endIdx < pathSize)
                {
                    float currentYaw = tf2::getYaw(path.poses[endIdx].pose.orientation);
                    float nextYaw  = tf2::getYaw(path.poses[endIdx+1].pose.orientation);
                    accumulatedRotation += fabs(mppi_utils::shortestAngularDistance(currentYaw, nextYaw));
                    if(accumulatedRotation > rotationIdx){
                        rotationIdx = endIdx + 1;
                        break;
                    }
                    if (endIdx + 2 < path.poses.size()){
                        float ndx = path.poses[endIdx+2].pose.position.x - path.poses[endIdx+1].pose.position.x;
                        float ndy =  path.poses[endIdx+2].pose.position.y - path.poses[endIdx+1].pose.position.y;
                        // translation resumes, after endIdx+1
                        // no in place rotation possible
                        if(hypotf(ndx,ndy)>1e-4)
                        {
                            break;
                        }
                    }else{
                        break;
                    }
                }
            }
        }  
        return std::min(rotationIdx, inversionIdx);
    }
}//namespace controller::mppi_controller::utils