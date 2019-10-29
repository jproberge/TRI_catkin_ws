/*
 * File: calibration_utls.h
 * Description: file with helper functions used to solve for the camera extrinsic
 * calibrations.
 *
 * Author: Jean-Philippe Roberge (jean-philippe.roberge@etsmtl.ca)
 *         Michael Lin (michaelv03@gmail.com)
 * Date: October 28 2019
*/

#ifndef CALIBRATION_UTILS_HPP
#define CALIBRATION_UTILS_HPP

#include <stdlib.h>
#include <Eigen/Dense>

using namespace Eigen;

namespace tri_proj {
    class CalibrationUtils
    {
    public:
        static void compute_extrinsic_calibration(Vector3f pt1_C, Vector3f pt2_C, 
                            Vector3f pt3_C, Vector3f pt1_O, Vector3f pt2_O, Vector3f pt3_O);
    };

}

#endif
