/*
 * File: calibration_utls.cpp
 * Description: file with helper functions used to solve for the camera extrinsic
 * calibrations.
 *
 * Author: Jean-Philippe Roberge (jean-philippe.roberge@etsmtl.ca)
 *         Michael Lin (michaelv03@gmail.com)
 * Date: October 28 2019
*/

#include <math.h>
#include "calibration_utils.h"


namespace tri_proj {

    void CalibrationUtils::compute_extrinsic_calibration(Vector3f pt1_C, Vector3f pt2_C, 
                                Vector3f pt3_C, Vector3f pt1_O, Vector3f pt2_O, Vector3f pt3_O)
    {
        // Get the unit vector x pointing from pt1 to pt2
        double x_vec_len = (pt2_C - pt1_C).norm();
        Vector3f x_hat_C_F = (pt2_C - pt1_C)/x_vec_len;

        // Get the unit vector y pointing from pt1 to pt3
        double y_vec_len = (pt3_C - pt1_C).norm();
        Vector3f y_hat_C_F = (pt3_C - pt1_C)/y_vec_len;

        Vector3f z_hat_C_F = x_hat_C_F.cross(y_hat_C_F);

        Matrix3f R;
        R << x_hat_C_F[0], y_hat_C_F[0], z_hat_C_F[0],
             x_hat_C_F[1], y_hat_C_F[1], z_hat_C_F[1],
             x_hat_C_F[2], y_hat_C_F[2], z_hat_C_F[2];

        Matrix4f T_C_F;
        T_C_F.block<3,3>(0,0) = R;
        T_C_F.block<3,1>(0,3) = pt1_C;

    }

}
