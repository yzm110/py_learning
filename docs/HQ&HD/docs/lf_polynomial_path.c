/* PRQA S 0292 17 */
/*********************************************************************
    MODULE SPECIFICATION:

        $ProjectName: /DAS/060_Projects/BYD/SCam4__/20_SCam/30_Software/50_Construction/Application/SourceCode/Algo/FUSION/Project/sources/LaneFusion/Util/project.pj $
        $RCSfile: lf_polynomial_path.c $
        $Revision: 1.28 $
        $Date: 2020/10/02 21:26:35CST $

        TRW Ltd owns the copyright in this document and associated
        documents and all rights are reserved.  These documents must
        not be used for any purpose other than that for which they
        are supplied and must not be copied in whole or in part or
        disclosed to others without prior written consent of TRW
        Ltd. Any copy of this drawing or document made by any method
        must also include a copy of this legend.

    $CopyrightDate: (c) ZF 2019 $

*********************************************************************/

/*************************************************************/
/*      INCLUDES                                             */
/*************************************************************/

/* --- General         --- */
#include "../Database/lf_common.h"

/* --- Module header   --- */
#include "lf_polynomial_path.h"

/*************************************************************/
/*   PRIVATE FUNCTION DECLARATIONS                           */
/*************************************************************/

/**
 * @details This function returns the x which minimizes the distances of point (px,py) to (x, poly(x)).
 *          Uses the numeric newton approach.
 * @param[out] 'p_x_nearest_f32' A best guess of x which minimizes the distance from 'p_point_s' to 'p_polynomial_s(x)'. At least as good as 'start_x_f32'.
 * @param[in] 'p_polynomial_s' polynomial path
 * @param[in] 'point_s' point to project onto polynomial.
 * @param[in] 'start_x_f32' value for x to start newton iteration.
 * @param[in] 'nr_iterations_u8' maximum nr. of iterations.
 * @return TRUE if a (at least a local) minimum of the distance from 'point_s' to 'p_bipolynomial_s' is found, FALSE otherwise.
 */
PRIVATE BOOLEAN closest_point_along_polynomial_path( F32* const p_x_nearest_f32, const POLYNOMIAL3* const p_polynomial_s, const LF_POINT_2D point_s, const F32 start_x_f32, U8 nr_iterations_u8 );

/*************************************************************/
/*   FUNCTION DEFINITIONS                                    */
/*************************************************************/

POLYNOMIAL3 lf_extrapolate_polynomial_constant_curvature( const POLYNOMIAL3* const p_original_polynomial_s, const F32 view_range_x_f32 )
{
    POLYNOMIAL3 extrapolated_polynomial_s;

    /* Translate polynomial to viewrange distance */
    polynomial3_translate( &extrapolated_polynomial_s, p_original_polynomial_s, -view_range_x_f32, 0.0F );

    /* remove curvature rate */
    extrapolated_polynomial_s.c3_f32 = 0.0F;

    /* Translate polynomial back to origin */
    polynomial3_translate( &extrapolated_polynomial_s, &extrapolated_polynomial_s, view_range_x_f32, 0.0F );

    return extrapolated_polynomial_s;
}


LF_POSE_2D lf_evaluate_polynomial_path( const POLYNOMIAL3* const p_trajectory_s, F32 x_f32 )
{
    F32 y_derivative_f32;
    F32 sin_of_theta_f32;
    F32 cos_of_theta_f32;
    LF_POSE_2D feature_s;

    y_derivative_f32 = polynomial3_derive( p_trajectory_s, x_f32 );
    sin_of_theta_f32 = 1.0F / sqrtf(1.0F + ( y_derivative_f32 * y_derivative_f32 ) );
    cos_of_theta_f32 = y_derivative_f32 * sin_of_theta_f32 * (-1.0F);

    feature_s.position_s.x_f32 = x_f32;
    feature_s.position_s.y_f32 = polynomial3_solve(p_trajectory_s, x_f32);

    feature_s.orientation_s.sin_f32 = sin_of_theta_f32;
    feature_s.orientation_s.cos_f32 = cos_of_theta_f32;

    return feature_s;
}


LF_POSE_2D lf_transform_point_to_polynomial_path( const POLYNOMIAL3* const p_trajectory_s , const LF_POINT_2D point_s, LF_POINT_2D * const p_nearest_point_on_centered_path_s )
{
    /* locals */
    F32        nearest_x_f32;
    LF_POSE_2D nearest_point_on_path_s;
    LF_POSE_2D point_path_coords_s;

    /* config */
    const U8  nr_newton_raphson_iterations_u8 = 5U;

    if( NULL != p_trajectory_s)
    {
        /* get point on polynomial that is closest to point_s */
        (void)closest_point_along_polynomial_path( &nearest_x_f32, p_trajectory_s,
                                                   point_s, point_s.x_f32,
                                                   nr_newton_raphson_iterations_u8 );

        nearest_point_on_path_s = lf_evaluate_polynomial_path( p_trajectory_s, nearest_x_f32 );

        /* x_pc approximation */
        point_path_coords_s.position_s.x_f32 =    nearest_point_on_path_s.position_s.x_f32
                                               - (nearest_point_on_path_s.orientation_s.cos_f32 * p_trajectory_s->c0_f32);

        /* y_pc := |point - path| + |path - host_vehicle_center| */
        point_path_coords_s.position_s.y_f32 = (    SGN_f32( point_s.y_f32 - nearest_point_on_path_s.position_s.y_f32 )           /* direction path->point vector      */
                                                  * SGN_f32( nearest_point_on_path_s.orientation_s.sin_f32 )                      /* negative if path runs upside down */
                                                  * lf_ut_distance_between_points( point_s, nearest_point_on_path_s.position_s )) /* length of path->point vector      */
                                               + p_trajectory_s->c0_f32;
        /* path direction */
        point_path_coords_s.orientation_s = nearest_point_on_path_s.orientation_s;


        if(NULL != p_nearest_point_on_centered_path_s)
        {
            p_nearest_point_on_centered_path_s->x_f32 = point_path_coords_s.position_s.x_f32;
            p_nearest_point_on_centered_path_s->y_f32 =   nearest_point_on_path_s.position_s.y_f32     //  注意这不是point_path_coords_s 是 nearest_point_on_path_s
                                                       - (nearest_point_on_path_s.orientation_s.sin_f32 * p_trajectory_s->c0_f32);   // 最终的结果就是法线方向平移到中心点
        }
        else
        {
            /* optional output not requested */
        }
    }
    else
    {
        ASSERT(FALSE)
        (void)memset(&point_path_coords_s, 0, sizeof(LF_POSE_2D));
    }

    return point_path_coords_s;
}


PRIVATE BOOLEAN closest_point_along_polynomial_path( F32* const p_x_nearest_f32, const POLYNOMIAL3* const p_polynomial_s, const LF_POINT_2D point_s, const F32 start_x_f32, U8 nr_iterations_u8 )
{
    U8      it_u8;                       /* current iteration */
    F32     x__f32       = start_x_f32;  /* current resulting x */
    F32     dx_f32;                      /* iteration range */
    BOOLEAN successful_b = FALSE;        /* indicates whether algorithm converged at an distance-minimum */

    /* p(x) := path-function */
    F32 p_x_diff_f32         = start_x_f32 - point_s.x_f32;                                     /*   x - px   */
    F32 p_y_diff_f32         = polynomial3_solve(p_polynomial_s, start_x_f32) - point_s.y_f32;  /* p(x)- py   */
    F32 p_derivative_f32     = polynomial3_derive(p_polynomial_s, start_x_f32);                 /* p'(x)      */
    F32 p_2nd_derivative_f32 = polynomial3_derive_twice(p_polynomial_s, start_x_f32);           /* p''(x)     */

    /* d(x) := square distance function */
    F32 d_start_f32          = SQUARE(p_x_diff_f32) + SQUARE(p_y_diff_f32);                          /* d(x_start) */
    F32 d_2nd_derivative_f32 = (p_y_diff_f32*p_2nd_derivative_f32)+SQUARE(p_derivative_f32)+1.0F;    /* d''(x) / 2 */

    /* Config */
    const F32 max_delta_f32 = 0.05F;                           /* minimum requested accuracy */

    for ( it_u8 = 0U; it_u8 < (nr_iterations_u8+1U); it_u8++ ) /* nr_iteration+1 : allow success-check without performance reduction */
    {
        if ( fabsf(d_2nd_derivative_f32) > SMALL_NUMBER )
        {
            /* Newton Raphson iteration to search for zero of d'(x) : x_{n+1} = x_n - d'(x_n) / d''(x_n)*/
            dx_f32 = (p_x_diff_f32 + (p_derivative_f32*p_y_diff_f32)) / d_2nd_derivative_f32;
        }
        else
        {
            /* stuck on inflection point of d'(x), add small step to continue searching for zero of d'(x) : x_{n+1} = x_n + eps*sgn(d'''(x_n)) */
            dx_f32 = -2.0F * max_delta_f32 * SGN_f32( (3.0F * p_derivative_f32 * p_2nd_derivative_f32) + (6.0F * p_polynomial_s->c3_f32 * p_y_diff_f32) );
        }

        /* do iteration step */
        x__f32 = x__f32 - dx_f32;

        /* evaluate path and distance */
        p_x_diff_f32         = x__f32 - point_s.x_f32;
        p_y_diff_f32         = polynomial3_solve(p_polynomial_s, x__f32) - point_s.y_f32;
        p_derivative_f32     = polynomial3_derive(p_polynomial_s, x__f32);
        p_2nd_derivative_f32 = polynomial3_derive_twice(p_polynomial_s, x__f32);
        d_2nd_derivative_f32 = (p_y_diff_f32*p_2nd_derivative_f32)+SQUARE(p_derivative_f32)+1.0F;

        /* stop if accuracy is better than MAX_DELTA_X */
        if( fabsf(dx_f32) < max_delta_f32 )
        {
            successful_b = (d_2nd_derivative_f32 > 0.0F) ? TRUE : FALSE;    /* check if minimum or maximum is found */
            break;
        }
        else
        {
            /* continue searching */
        }
    }

    /* output */
    if(d_start_f32 >= (SQUARE(p_x_diff_f32) + SQUARE(p_y_diff_f32)))
    {
        *p_x_nearest_f32 = x__f32;      /* estimated value is valid */
    }
    else
    {
        *p_x_nearest_f32 = start_x_f32;  /* start value is better than estimated value */
        successful_b = FALSE;
    }

    return successful_b;
}
