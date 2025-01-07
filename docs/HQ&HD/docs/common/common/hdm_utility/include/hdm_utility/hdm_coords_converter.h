/******************************************************************************
/                              Copyright
/------------------------------------------------------------------------------
/    Copyright © 2020 SAIC MOTOR Z-ONE SOFTWARE COMPANY.  All rights reserved.
/
/    This software is furnished under a license and may be used and copied
/    only in accordance with the terms of such license and with the inclusion
/    of the above copyright notice. This software or any other copies thereof
/    may not be provided or otherwise made available to any other person.
/    No title to and ownership of the software is hereby transferred.
/
/    The information in this software is subject to change without notice
/    and should not be constructed as a commitment by
/    SAIC MOTOR Z-ONE SOFTWARE COMPANY.
/
/    SAIC MOTOR Z-ONE SOFTWARE COMPANY assumes no responsibility for the use
/    or reliability of its Software on equipment which is not supported by
/    SAIC MOTOR Z-ONE SOFTWARE COMPANY.

/    Date: 2021-8-17
/    Author: tang shaohua
/------------------------------------------------------------------------------
*******************************************************************************/

#ifndef _CHDM_COORDS_CONVERTER_H_
#define _CHDM_COORDS_CONVERTER_H_

#include <vector>
#include "common/data/zone_shared_data.h"


namespace hdm_utility
{	

struct SHdmSplitSegment {
    uint32_t                         length;
    std::vector<zone::common::Point> points;
};

class CHdmCoordsConverter {
public:
    CHdmCoordsConverter(const zone::common::Point &wgs84_vehicle, float64_t heading_angle);
    ~CHdmCoordsConverter();

   static float64_t IntCoord2Degree(int32_t int_coord);
    

    static int32_t Degree2IntCoord(float64_t degree);

    bool Wgs84ToGcj02(float64_t wgLon, float64_t wgLat,float64_t& mgLon,float64_t& mgLat);

    bool Gcj02ToWgs84(float64_t lon, float64_t lat, float64_t& mgLon, float64_t& mgLat);

    /**@brief 计算两点间的距离(wgs84坐标系)
    * @param[in]  p1       线段的端点1（经纬度）
    * @param[in]  p2       线段的端点2（经纬度）

    * @return  距离,单位(米)
    */
     float64_t CalcWgs84Distance(const zone::common::Point &p1, const zone::common::Point &p2);
    
    /**@brief 计算两点间的距离(自车坐标系)
    * @param[in]  p1       线段的端点1（自车坐标系）
    * @param[in]  p2       线段的端点2（自车坐标系）

    * @return  距离,单位(米)
    */
     float64_t CalcVehicleDistance(const zone::common::Point &p1, const zone::common::Point &p2);

    /**@brief 计算前方的点和当前点连线，和正北的夹角（顺时针）
    * @param[in]  cur_point       当前的点
    * @param[in]  ahead_point     前方的点

    * @return  夹角（顺时针）
    */
     float64_t CalcAngle(const zone::common::Point &cur_point, const zone::common::Point &ahead_point);

    /**@brief 计算link上的距离该link起点某距离处的坐标
    * @param[in]  point_list    形状点列（经纬度）
    * @param[in]  distance      前方的距离
    * @param[out] proj_point    该距离处的坐标
    * @param[out] proj_index    该距离在形点上的索引

    * @return  函数执行结果
    */
     void CalculatePointByDistance(const std::vector<zone::common::Point> &point_list, float64_t distance,
                                    zone::common::Point &proj_point, uint16_t &proj_index);

     bool Wgs84ToUtm( const float64_t&	lon, 
					const float64_t& 	lat, 
					const float64_t& 	height,
					float64_t&		 	dx, 
					float64_t&		 	dy, 
					float64_t&		 	dz);
    
     bool UtmToWgs84(const int32_t zone,
                   const float64_t&    dx, 
				   const float64_t&	dy,
                   const float64_t&	dz,
                   float64_t&	lon, 
				   float64_t& 	lat, 
				   float64_t& 	height);

     bool UtmToVehicle(const float64_t&	raw_x_utm, 
					 const float64_t& 	raw_y_utm, 
					 const float64_t& 	raw_z_utm, 
					 const float64_t& 	heading_angle,
					 const float64_t&	in_x_utm, 
					 const float64_t& 	in_y_utm, 
					 const float64_t& 	in_z_utm, 
					 float64_t& 		out_x_vehicle, 
					 float64_t& 		out_y_vehicle, 
					 float64_t& 		out_z_vehicle);


     bool VehicleToUtm(const float64_t&	raw_x_utm, 
					 const float64_t& 	raw_y_utm, 
					 const float64_t& 	raw_z_utm, 
					 const float64_t& 	heading_angle,
					 const float64_t&	x_vehicle, 
					 const float64_t& 	y_vehicle, 
					 const float64_t& 	z_vehicle, 
					 float64_t& 		x_utm, 
					 float64_t& 		y_utm, 
					 float64_t& 		z_utm );

     bool Wgs84ToVehicle(const zone::common::Point  *wgs84_pts,
                        const uint32_t              pts_number,                      
					    zone::common::Point        *vehicle_pts);

     bool VehicleToWgs84(const zone::common::Point  *vehicle_pts, 
                        const uint32_t              pts_number,                       
					    zone::common::Point        *wgs84_pts);
    
    // 用原始的wgs84坐标点列计算出带offset的自车坐标系点列
     bool CaclVehiclePointsAndOffsets(const int32_t  min_offset_range,   // 最小范围offset，单位厘米，规控要求自车后50m,（-5000）
                                            const int32_t  max_offset_range,   // 最大范围offset，单位厘米，规控要求自车前100m,（10000）
                                            const int32_t  start_offset,       // 输入wgs84_pts起点相对于自车位置偏移，单位厘米
                                            const int32_t  end_offset,         // 输入wgs84_pts终点相对于自车位置偏移，单位厘米                                 
                                            const zone::common::Point  *wgs84_pts,           // 要转换的原始wgs84点列
                                            const uint32_t              pts_number,          // 要转换的原始wgs84点数目
                                            std::vector<int32_t>       &vehicle_pts_offset,  // 输出点列的offset，相对自车位置
                                            std::vector<zone::common::Point> &vehicle_pts);  // 输出点列，和vehicle_pts_offset一一对应
private:
    zone::common::Point wgs84_vehicle_;
    float64_t           heading_angle_;
    

};

}

#endif  // _CHDM_COORDS_CONVERTER_H_	
