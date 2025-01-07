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

#ifndef _CHDM_GEOMETRY_H_
#define _CHDM_GEOMETRY_H_

#include <vector>
#include "common/data/zone_shared_data.h"
#include "hdm_utility/hdm_coords_converter.h"


namespace hdm_utility
{	

class CHdmGeometry {
public:
    CHdmGeometry();
    ~CHdmGeometry();

    static bool Wgs84ToGcj02(float64_t wgLon, float64_t wgLat,float64_t& mgLon,float64_t& mgLat);

    static bool Gcj02ToWgs84(float64_t lon, float64_t lat, float64_t& mgLon, float64_t& mgLat);

    /**@brief 计算点到线段上的投影点
    * @param[in]  point1              直线段上的首点
    * @param[in]  point2              直线段上的尾点
    * @param[in]  p                   某一个点
    * @param[out] foot_point          投影的垂足点

    * @return  投影是否在线段内
    * - true      垂足在线段内
    * - false     垂足在线段外
    */
    static void CalcProjectionPoint(const zone::common::Point &point1, const zone::common::Point &point2, const zone::common::Point &p, 
                        zone::common::Point &foot_point, bool &is_inside);

    /**@brief 获取点到形点串的最近点
    * @param[in]  point_list       形点串
    * @param[in]  p                某一个点
    * @param[out] foot_point       投影到形点串上的垂足
    * @param[out] index            垂足在形点串上的索引
    * @param[out] distance         点到垂足的距离

    * @return  
    */
    static bool GetNearestPoint(const std::vector<zone::common::Point> &point_list, const zone::common::Point &p,
                                uint16_t &index, zone::common::Point &foot_point, float64_t &distance);

    /**@brief 获取点到形点串的垂足点
    * @param[in]  point_list       形点串
    * @param[in]  p                某一个点
    * @param[out] foot_point       投影到形点串上的垂足
    * @param[out] index            垂足在形点串上的索引
    * @param[out] distance         点到垂足的距离
    * @param[out] find_inside      垂足点是否在线里

    * @return  
    */
    static bool GetNearestPoint(const std::vector<zone::common::Point> &point_list, const zone::common::Point &p,
                                uint16_t &index, zone::common::Point &foot_point, float64_t &distance, bool &find_inside);

    /**@brief 判断点是否在线段上
    * @param[in]  p1       线段的端点1
    * @param[in]  p2       线段的端点2
    * @param[in]  p        某一个点

    * @return  函数执行结果
    * - true      在线段内
    * - false     在线段外
    */
    static bool IsPointInside(const zone::common::Point &p1, const zone::common::Point &p2, const zone::common::Point &p);

    /**@brief 计算两点间的距离(wgs84坐标系)
    * @param[in]  p1       线段的端点1（经纬度）
    * @param[in]  p2       线段的端点2（经纬度）

    * @return  距离,单位(米)
    */
    static float64_t CalcWgs84Distance(const zone::common::Point &p1, const zone::common::Point &p2);

    /**
     * @brief 计算线段各点的距离(wgs84坐标系), start_index起始点位置
     * 
     * @param wgs84_pts 
     * @param distance_vec 
     * @return true 
     * @return false 
     */
    static bool CalcWgs84LineDistance(const std::vector<zone::common::Point> &wgs84_pts,
                                      std::vector<float64_t>                 *distance_vec);

    /**
     * @brief 计算线段各点的距离(自车坐标系)
     * 
     * @param vehicle_pts
     * @param distance_vec 
     * @return true 
     * @return false 
     */
    static bool CalcVehicleLineDistance(const std::vector<zone::common::Point>& vehicle_pts,
                                      std::vector<float64_t>                 *distance_vec);

    /**@brief 计算两点间的距离(自车坐标系)
    * @param[in]  p1       线段的端点1（自车坐标系）
    * @param[in]  p2       线段的端点2（自车坐标系）

    * @return  距离,单位(米)
    */
    static float64_t CalcVehicleDistance(const zone::common::Point &p1, const zone::common::Point &p2);

    /**@brief 计算前方的点和当前点连线，和正北的夹角（顺时针）
    * @param[in]  cur_point       当前的点
    * @param[in]  ahead_point     前方的点

    * @return  夹角（顺时针）
    */
    static float64_t CalcAngle(const zone::common::Point &cur_point, const zone::common::Point &ahead_point);

    /**@brief 计算link上的距离该link起点某距离处的坐标
    * @param[in]  point_list    形状点列（经纬度）
    * @param[in]  distance      前方的距离
    * @param[out] proj_point    该距离处的坐标
    * @param[out] proj_index    该距离在形点上的索引

    * @return  函数执行结果
    */
    static void CalculatePointByDistance(const std::vector<zone::common::Point> &point_list, float64_t distance,
                                    zone::common::Point &proj_point, uint16_t &proj_index);

    static bool Wgs84ToUtm(const std::vector<zone::common::Point>  &wgs84_point_list,
                           std::vector<zone::common::Point>        &utm_point_list);

    static bool Wgs84ToUtm( const float64_t&	lon, 
					const float64_t& 	lat, 
					const float64_t& 	height,
					float64_t&		 	dx, 
					float64_t&		 	dy, 
					float64_t&		 	dz);
    
    static bool UtmToWgs84(const int32_t zone,
                   const float64_t&    dx, 
				   const float64_t&	dy,
                   const float64_t&	dz,
                   float64_t&	lon, 
				   float64_t& 	lat, 
				   float64_t& 	height);

    static bool UtmToVehicle(const zone::common::Point  *utm_pts,
                      const uint32_t              pts_number,
                      const float64_t            &heading_angle,
                      const zone::common::Point  &utm_vehicle,
                      zone::common::Point        *vehicle_pts);

    static bool UtmToVehicle(const float64_t&	raw_x_utm, 
					 const float64_t& 	raw_y_utm, 
					 const float64_t& 	raw_z_utm, 
					 const float64_t& 	heading_angle,
					 const float64_t&	in_x_utm, 
					 const float64_t& 	in_y_utm, 
					 const float64_t& 	in_z_utm, 
					 float64_t& 		out_x_vehicle, 
					 float64_t& 		out_y_vehicle, 
					 float64_t& 		out_z_vehicle);


    static bool VehicleToUtm(const float64_t&	raw_x_utm, 
					 const float64_t& 	raw_y_utm, 
					 const float64_t& 	raw_z_utm, 
					 const float64_t& 	heading_angle,
					 const float64_t&	x_vehicle, 
					 const float64_t& 	y_vehicle, 
					 const float64_t& 	z_vehicle, 
					 float64_t& 		x_utm, 
					 float64_t& 		y_utm, 
					 float64_t& 		z_utm );

    static bool Wgs84ToVehicle(const zone::common::Point  *wgs84_pts,
                        const uint32_t              pts_number,
                        const float64_t               &heading_angle,
                        const zone::common::Point  &wgs84_vehicle,  // 自车当前wgs84坐标
					    zone::common::Point        *vehicle_pts);

    static bool Wgs84ToVehicle3(const zone::common::Point  *wgs84_pts,
                        const uint32_t              pts_number,
                        const float64_t               &heading_angle,
                        const zone::common::Point  &wgs84_vehicle,  // 自车当前wgs84坐标
					    zone::common::Point        *vehicle_pts);

    static bool Wgs84ToVehicleSigle(const zone::common::Point  &wgs84_pts,
                        const float64_t               &heading_angle,
                        const zone::common::Point  &wgs84_vehicle,
                        zone::common::Point        *vehicle_pts);

    static bool VehicleToWgs84(const zone::common::Point  *vehicle_pts, 
                        const uint32_t              pts_number,
                        const float64_t                heading_angle,
                        const zone::common::Point  &wgs84_vehicle,  // 自车当前wgs84坐标
					    zone::common::Point        *wgs84_pts);
    //this function will always return true
    static bool VehicleToWgs84v2(const zone::common::Point& vehicle_pts,
                                float64_t                  heading_angle,//unit:degree
                                const zone::common::Point& wgs84_vehicle,
                                zone::common::Point&       out_wgs84_pts);

    static bool VehicleToWgs84v2(const zone::common::Point* vehicle_pts,
                                uint32_t                   pts_number,
                                float64_t                  heading_angle,//unit:degree
                                const zone::common::Point& wgs84_vehicle,
                                zone::common::Point        *wgs84_pts);

    // 用原始的wgs84坐标点列计算出带offset的自车坐标系点列
    static bool CaclVehiclePointsAndOffsets(const int32_t  min_offset_range,   // 最小范围offset，单位厘米，规控要求自车后50m,（-5000）
                                            const int32_t  max_offset_range,   // 最大范围offset，单位厘米，规控要求自车前100m,（10000）
                                            const int32_t  start_offset,       // 输入wgs84_pts起点相对于自车位置偏移，单位厘米
                                            const int32_t  end_offset,         // 输入wgs84_pts终点相对于自车位置偏移，单位厘米
                                            const zone::common::Point  &wgs84_vehicle_pos,   // 自车当前wgs84坐标
                                            const float64_t                heading_angle,       // 自车航向角度, 单位度，以正北方向为0度，顺时钟，[0,360)
                                            const zone::common::Point  *wgs84_pts,           // 要转换的原始wgs84点列
                                            const uint32_t              pts_number,          // 要转换的原始wgs84点数目
                                            std::vector<int32_t>       &vehicle_pts_offset,  // 输出点列的offset，相对自车位置
                                            std::vector<zone::common::Point> &vehicle_pts);  // 输出点列，和vehicle_pts_offset一一对应

    // 用原始的wgs84坐标点列计算出带offset的自车坐标系点列,参考定位方法
    static bool CaclVehiclePointsAndOffsets2(const int32_t  min_offset_range,   // 最小范围offset，单位厘米，规控要求自车后50m,（-10000）
                                             const int32_t  max_offset_range,   // 最大范围offset，单位厘米，规控要求自车前100m,（20000）
                                             const int32_t  start_offset,       // 输入wgs84_pts起点相对于自车位置偏移，单位厘米
                                             const int32_t  end_offset,         // 输入wgs84_pts终点相对于自车位置偏移，单位厘米
                                             const zone::common::Point   &wgs84_vehicle_pos,     // 自车当前wgs84坐标
                                             const float64_t              heading_angle,       // 自车航向角度, 单位度，以正北方向为0度，顺时钟，[0,360)
                                             const zone::common::Point   *wgs84_pts,           // 要转换的原始wgs84点列
                                             const uint32_t               pts_number,          // 要转换的原始wgs84点数目
                                             std::vector<int32_t>        &vehicle_pts_offset,  // 输出点列的offset，相对自车位置
                                             std::vector<zone::common::Point> &vehicle_pts,    // 输出点列，和vehicle_pts_offset一一对应
                                             int32_t& start_index,                             // 输出点列的起点对应输入点列wgs84_pts中的索引
                                             int32_t& end_index,                               // 输出点列的终点对应输入点列wgs84_pts中的索引
                                             const std::vector<float64_t>     &distance_vec);

    // 用原始的wgs84坐标点列计算出带offset的自车坐标系点列,参考定位方法
    static bool CaclVehiclePointsAndOffsets3(const int32_t  min_offset_range,   // 最小范围offset，单位厘米，规控要求自车后50m,（-10000）
                                             const int32_t  max_offset_range,   // 最大范围offset，单位厘米，规控要求自车前100m,（20000）
                                             const int32_t  start_offset,       // 输入wgs84_pts起点相对于自车位置偏移，单位厘米
                                             const int32_t  end_offset,         // 输入wgs84_pts终点相对于自车位置偏移，单位厘米
                                             const zone::common::Point   &wgs84_vehicle_pos,     // 自车当前wgs84坐标
                                             std::vector<zone::common::Point>            &point_xy_list, // WGS84形状点相对自车的投影
                                             int32_t                      &point_xy_min_index, // pre min_index
                                             int32_t                      &point_xy_max_index, // pre max_index
                                             const float64_t              heading_angle,       // 自车航向角度, 单位度，以正北方向为0度，顺时钟，[0,360)
                                             const zone::common::Point   *wgs84_pts,           // 要转换的原始wgs84点列
                                             const uint32_t               pts_number,          // 要转换的原始wgs84点数目
                                             std::vector<int32_t>        &vehicle_pts_offset,  // 输出点列的offset，相对自车位置
                                             std::vector<zone::common::Point> &vehicle_pts,    // 输出点列，和vehicle_pts_offset一一对应
                                             const std::vector<float64_t>     &distance_vec);    


    // 用原始的wgs84坐标点列计算出带offset的平面直角坐标系（以自车为原点，不考虑自车航向角）点列,参考定位方法
    static bool CaclCartesianPointsAndOffsets(const int32_t  min_offset_range,   // 最小范围offset，单位厘米，规控要求自车后50m,（-10000）
                                              const int32_t  max_offset_range,   // 最大范围offset，单位厘米，规控要求自车前100m,（20000）
                                              const int32_t  start_offset,       // 输入wgs84_pts起点相对于自车位置偏移，单位厘米
                                              const int32_t  end_offset,         // 输入wgs84_pts终点相对于自车位置偏移，单位厘米
                                              const zone::common::Point& wgs84_vehicle_pos,    // 自车当前wgs84坐标
                                              const zone::common::Point* wgs84_pts,            // 要转换的原始wgs84点列
                                              const uint32_t               pts_number,         // 要转换的原始wgs84点数目
                                              std::vector<int32_t>& vehicle_pts_offset,        // 输出点列的offset，相对自车位置
                                              std::vector<zone::common::Point>& vehicle_pts,   // 输出点列，和vehicle_pts_offset一一对应
                                              const std::vector<float64_t>& distance_vec);     // 输入点列的间距，单位厘米

    // 用平面直角坐标系和wgs84原点计算出当前自车坐标系点列和offset
    static bool CaclVehiclePointsAndOffsetsByCartesian(
                                            const std::vector<zone::common::Point>& input_pts,          // 平面直角坐标系的点列
                                            const std::vector<int32_t>&             input_pts_offset,   // 平面直角坐标系的点列相对于原点的offset
                                            const zone::common::Point&              wgs84_origin_pos,   // 平面直角坐标系原点的wgs84坐标
                                            const uint32_t                          origin_offset,      // 参考原点相对于path起点的距离
                                            const zone::common::Point&              wgs84_vehicle_pos,  // 自车当前wgs84坐标
                                            const uint32_t                          vehicle_offset,     // 自车相对于path起点的距离
                                            const float64_t                         heading_angle,      // 自车航向角度, 单位度，以正北方向为0度，顺时钟，[0,360)
                                            std::vector<zone::common::Point>&       vehicle_pts,        // 转换后的自车坐标系点列
                                            std::vector<int32_t>&                   vehicle_pts_offset);// 输出点列的offset，相对自车位置

    static bool CaclPointByVehiclePointsAndOffsets(const int32_t* vehicle_pts_offset,  // 输入坐标点偏移
                                        const zone::common::Point       *vehicle_pts,   // 输入坐标点
                                        uint32_t                   vehicle_pts_num,     // 输入坐标点数目
                                        int32_t                    point_offset,        // 要计算的点的距离
                                        zone::common::Point             &point,         // 要计算的点
                                        int32_t                   &index,               // 计算点的前一个原始点的索引
                                        bool                      &is_origin,           // 计算点是否就是原始点
                                        bool                      start_flag=true);     // true: start, false: end

    /**
     * @brief 根据start_offset和end_offset，切割二者之间的所有形状点，
     * 如果start_offset不在形状点列内，要把start_offset对应的点加入起点，
     * 如果end_offset不在形状点列内，要把start_offset对应的点加入终点。
     * 
     * @param vehicle_pts_offset 输入坐标点偏移
     * @param vehicle_pts 输入坐标点
     * @param vehicle_pts_num 输入坐标点数目
     * @param start_offset 切割的开始offset，单位厘米，相对自车位置
     * @param end_offset 切割的终止offset，单位厘米，相对自车位置
     * @param split_vehicle_pts 输出点列
     * @return true 
     * @return false 
     */
    static bool SplitVehiclePoints(const int32_t             *vehicle_pts_offset, // 输入坐标点偏移
                                   const zone::common::Point *vehicle_pts,        // 输入坐标点
                                   uint32_t                   vehicle_pts_num,    // 输入坐标点数目
                                   int32_t                    start_offset,       // 切割的开始offset，单位厘米，相对自车位置
                                   int32_t                    end_offset,         // 切割的终止offset，单位厘米，相对自车位置
                                   std::vector<zone::common::Point> &split_vehicle_pts); // 输出点列
 
    /**
     * @brief 根据start_offset和end_offset，切割二者之间的所有形状点，
     * 如果start_offset不在形状点列内，要把start_offset对应的点加入起点，
     * 如果end_offset不在形状点列内，要把start_offset对应的点加入终点。
     * 
     * @param vehicle_pts_offset 输入坐标点偏移
     * @param vehicle_pts 输入坐标点
     * @param vehicle_pts_num 输入坐标点数目
     * @param start_offset 切割的开始offset，单位厘米，相对自车位置
     * @param end_offset 切割的终止offset，单位厘米，相对自车位置
     * @param split_vehicle_pts_offset 输出点列对应的offset
     * @param split_vehicle_pts 输出点列
     * @return true 
     * @return false 
     */
    static bool SplitVehiclePoints(const int32_t             *vehicle_pts_offset,
                                   const zone::common::Point *vehicle_pts,
                                   uint32_t                   vehicle_pts_num,
                                   int32_t                    start_offset, 
                                   int32_t                    end_offset,
                                   std::vector<int32_t>      &split_vehicle_pts_offset,
                                   std::vector<zone::common::Point> &split_vehicle_pts);


    /**
     * @brief 切割成虚线线段
     * 
     * @param vehicle_pts_offset 
     * @param vehicle_pts 
     * @param first_seg_length 第一段长段, 即画线的长度, 单位厘米
     * @param second_seg_length  第二段长段, 即空白的长度, 单位厘米
     * @param segments 段 
     * @return true 
     * @return false 
     */
    static bool Split2DashedLineSegments(const std::vector<int32_t>              &vehicle_pts_offset,
                                         const std::vector<zone::common::Point>  &vehicle_pts,
                                         int32_t                                 first_seg_length,
                                         int32_t                                 second_seg_length, 
                                         std::vector<SHdmSplitSegment>           *segments);
    
    static bool ShiftVehiclePoints(const zone::common::Point *vehicle_pts,
                                   uint32_t                   vehicle_pts_num,
                                   bool                       is_shift_left,
                                   uint32_t                   shift_dist,
                                   std::vector<zone::common::Point> &shift_pts);

    /**
     * @brief 判断pta位于有向直线p1->p2的左边还是右边
     * 
     * @param p1 起始点
     * @param p2 结束点
     * @param pta 待判断点
     * @return 1: pta位于 p1->p2右边,-1: pta位于 p1->p2左边,0: pta位于p1->p2上(p1->p2和p1->pt共线)
     */
    static int calculateLineSide(const zone::common::Point& pt1, const zone::common::Point& pt2, const zone::common::Point& pt);

};

}

#endif  // _CHDM_GEOMETRY_H_	
