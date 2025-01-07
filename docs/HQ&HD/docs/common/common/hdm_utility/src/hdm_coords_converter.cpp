#include "hdm_utility/hdm_coords_converter.h"
#include "geographic_transform/geographic_transform.h"
#include "hdm_utility/hdm_dbg_log.h"
#include "geographic_transform/geographic_geometry.h"

using namespace std;
using namespace geographic_transform;

#define HDM_TEST_OUTPUT_DEBUG 0
namespace hdm_utility
{

CHdmCoordsConverter::CHdmCoordsConverter(const zone::common::Point &wgs84_vehicle_, float64_t heading_angle_)
: wgs84_vehicle_(wgs84_vehicle_)
, heading_angle_(heading_angle_)
{    

}

CHdmCoordsConverter::~CHdmCoordsConverter()
{

}

float64_t CHdmCoordsConverter::IntCoord2Degree(int32_t int_coord)
{
    return ((float64_t)int_coord) / 2147483648.0 * 180.0;
}

int32_t CHdmCoordsConverter::Degree2IntCoord(float64_t degree)
{
    return degree / 180.0 * 2147483648.0;
}

bool CHdmCoordsConverter::Wgs84ToGcj02(float64_t wgLon, float64_t wgLat,float64_t& mgLon,float64_t& mgLat)  
{
    GeoPoint wgs84(wgLat, wgLon, 0);
    GeoPoint gcj02;
    wgs84.Wgs84LatLon_2_Gcj02LatLon(wgs84, &gcj02);
    mgLon = gcj02.longitude();
    mgLat = gcj02.latitude();
    return true;
}

bool CHdmCoordsConverter::Gcj02ToWgs84(float64_t lon, float64_t lat, float64_t& mgLon, float64_t& mgLat )
{
    GeoPoint gcj02(lat, lon);
    GeoPoint wgs84;
    gcj02.Gcj02LatLon_2_Wgs84LatLon(gcj02, &wgs84);
    mgLon = wgs84.longitude();
    mgLat = wgs84.latitude();
    return true;
}

float64_t CHdmCoordsConverter::CalcVehicleDistance(const zone::common::Point &p1, const zone::common::Point &p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

float64_t CHdmCoordsConverter::CalcWgs84Distance(const zone::common::Point &p1, const zone::common::Point &p2)
{
    GeoPoint gp1(p1.y, p1.x, p1.z);
    GeoPoint gp2(p2.y, p2.x, p2.z);
    return gp1.distance(gp2);
}

bool CHdmCoordsConverter::Wgs84ToUtm(const float64_t &lon, const float64_t& lat, const float64_t& 	height,
					float64_t&	dx, float64_t& dy, float64_t& dz)
{
    GeoPoint geo_transform;
    geo_transform.lla2utm(lat, lon, false, &dx, &dy);
    return true;    
}
    
bool CHdmCoordsConverter::UtmToWgs84(const int32_t zone, const float64_t& dx, const float64_t&	dy, const float64_t&	dz,
                   float64_t& lon,  float64_t& lat, float64_t& height)
{
    GeoPoint geo_transform;
	  geo_transform.utm2lla(zone, false, dx, dy, &lat, &lon);
    return true;
}

bool CHdmCoordsConverter::UtmToVehicle(const float64_t &raw_x_utm, const float64_t &raw_y_utm, const float64_t &raw_z_utm, 
        const float64_t &heading_angle_, const float64_t &in_x_utm, const float64_t &in_y_utm, const float64_t &in_z_utm, 
                    float64_t &out_x_vehicle, float64_t &out_y_vehicle, float64_t &out_z_vehicle)
{
    GeoPoint geo_transform;
    geo_transform.utm2vehicle(raw_x_utm, raw_y_utm, raw_z_utm, heading_angle_,
					         in_x_utm, in_y_utm, in_z_utm, out_x_vehicle, out_y_vehicle, out_z_vehicle);
    return true;   
}

bool CHdmCoordsConverter::VehicleToUtm(const float64_t&	raw_x_utm, const float64_t& 	raw_y_utm,  const float64_t& 	raw_z_utm, 
					 const float64_t& 	heading_angle_, const float64_t&	x_vehicle,  const float64_t& 	y_vehicle,  const float64_t& 	z_vehicle, 
					 float64_t& 		x_utm,  float64_t& 		y_utm,  float64_t& 		z_utm )
{
    GeoPoint geo_transform;
    geo_transform.vehicle2utm(raw_x_utm, raw_y_utm, raw_z_utm, heading_angle_,
					         x_vehicle, y_vehicle, z_vehicle, x_utm, y_utm, z_utm);
    return true;  
}

bool CHdmCoordsConverter::Wgs84ToVehicle(const zone::common::Point  *wgs84_pts,
                        const uint32_t              pts_number,                     
                        zone::common::Point        *vehicle_pts)
{
    zone::common::Point utm_vehicle;
    zone::common::Point in_utm_point;

    CHdmCoordsConverter::Wgs84ToUtm(wgs84_vehicle_.x, wgs84_vehicle_.y, wgs84_vehicle_.z, utm_vehicle.x, utm_vehicle.y, utm_vehicle.z);
    for (uint32_t i = 0; i <  pts_number; ++i)    {
        CHdmCoordsConverter::Wgs84ToUtm(wgs84_pts[i].x, wgs84_pts[i].y, wgs84_pts[i].z, in_utm_point.x, in_utm_point.y, in_utm_point.z);
        CHdmCoordsConverter::UtmToVehicle(utm_vehicle.x, utm_vehicle.y, utm_vehicle.z, heading_angle_, 
            in_utm_point.x, in_utm_point.y, in_utm_point.z, vehicle_pts[i].x, vehicle_pts[i].y, vehicle_pts[i].z);
    };
    return true;
}

bool CHdmCoordsConverter::VehicleToWgs84(const zone::common::Point  *vehicle_pts, 
                        const uint32_t              pts_number,                      
					              zone::common::Point        *wgs84_pts)
{
    zone::common::Point utm_vehicle;
    zone::common::Point out_utm_point;

    CHdmCoordsConverter::Wgs84ToUtm(wgs84_vehicle_.x, wgs84_vehicle_.y, wgs84_vehicle_.z, utm_vehicle.x, utm_vehicle.y, utm_vehicle.z);
    for (uint32_t i = 0; i <  pts_number; ++i)    {
        CHdmCoordsConverter::VehicleToUtm(utm_vehicle.x, utm_vehicle.y, utm_vehicle.z, heading_angle_, 
              vehicle_pts[i].x, vehicle_pts[i].y, vehicle_pts[i].z, out_utm_point.x, out_utm_point.y, out_utm_point.z);
        CHdmCoordsConverter::UtmToWgs84((int32_t(wgs84_vehicle_.x / 6) + 31), out_utm_point.x, out_utm_point.y, out_utm_point.z, wgs84_pts[i].x, wgs84_pts[i].y, wgs84_pts[i].z);    
    };
    return true;
}

// 用原始的wgs84坐标点列计算出带offset的自车坐标系点列
bool CHdmCoordsConverter::CaclVehiclePointsAndOffsets(const int32_t  min_offset_range,   // 最小范围offset，单位厘米，规控要求自车后50m,（-5000）
                                               const int32_t  max_offset_range,   // 最大范围offset，单位厘米，规控要求自车前100m,（10000）
                                               const int32_t  start_offset,       // 输入wgs84_pts起点相对于自车位置偏移，单位厘米
                                               const int32_t  end_offset,         // 输入wgs84_pts终点相对于自车位置偏移，单位厘米                                               
                                               const zone::common::Point  *wgs84_pts,           // 要转换的原始wgs84点列
                                               const uint32_t              pts_number,          // 要转换的原始wgs84点数目
                                               std::vector<int32_t>       &vehicle_pts_offset,  // 输出点列的offset，相对自车位置
                                               std::vector<zone::common::Point> &vehicle_pts)   // 输出点列，和vehicle_pts_offset一一对应
{   
#if HDM_TEST_OUTPUT_DEBUG
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "CHdmCoordsConverter::CaclVehiclePointsAndOffsets, pts_number: %d.\n", pts_number);
    
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, 
              "input  pts_number: %d, min_offset_range: %d, max_offset_range: %d, start_offset: %d, end_offset: %d, heading_angle_: %f, vehicle_pts_offset_size[%u],",
              pts_number, min_offset_range, max_offset_range, start_offset, end_offset, heading_angle_, vehicle_pts_offset.size());
    zone::common::Point wgs84_point;
         CHdmCoordsConverter::Gcj02ToWgs84(wgs84_vehicle_pos.x, wgs84_vehicle_pos.y, wgs84_point.x, wgs84_point.y);
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, 
              "wgs84_vehicle_pos[x:%f, y:%f]\n", wgs84_point.x, wgs84_point.y); 
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "inputpoint:");
    for (uint32_t i = 0; i < pts_number; ++i) {
         zone::common::Point wgs84_point;
         CHdmCoordsConverter::Gcj02ToWgs84(wgs84_pts[i].x, wgs84_pts[i].y, wgs84_point.x, wgs84_point.y);
         HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "[%d: %f, %f]", i, wgs84_point.x, wgs84_point.y);    
    }
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "\n");
#endif

    if (nullptr == wgs84_pts) {
        HDM_ERROR("CHdmCoordsConverter::CaclVehiclePointsAndOffsets","wgs84_pts is nullptr.\n");
        return false;
    }

    if (pts_number <= 0) {
        HDM_ERROR("CHdmCoordsConverter::CaclVehiclePointsAndOffsets", "pts_number <= 0.\n");
        return false;
    }
    
    if (start_offset >= end_offset) {
#if HDM_TEST_OUTPUT_DEBUG
        HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true,
                  "start_offset >= end_offset: start_offset:%d, end_offset:%d\n", start_offset, end_offset);
#endif
        return false;
    }

    std::vector<int32_t>               temp_vehicle_pts_offset;
    std::vector<zone::common::Point>   temp_vehicle_pts;
    temp_vehicle_pts_offset.reserve(pts_number);
    temp_vehicle_pts.resize(pts_number);
    // wgs84转换车身坐标系
    CHdmCoordsConverter::Wgs84ToVehicle(wgs84_pts, pts_number, &temp_vehicle_pts[0]);

    /// 求出总长度
    float64_t               sum_dis      = 0;
    float64_t               delta_offset = end_offset - start_offset;
    std::vector<float64_t>  distance_vec;
    distance_vec.reserve(pts_number);
    distance_vec.push_back(0);
    for (uint32_t i = 1; i < pts_number; i++)
    {
        float64_t point_dis =  CHdmCoordsConverter::CalcWgs84Distance(wgs84_pts[i - 1], wgs84_pts[i]);
        sum_dis += point_dis;
        distance_vec.push_back(sum_dis);
    };
    // 计算offset换算比例
    float64_t percent = delta_offset / sum_dis;

    // 计算输出点列的offset
    for (float64_t& distance : distance_vec) {
        int32_t offset = start_offset + distance * percent;
        temp_vehicle_pts_offset.push_back(offset);
    }
    
    /// 不在范围内
    if (temp_vehicle_pts_offset[0] > max_offset_range) {  // 在range的前方
#if HDM_TEST_OUTPUT_DEBUG
        HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true,
                  "OutRange: temp_vehicle_pts_offset[0]:%d, max_offset_range:%d\n", temp_vehicle_pts_offset[0], max_offset_range);
        HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "offset: ");
        for (int i = 0; i < temp_vehicle_pts_offset.size(); ++i) {
            HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "[%d: %d]", i, temp_vehicle_pts_offset[i]);
        }
        HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "\n");
#endif
        return true;
    }
    if (temp_vehicle_pts_offset.back() < min_offset_range) {  // 在range的后方
#if HDM_TEST_OUTPUT_DEBUG
        HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true,
                  "OutRange: temp_vehicle_pts_offset[-1]:%d, min_offset_range:%d\n", temp_vehicle_pts_offset.back(), min_offset_range);
#endif
        return true;
    }
    int32_t min_index = 0;
    int32_t max_index = pts_number - 1;
    // 在范围内
    for (int32_t i = 0; i < static_cast<int32_t>(pts_number - 1); i++)
    {
        if ((temp_vehicle_pts_offset[i] <= min_offset_range) && 
           (min_offset_range < temp_vehicle_pts_offset[i+1])) {
            min_index = i;
        }
        else if ((temp_vehicle_pts_offset[i] < max_offset_range) &&
                 (max_offset_range <= temp_vehicle_pts_offset[i+1])) {
            max_index = i + 1;
            break;
        }
        else {
            continue;
        }
    }
    // 更新输出点列的offset和输出的点列
    size_t size = max_index - min_index + 1;
    vehicle_pts_offset.reserve(vehicle_pts_offset.size() + size);
    std::copy(temp_vehicle_pts_offset.begin() + min_index, temp_vehicle_pts_offset.begin() + max_index + 1,
              std::back_inserter(vehicle_pts_offset));
    // vehicle_pts_offset.insert(vehicle_pts_offset.end(), temp_vehicle_pts_offset.begin() + min_index, temp_vehicle_pts_offset.begin() + max_index);
    // 输出的点列
    vehicle_pts.reserve(vehicle_pts.size() + size);
    std::copy(temp_vehicle_pts.begin() + min_index, temp_vehicle_pts.begin() + max_index + 1,
              std::back_inserter(vehicle_pts));
    // vehicle_pts.insert(vehicle_pts.end(), temp_vehicle_pts.begin() + min_index, temp_vehicle_pts.begin() + max_index);
#if HDM_TEST_OUTPUT_DEBUG
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "output size: %d, outputpoint: ", vehicle_pts_offset.size());
    
    for (size_t i = min_index; i < max_index + 1; ++i) {
        HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "[%d:%d, %f, %f]", i, temp_vehicle_pts_offset[i], temp_vehicle_pts[i].x, temp_vehicle_pts[i].y);  
    }
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "\n");
    
    // vehicle 2 wgs84
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "output2wgs84:", vehicle_pts_offset.size());
    std::vector<zone::common::Point>  temp_wgs84_pts;
    uint32_t  temp_number = max_index - min_index + 1;
    temp_wgs84_pts.reserve(temp_number);
    temp_wgs84_pts.resize(temp_number);
    CHdmCoordsConverter::VehicleToWgs84(&temp_vehicle_pts[min_index], temp_number, heading_angle_,
                                 wgs84_vehicle_pos, &temp_wgs84_pts[0]);
    for (size_t i = 0; i < temp_wgs84_pts.size(); ++i) {
        zone::common::Point wgs84_point;
        CHdmCoordsConverter::Gcj02ToWgs84(temp_wgs84_pts[i].x, temp_wgs84_pts[i].y, wgs84_point.x, wgs84_point.y);
        HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "[%d: %f, %f]", i, wgs84_point.x, wgs84_point.y);  
    }
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "\n");
    // all vehicle 2 wgs84
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "ALL output2wgs84:", vehicle_pts_offset.size());
    std::vector<zone::common::Point>  all_wgs84_pts;
    all_wgs84_pts.reserve(vehicle_pts.size());
    all_wgs84_pts.resize(vehicle_pts.size());
    CHdmCoordsConverter::VehicleToWgs84(&vehicle_pts[0], vehicle_pts.size(), heading_angle_,
                                 wgs84_vehicle_pos, &all_wgs84_pts[0]);
    for (size_t i = 0; i < all_wgs84_pts.size(); ++i) {
        zone::common::Point wgs84_point;
        CHdmCoordsConverter::Gcj02ToWgs84(all_wgs84_pts[i].x, all_wgs84_pts[i].y, wgs84_point.x, wgs84_point.y);
        HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "[%d: %f, %f]", i, wgs84_point.x, wgs84_point.y);  
    }
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "\n");
#endif
    return true;
}



}