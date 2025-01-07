#include "hdm_utility/hdm_geometry.h"
#include "geographic_transform/geographic_transform.h"
#include "hdm_utility/hdm_dbg_log.h"
#include "geographic_transform/geographic_geometry.h"

using namespace std;
using namespace geographic_transform;

#define HDM_TEST_OUTPUT_DEBUG 0
#define FPRECISION 0.000001

namespace data_helper{
    // temp used here, to be refactored
    // Methods from map API which is not related to API, 
    // shall be splited into other class and used independent to the Map Interface
    const int LaneSeqStartNum = 1;  // zhonghaiting: lane seq start from 1
    const double cDeg2Rad = M_PI/180.0;
    const double c2PI = 2*M_PI;
    const float cUnitLL2Coord = 1e7;
    const float cUnitHt2Coord = 1e2;
    const float cUnitCoord2LL = 1e-7;
    const float cUnitCoord2Ht = 1e-2;
    
    template<class T>
    void RotFrame2D(const T x, const T y, const T ang_deg, T& outX, T& outY)
    {
        const T ang_rad = ang_deg * cDeg2Rad;
        const T sin_v = sin(ang_rad);
        const T cos_v = cos(ang_rad);
        outX = x * cos_v + y * sin_v;
        outY = - x * sin_v + y * cos_v; 
    }
}

namespace hdm_utility
{

CHdmGeometry::CHdmGeometry()
{    

}

CHdmGeometry::~CHdmGeometry()
{

}

bool CHdmGeometry::Wgs84ToGcj02(float64_t wgLon, float64_t wgLat,float64_t& mgLon,float64_t& mgLat)  
{
    GeoPoint wgs84(wgLat, wgLon, 0);
    GeoPoint gcj02;
    wgs84.Wgs84LatLon_2_Gcj02LatLon(wgs84, &gcj02);
    mgLon = gcj02.longitude();
    mgLat = gcj02.latitude();
    return true;
}

bool CHdmGeometry::Gcj02ToWgs84(float64_t lon, float64_t lat, float64_t& mgLon, float64_t& mgLat )
{
    GeoPoint gcj02(lat, lon);
    GeoPoint wgs84;
    gcj02.Gcj02LatLon_2_Wgs84LatLon(gcj02, &wgs84);
    mgLon = wgs84.longitude();
    mgLat = wgs84.latitude();
    return true;
}

float64_t CHdmGeometry::CalcAngle(const zone::common::Point &cur_point, const zone::common::Point &ahead_point)
{
  float64_t x = ahead_point.x- cur_point.x;
  float64_t y = ahead_point.y- cur_point.y;

  // 特殊角度
  if (abs(y) < FPRECISION)
  {
    if (x > FPRECISION)
    {
      return 90;
    }
    else if (x < FPRECISION)
    {
      return 270;
    }
    else // cur和ahead两个点重合
    {
      return 0;
    }
  }

  float64_t angle = atan(abs(x / y)) * 180 / LOC_MAP_PI;
  //printf("CHdmGeometry::CalcAngle angle[%f], Point[%f, %f]\n", angle, x, y);

  if ((x >= FPRECISION) && (y < FPRECISION))
  {
    angle = 180 - angle;
  }
  else if ((x < FPRECISION) && (y < FPRECISION))
  {
    angle = 180 + angle;
  }
  else if ((x < FPRECISION) && (y  > FPRECISION))
  {
    angle = 360 - angle;
  }
  return angle;
}

bool CHdmGeometry::IsPointInside(const zone::common::Point &p1, const zone::common::Point &p2, 
                                     const zone::common::Point &p)
{
  float64_t x_min = ZONE_MIN(p1.x, p2.x);
  float64_t x_max = ZONE_MAX(p1.x, p2.x);
  float64_t y_min = ZONE_MIN(p1.y, p2.y);
  float64_t y_max = ZONE_MAX(p1.y, p2.y);

  bool is_inside = false;

  if (p.x >= x_min && p.x <= x_max && p.y >= y_min && p.y <= y_max)
  {
    //printf("point in the poins \n" );
    is_inside = true;
  }

  return is_inside;
}

float64_t CHdmGeometry::CalcVehicleDistance(const zone::common::Point &p1, const zone::common::Point &p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

void CHdmGeometry::CalcProjectionPoint(const zone::common::Point &point1, const zone::common::Point &point2,
                    const zone::common::Point &p, zone::common::Point &foot_point, bool &is_inside)
{
  float64_t x1 = point1.x;
  float64_t y1 = point1.y;
  float64_t x2 = point2.x;
  float64_t y2 = point2.y;
  float64_t x0 = p.x;
  float64_t y0 = p.y;

  float64_t distance = 0.f;
  // printf("first rocoll pre-inside [%d]\n", is_inside);

  // 水平和垂直的特殊场景，不需要复杂计算
  if (abs(x1 - x2) < FPRECISION)
  {
    foot_point.x = x1;
    foot_point.y = y0;
    distance = abs(x0 - x1);

    is_inside = IsPointInside(point1, point2, foot_point);

    return;
  }
  else if (abs(y1 - y2)< FPRECISION)
  {
    foot_point.x = x0;
    foot_point.y = y1;
    distance = abs(y0 - y1);
    
    is_inside = IsPointInside(point1, point2, foot_point);


    return;
  }

  // 斜率
  float64_t k = (y2-y1) / (x2-x1);

  // ax+by+c=0 方程参数
  float64_t a = k;
  float64_t b = -1;
  float64_t c = y1 - k * x1;

  // 点p到线段p1p2的垂直距离
  distance = abs(a*x0 + b*y0 + c) / sqrt(a*a + b*b);

  // 垂足
  foot_point.x = (b*b*x0 - a*b*y0 - a*c) / (a*a + b*b);
  foot_point.y = (a*a*y0 - a*b*x0 - b*c) / (a*a + b*b);

  is_inside = IsPointInside(point1, point2, foot_point);
   //printf("first rocoll inside [%d]\n", is_inside);
}

bool CHdmGeometry::GetNearestPoint(const vector<zone::common::Point> &point_list, const zone::common::Point &p,
                          uint16_t &index, zone::common::Point &foot_point, float64_t &distance)
{
  bool find_inside = false;
  return GetNearestPoint(point_list, p, index, foot_point, distance, find_inside);  
}

bool CHdmGeometry::GetNearestPoint(const vector<zone::common::Point> &point_list, const zone::common::Point &p,
                          uint16_t &index, zone::common::Point &foot_point, float64_t &distance, bool &find_inside)
{
  distance = (float64_t)0xffffffff; 
  zone::common::Point temp_point;
  find_inside = false;
  // printf("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& [%.8f,  %.8f]\n", p.x, p.y );
  if (point_list.empty()) {
    return false;
  }

  for (int32_t i = 0; i < (int32_t)point_list.size() - 1; i++)
  {
    bool is_inside = false;
    // 遍历线上的每个线段，算出垂线最短的垂足 
    CalcProjectionPoint(point_list[i], point_list[i+1], p, temp_point, is_inside);
    // printf("start point [%.8f,  %.8f]\n", point_list[i].x, point_list[i].y );
    // printf("end point[%.8f,  %.8f]\n", point_list[i+1].x, point_list[i+1].y );
    // printf("in point [%.8f,  %.8f]\n", p.x, p.y );
    // printf("foot point[%.8f,  %.8f]\n", temp_point.x, temp_point.y );
    // printf("point[%d] in the poins (%d)\n", i, is_inside);
    if (is_inside)    {
      float64_t temp_dis = CalcWgs84Distance(temp_point, p);
      if (temp_dis < distance) {
        find_inside = true;
        distance = temp_dis;
        index = i;
        foot_point.x = temp_point.x;
        foot_point.y = temp_point.y;
        // printf("1111111222222222 infor [%.8f,  %.8f], distance[%0.8f]\n", foot_point.x, foot_point.y, distance );
      }
    }
    else {
      float64_t start_dis = CalcWgs84Distance(point_list[i], p);
      float64_t end_dis = CalcWgs84Distance(point_list[i + 1], p);

      if (start_dis < distance)
      {
        distance = start_dis;
        foot_point = point_list[i];
        index = i;
        if (0 != index) {
          find_inside = true;
        }
        else {
          find_inside = false;          
        }
      }
      
      if (end_dis < distance){
        distance = end_dis;
        foot_point = point_list[i + 1];
        index = i + 1;
        if (point_list.size() - 1 != index) {
          find_inside = true;
        }
        else {
          find_inside = false;
        }
      };       
      // printf("3333333333333 outfor [%.8f,  %.8f], distance[%0.8f]\n", foot_point.x, foot_point.y, distance );
    }    
  }
  return true;
  
}

void CHdmGeometry::CalculatePointByDistance(const vector<zone::common::Point> &point_list, float64_t distance,
  zone::common::Point &insert_point, uint16_t &insert_index)
{
  float64_t dis = 0;
  bool find_point = false;
  //printf("point_list.size()[%lu], distance[%f]\n", point_list.size(), distance);
  for (uint16_t i = 0; i < point_list.size()-1; ++i)
  {
    float64_t remain_dis = distance - dis;
    float64_t point_dis = CalcWgs84Distance(point_list.at(i), point_list.at(i+1));
    
    dis += point_dis;
    //printf("point_dis[%f]==>[%f]\n", point_dis, dis);
    if ( dis >= distance )
    {
      //则目标位置在第i个点和第i+1个点之间
      insert_index = i;
      
      insert_point.x = point_list.at(i).x + (point_list.at(i+1).x - point_list.at(i).x) * (remain_dis / point_dis);
      insert_point.y = point_list.at(i).y + (point_list.at(i+1).y - point_list.at(i).y) * (remain_dis / point_dis);
      //printf("################# i[%f, %f], i+1[%f, %f], insert_point[%f, %f]", point_list.at(i).x, point_list.at(i).x, (point_list.at(i+1)).x, (point_list.at(i+1)).y,
      //insert_point.x, insert_point.y);
      find_point = true;
      break;
    }
  }
  if (!find_point && !point_list.empty()) {
    insert_point = *point_list.rbegin();
  }
}

float64_t CHdmGeometry::CalcWgs84Distance(const zone::common::Point &p1, const zone::common::Point &p2)
{
    GeoPoint gp1(p1.y, p1.x, p1.z);
    GeoPoint gp2(p2.y, p2.x, p2.z);
    return gp1.distance(gp2);
}

bool CHdmGeometry::CalcWgs84LineDistance(const std::vector<zone::common::Point> &wgs84_pts,
                                         std::vector<float64_t>                 *distance_vec)
{
    if (nullptr == distance_vec) {
        return false;
    }
    uint32_t start_index = distance_vec->size();
    float64_t  sum_dis = 0.0;
    distance_vec->resize(wgs84_pts.size());
    uint32_t i = 1;
    if (0 == start_index){
      (*distance_vec)[0] = 0.0;
    }
    else{
      sum_dis = (*distance_vec)[start_index-1];
      i = start_index;
    }
    
    for (; i < wgs84_pts.size(); i++)
    {
      float64_t point_dis =  CHdmGeometry::CalcWgs84Distance(wgs84_pts[i - 1], wgs84_pts[i]);
      sum_dis += point_dis;
      (*distance_vec)[i] = sum_dis;
    };
    return true;
}

bool CHdmGeometry::CalcVehicleLineDistance(const std::vector<zone::common::Point>& vehicle_pts, std::vector<float64_t>* distance_vec)
{
    if (nullptr == distance_vec) {
        return false;
    }
    uint32_t start_index = distance_vec->size();
    float64_t  sum_dis = 0.0;
    distance_vec->resize(vehicle_pts.size());
    uint32_t i = 1;
    if (0 == start_index) {
        (*distance_vec)[0] = 0.0;
    } else {
        sum_dis = (*distance_vec)[start_index - 1];
        i = start_index;
    }

    for (; i < vehicle_pts.size(); i++) {
        float64_t point_dis = CHdmGeometry::CalcVehicleDistance(vehicle_pts[i - 1], vehicle_pts[i]);
        sum_dis += point_dis;
        (*distance_vec)[i] = sum_dis;
    };
    return true;
}


bool CHdmGeometry::Wgs84ToUtm(const std::vector<zone::common::Point>  &wgs84_point_list,
                              std::vector<zone::common::Point>        &utm_point_list)
{
    utm_point_list.resize(wgs84_point_list.size());
    for (size_t i = 0; i < wgs84_point_list.size(); ++i) {
        CHdmGeometry::Wgs84ToUtm(wgs84_point_list[i].x, wgs84_point_list[i].y, wgs84_point_list[i].z,
                                 utm_point_list[i].x, utm_point_list[i].y, utm_point_list[i].z);
    }
    return true;
}

bool CHdmGeometry::Wgs84ToUtm(const float64_t &lon, const float64_t& lat, const float64_t& 	height,
					float64_t&	dx, float64_t& dy, float64_t& dz)
{
    GeoPoint geo_transform;
    geo_transform.lla2utm(lat, lon, false, &dx, &dy);
    return true;    
}
    
bool CHdmGeometry::UtmToWgs84(const int32_t zone, const float64_t& dx, const float64_t&	dy, const float64_t&	dz,
                   float64_t& lon,  float64_t& lat, float64_t& height)
{
    GeoPoint geo_transform;
	  geo_transform.utm2lla(zone, false, dx, dy, &lat, &lon);
    return true;
}

bool CHdmGeometry::UtmToVehicle(const zone::common::Point  *utm_pts,
                                const uint32_t              pts_number,
                                const float64_t            &heading_angle,
                                const zone::common::Point  &utm_vehicle,
                                zone::common::Point        *vehicle_pts)
{
    // zone::common::Point utm_vehicle;
    zone::common::Point in_utm_point;

    // CHdmGeometry::Wgs84ToUtm(wgs84_vehicle.x, wgs84_vehicle.y, wgs84_vehicle.z, utm_vehicle.x, utm_vehicle.y, utm_vehicle.z);
    for (uint32_t i = 0; i < pts_number; ++i)    {
        CHdmGeometry::UtmToVehicle(utm_vehicle.x, utm_vehicle.y, utm_vehicle.z, heading_angle, 
                                   utm_pts[i].x, utm_pts[i].y, utm_pts[i].z,
                                   vehicle_pts[i].x, vehicle_pts[i].y, vehicle_pts[i].z);
    };
    return true;
}

bool CHdmGeometry::UtmToVehicle(const float64_t &raw_x_utm, const float64_t &raw_y_utm, const float64_t &raw_z_utm, 
        const float64_t &heading_angle, const float64_t &in_x_utm, const float64_t &in_y_utm, const float64_t &in_z_utm, 
                    float64_t &out_x_vehicle, float64_t &out_y_vehicle, float64_t &out_z_vehicle)
{
    GeoPoint geo_transform;
    geo_transform.utm2vehicle(raw_x_utm, raw_y_utm, raw_z_utm, heading_angle,
					         in_x_utm, in_y_utm, in_z_utm, out_x_vehicle, out_y_vehicle, out_z_vehicle);
    return true;   
}

bool CHdmGeometry::VehicleToUtm(const float64_t&	raw_x_utm, const float64_t& 	raw_y_utm,  const float64_t& 	raw_z_utm, 
					 const float64_t& 	heading_angle, const float64_t&	x_vehicle,  const float64_t& 	y_vehicle,  const float64_t& 	z_vehicle, 
					 float64_t& 		x_utm,  float64_t& 		y_utm,  float64_t& 		z_utm )
{
    GeoPoint geo_transform;
    geo_transform.vehicle2utm(raw_x_utm, raw_y_utm, raw_z_utm, heading_angle,
					         x_vehicle, y_vehicle, z_vehicle, x_utm, y_utm, z_utm);
    return true;  
}

bool CHdmGeometry::Wgs84ToVehicle(const zone::common::Point  *wgs84_pts,
                        const uint32_t              pts_number,
                        const float64_t               &heading_angle,
                        const zone::common::Point  &wgs84_vehicle,
                        zone::common::Point        *vehicle_pts)
{
    zone::common::Point utm_vehicle;
    zone::common::Point in_utm_point;

    CHdmGeometry::Wgs84ToUtm(wgs84_vehicle.x, wgs84_vehicle.y, wgs84_vehicle.z, utm_vehicle.x, utm_vehicle.y, utm_vehicle.z);
    for (uint32_t i = 0; i <  pts_number; ++i)    {
        CHdmGeometry::Wgs84ToUtm(wgs84_pts[i].x, wgs84_pts[i].y, wgs84_pts[i].z, in_utm_point.x, in_utm_point.y, in_utm_point.z);
        CHdmGeometry::UtmToVehicle(utm_vehicle.x, utm_vehicle.y, utm_vehicle.z, heading_angle, 
            in_utm_point.x, in_utm_point.y, in_utm_point.z, vehicle_pts[i].x, vehicle_pts[i].y, vehicle_pts[i].z);
    };
    return true;
}

bool CHdmGeometry::Wgs84ToVehicle3(const zone::common::Point  *wgs84_pts,
                        const uint32_t              pts_number,
                        const float64_t               &heading_angle,
                        const zone::common::Point  &wgs84_vehicle,
                        zone::common::Point        *vehicle_pts)
{

  for (uint32_t i = 0; i <  pts_number; ++i){
      double map_point_x, map_point_y;
      geographic_transform::GeoPoint map_line_point(wgs84_pts[i].y , wgs84_pts[i].x );
      geographic_transform::GeoPoint vehicle(wgs84_vehicle.y, wgs84_vehicle.x );

      double temp_map_heading = 0.0;
      temp_map_heading = heading_angle <= 90 ? 90 - heading_angle : 360 - (heading_angle - 90); 

      vehicle.offsetOnGeoid(map_line_point, map_point_x, map_point_y);

      data_helper::RotFrame2D(map_point_x, map_point_y, temp_map_heading, map_point_x, map_point_y);

      vehicle_pts[i].x = map_point_x;
      vehicle_pts[i].y = map_point_y;
  };
  return true;
}

bool CHdmGeometry::Wgs84ToVehicleSigle(const zone::common::Point  &wgs84_pts,
                        const float64_t               &heading_angle,
                        const zone::common::Point  &wgs84_vehicle,
                        zone::common::Point        *vehicle_pts)
{
      double map_point_x, map_point_y;
      geographic_transform::GeoPoint map_line_point(wgs84_pts.y , wgs84_pts.x);
      geographic_transform::GeoPoint vehicle(wgs84_vehicle.y, wgs84_vehicle.x);

      double temp_map_heading = 0.0;
      temp_map_heading = heading_angle <= 90 ? 90 - heading_angle : 360 - (heading_angle - 90); 

      vehicle.offsetOnGeoid(map_line_point, map_point_x, map_point_y);

      data_helper::RotFrame2D(map_point_x, map_point_y, temp_map_heading, map_point_x, map_point_y);

      vehicle_pts->x = map_point_x;
      vehicle_pts->y = map_point_y;
  return true;
}


bool CHdmGeometry::VehicleToWgs84(const zone::common::Point  *vehicle_pts, 
                        const uint32_t              pts_number,
                        const float64_t                heading_angle,
                        const zone::common::Point  &wgs84_vehicle,  // 自车当前wgs84坐标
					              zone::common::Point        *wgs84_pts)
{
    zone::common::Point utm_vehicle;
    zone::common::Point out_utm_point;

    CHdmGeometry::Wgs84ToUtm(wgs84_vehicle.x, wgs84_vehicle.y, wgs84_vehicle.z, utm_vehicle.x, utm_vehicle.y, utm_vehicle.z);
    for (uint32_t i = 0; i <  pts_number; ++i)    {
        CHdmGeometry::VehicleToUtm(utm_vehicle.x, utm_vehicle.y, utm_vehicle.z, heading_angle, 
              vehicle_pts[i].x, vehicle_pts[i].y, vehicle_pts[i].z, out_utm_point.x, out_utm_point.y, out_utm_point.z);
        CHdmGeometry::UtmToWgs84((int32_t(wgs84_vehicle.x / 6) + 31), out_utm_point.x, out_utm_point.y, out_utm_point.z, wgs84_pts[i].x, wgs84_pts[i].y, wgs84_pts[i].z);    
    };
    return true;
}

bool CHdmGeometry::VehicleToWgs84v2(const zone::common::Point& vehicle_pts,
                                    float64_t                  heading_angle,//unit:degree
                                    const zone::common::Point& wgs84_vehicle,
                                    zone::common::Point&       out_wgs84_pts)
{
    double eastOffset = 0.0, northOffset = 0.0;

    double temp_map_heading = heading_angle <= 90 ? 90 - heading_angle : 360 - (heading_angle - 90); 

    // trans from vehicle frame to E-N frame, E-N frame can be rotated from vehicle frame by -1 * heading
    data_helper::RotFrame2D<double>(vehicle_pts.x, vehicle_pts.y, -temp_map_heading, eastOffset, northOffset);
    
    geographic_transform::GeoPoint out_point(wgs84_vehicle.y , wgs84_vehicle.x, wgs84_vehicle.z);
    out_point.moveOnGeoid(eastOffset, northOffset);
    out_wgs84_pts.x = out_point.longitude();
    out_wgs84_pts.y = out_point.latitude();
    out_wgs84_pts.z = out_point.elevation();
    return true;
}

bool CHdmGeometry::VehicleToWgs84v2(const zone::common::Point* vehicle_pts,
                                    uint32_t                   pts_number,
                                    float64_t                  heading_angle,//unit:degree
                                    const zone::common::Point& wgs84_vehicle,
                                    zone::common::Point        *wgs84_pts)
{
    if(nullptr == vehicle_pts || nullptr == wgs84_pts || pts_number == 0)
        return false;
    float eastOffset = 0.0, northOffset = 0.0;

    double temp_map_heading = heading_angle <= 90 ? 90 - heading_angle : 360 - (heading_angle - 90); 

    for (uint32_t i = 0; i <  pts_number; ++i)
    {
        data_helper::RotFrame2D<float>(vehicle_pts[i].x, vehicle_pts[i].y, -temp_map_heading, eastOffset, northOffset);
        geographic_transform::GeoPoint out_point(wgs84_vehicle.y , wgs84_vehicle.x,wgs84_vehicle.z);
        out_point.moveOnGeoid(eastOffset, northOffset);
        wgs84_pts[i].x = out_point.longitude();
        wgs84_pts[i].y = out_point.latitude();
        wgs84_pts[i].z = out_point.elevation();
    }
    return true;
}

// 用原始的wgs84坐标点列计算出带offset的自车坐标系点列
bool CHdmGeometry::CaclVehiclePointsAndOffsets(const int32_t  min_offset_range,   // 最小范围offset，单位厘米，规控要求自车后50m,（-5000）
                                               const int32_t  max_offset_range,   // 最大范围offset，单位厘米，规控要求自车前100m,（10000）
                                               const int32_t  start_offset,       // 输入wgs84_pts起点相对于自车位置偏移，单位厘米
                                               const int32_t  end_offset,         // 输入wgs84_pts终点相对于自车位置偏移，单位厘米
                                               const zone::common::Point  &wgs84_vehicle_pos,   // 自车当前wgs84坐标
                                               const float64_t                heading_angle,       // 自车航向角度, 单位度，以正北方向为0度，顺时钟，[0,360)
                                               const zone::common::Point  *wgs84_pts,           // 要转换的原始wgs84点列
                                               const uint32_t              pts_number,          // 要转换的原始wgs84点数目
                                               std::vector<int32_t>       &vehicle_pts_offset,  // 输出点列的offset，相对自车位置
                                               std::vector<zone::common::Point> &vehicle_pts)   // 输出点列，和vehicle_pts_offset一一对应
{   
#if HDM_TEST_OUTPUT_DEBUG
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "CHdmGeometry::CaclVehiclePointsAndOffsets, pts_number: %d.\n", pts_number);
    
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, 
              "input  pts_number: %d, min_offset_range: %d, max_offset_range: %d, start_offset: %d, end_offset: %d, heading_angle: %f, vehicle_pts_offset_size[%u],",
              pts_number, min_offset_range, max_offset_range, start_offset, end_offset, heading_angle, vehicle_pts_offset.size());
    zone::common::Point wgs84_point;
         CHdmGeometry::Gcj02ToWgs84(wgs84_vehicle_pos.x, wgs84_vehicle_pos.y, wgs84_point.x, wgs84_point.y);
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, 
              "wgs84_vehicle_pos[x:%f, y:%f]\n", wgs84_point.x, wgs84_point.y); 
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "inputpoint:");
    for (uint32_t i = 0; i < pts_number; ++i) {
         zone::common::Point wgs84_point;
         CHdmGeometry::Gcj02ToWgs84(wgs84_pts[i].x, wgs84_pts[i].y, wgs84_point.x, wgs84_point.y);
         HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "[%d: %f, %f]", i, wgs84_point.x, wgs84_point.y);    
    }
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "\n");
#endif

    if (nullptr == wgs84_pts) {
        HDM_ERROR("CHdmGeometry::CaclVehiclePointsAndOffsets","wgs84_pts is nullptr.\n");
        return false;
    }

    if (pts_number <= 0) {
        HDM_ERROR("CHdmGeometry::CaclVehiclePointsAndOffsets", "pts_number <= 0.\n");
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
    CHdmGeometry::Wgs84ToVehicle(wgs84_pts, pts_number, heading_angle, wgs84_vehicle_pos,
                                 &temp_vehicle_pts[0]);

    /// 求出总长度
    float64_t               sum_dis      = 0;
    float64_t               delta_offset = end_offset - start_offset;
    std::vector<float64_t>  distance_vec;
    distance_vec.reserve(pts_number);
    distance_vec.push_back(0);
    for (uint32_t i = 1; i < pts_number; i++)
    {
        float64_t point_dis =  CHdmGeometry::CalcWgs84Distance(wgs84_pts[i - 1], wgs84_pts[i]);
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
    CHdmGeometry::VehicleToWgs84(&temp_vehicle_pts[min_index], temp_number, heading_angle,
                                 wgs84_vehicle_pos, &temp_wgs84_pts[0]);
    for (size_t i = 0; i < temp_wgs84_pts.size(); ++i) {
        zone::common::Point wgs84_point;
        CHdmGeometry::Gcj02ToWgs84(temp_wgs84_pts[i].x, temp_wgs84_pts[i].y, wgs84_point.x, wgs84_point.y);
        HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "[%d: %f, %f]", i, wgs84_point.x, wgs84_point.y);  
    }
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "\n");
    // all vehicle 2 wgs84
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "ALL output2wgs84:", vehicle_pts_offset.size());
    std::vector<zone::common::Point>  all_wgs84_pts;
    all_wgs84_pts.reserve(vehicle_pts.size());
    all_wgs84_pts.resize(vehicle_pts.size());
    CHdmGeometry::VehicleToWgs84(&vehicle_pts[0], vehicle_pts.size(), heading_angle,
                                 wgs84_vehicle_pos, &all_wgs84_pts[0]);
    for (size_t i = 0; i < all_wgs84_pts.size(); ++i) {
        zone::common::Point wgs84_point;
        CHdmGeometry::Gcj02ToWgs84(all_wgs84_pts[i].x, all_wgs84_pts[i].y, wgs84_point.x, wgs84_point.y);
        HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "[%d: %f, %f]", i, wgs84_point.x, wgs84_point.y);  
    }
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "\n");
#endif
    return true;
}

// 用原始的utm坐标点列计算出带offset的自车坐标系点列
bool CHdmGeometry::CaclVehiclePointsAndOffsets2(const int32_t  min_offset_range,   // 最小范围offset，单位厘米，规控要求自车后50m,（-5000）
                                                const int32_t  max_offset_range,   // 最大范围offset，单位厘米，规控要求自车前100m,（10000）
                                                const int32_t  start_offset,       // 输入wgs84_pts起点相对于自车位置偏移，单位厘米
                                                const int32_t  end_offset,         // 输入wgs84_pts终点相对于自车位置偏移，单位厘米
                                                const zone::common::Point   &wgs84_vehicle_pos,   // 自车当前wgs84坐标
                                                const float64_t              heading_angle,       // 自车航向角度, 单位度，以正北方向为0度，顺时钟，[0,360)
                                                const zone::common::Point   *wgs84_pts,           // 要转换的原始wgs84点列
                                                const uint32_t               pts_number,          // 要转换的原始wgs84点数目
                                                std::vector<int32_t>        &vehicle_pts_offset,  // 输出点列的offset，相对自车位置
                                                std::vector<zone::common::Point> &vehicle_pts,    // 输出点列，和vehicle_pts_offset一一对应
                                                int32_t                     &start_index,         // 输出点列的起点对应输入点列wgs84_pts中的索引
                                                int32_t                     &end_index,           // 输出点列的终点对应输入点列wgs84_pts中的索引
                                                const std::vector<float64_t> &distance_vec)
{   
#if HDM_TEST_OUTPUT_DEBUG
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "CHdmGeometry::CaclVehiclePointsAndOffsets, pts_number: %d.\n", pts_number);
    
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, 
              "input  pts_number: %d, min_offset_range: %d, max_offset_range: %d, start_offset: %d, end_offset: %d, heading_angle: %f, vehicle_pts_offset_size[%u],",
              pts_number, min_offset_range, max_offset_range, start_offset, end_offset, heading_angle, vehicle_pts_offset.size());
    zone::common::Point wgs84_point;
         CHdmGeometry::Gcj02ToWgs84(wgs84_vehicle_pos.x, wgs84_vehicle_pos.y, wgs84_point.x, wgs84_point.y);
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, 
              "wgs84_vehicle_pos[x:%f, y:%f]\n", wgs84_point.x, wgs84_point.y); 
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "inputpoint:");
    for (uint32_t i = 0; i < pts_number; ++i) {
         zone::common::Point wgs84_point;
         CHdmGeometry::Gcj02ToWgs84(wgs84_pts[i].x, wgs84_pts[i].y, wgs84_point.x, wgs84_point.y);
         HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "[%d: %f, %f]", i, wgs84_point.x, wgs84_point.y);    
    }
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "\n");
#endif

    if (nullptr == wgs84_pts) {
        HDM_ERROR("CHdmGeometry::CaclVehiclePointsAndOffsets","wgs84_pts or utm_pts is nullptr.\n");
        return false;
    }

    if (pts_number <= 0) {
        HDM_ERROR("CHdmGeometry::CaclVehiclePointsAndOffsets", "pts_number <= 0.\n");
        return false;
    }
    
    if (start_offset >= end_offset) {
#if HDM_TEST_OUTPUT_DEBUG
        HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true,
                  "start_offset >= end_offset: start_offset:%d, end_offset:%d\n", start_offset, end_offset);
#endif
        return false;
    }

    if (distance_vec.size() != pts_number) {
      HDM_ERROR("CHdmGeometry::CaclVehiclePointsAndOffsets", "distance_vec size:%lu, pts_number:%d\n", distance_vec.size(), pts_number);
      return false;
    }

    std::vector<int32_t>               temp_vehicle_pts_offset;
    std::vector<zone::common::Point>   temp_vehicle_pts;
    temp_vehicle_pts_offset.reserve(pts_number);
    temp_vehicle_pts.resize(pts_number);

    /// 求出总长度
    float64_t               sum_dis      = 0;
    float64_t               delta_offset = end_offset - start_offset;
    // 计算offset换算比例
    sum_dis = distance_vec.back();
    float64_t percent = delta_offset / sum_dis;
    // 计算输出点列的offset
    for (const float64_t &distance : distance_vec) {
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

    uint32_t point_num = max_index - min_index;

    // wgs84转换车身坐标系
    for (int32_t i = min_index; i <= max_index; i++) {
      CHdmGeometry::Wgs84ToVehicleSigle(wgs84_pts[i], heading_angle, wgs84_vehicle_pos, &(temp_vehicle_pts[i]));
    }

    // 更新输出点列的offset和输出的点列
    size_t size = max_index - min_index + 1;
    vehicle_pts_offset.reserve(vehicle_pts_offset.size() + size);
    std::copy(temp_vehicle_pts_offset.begin() + min_index, temp_vehicle_pts_offset.begin() + max_index + 1,
              std::back_inserter(vehicle_pts_offset));
    // 输出的点列
    vehicle_pts.reserve(vehicle_pts.size() + size);
    std::copy(temp_vehicle_pts.begin() + min_index, temp_vehicle_pts.begin() + max_index + 1,
              std::back_inserter(vehicle_pts));
    start_index = min_index;
    end_index = max_index;

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
    CHdmGeometry::VehicleToWgs84(&temp_vehicle_pts[min_index], temp_number, heading_angle,
                                 wgs84_vehicle_pos, &temp_wgs84_pts[0]);
    for (size_t i = 0; i < temp_wgs84_pts.size(); ++i) {
        zone::common::Point wgs84_point;
        CHdmGeometry::Gcj02ToWgs84(temp_wgs84_pts[i].x, temp_wgs84_pts[i].y, wgs84_point.x, wgs84_point.y);
        HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "[%d: %f, %f]", i, wgs84_point.x, wgs84_point.y);  
    }
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "\n");
    // all vehicle 2 wgs84
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "ALL output2wgs84:", vehicle_pts_offset.size());
    std::vector<zone::common::Point>  all_wgs84_pts;
    all_wgs84_pts.reserve(vehicle_pts.size());
    all_wgs84_pts.resize(vehicle_pts.size());
    CHdmGeometry::VehicleToWgs84(&vehicle_pts[0], vehicle_pts.size(), heading_angle,
                                 wgs84_vehicle_pos, &all_wgs84_pts[0]);
    for (size_t i = 0; i < all_wgs84_pts.size(); ++i) {
        zone::common::Point wgs84_point;
        CHdmGeometry::Gcj02ToWgs84(all_wgs84_pts[i].x, all_wgs84_pts[i].y, wgs84_point.x, wgs84_point.y);
        HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "[%d: %f, %f]", i, wgs84_point.x, wgs84_point.y);  
    }
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "\n");
#endif
    return true;
}


bool CHdmGeometry::CaclVehiclePointsAndOffsets3(const int32_t  min_offset_range,   // 最小范围offset，单位厘米，规控要求自车后50m,（-5000）
                                                const int32_t  max_offset_range,   // 最大CaclVehiclePointsAndOffsets3CaclVehiclePointsAndOffsets3CaclVehiclePointsAndOffsets3范围offset，单位厘米，规控要求自车前100m,（10000）
                                                const int32_t  start_offset,       // 输入wgs84_pts起点相对于自车位置偏移，单位厘米
                                                const int32_t  end_offset,         // 输入wgs84_pts终点相对于自车位置偏移，单位厘米
                                                const zone::common::Point   &wgs84_vehicle_pos,   // 自车当前wgs84坐标
                                                std::vector<zone::common::Point>  &point_xy_list, // WGS84形状点相对自车的投影
                                                int32_t                      &point_xy_min_index, // pre min_index
                                                int32_t                      &point_xy_max_index, // pre max_index                                         
                                                const float64_t              heading_angle,       // 自车航向角度, 单位度，以正北方向为0度，顺时钟，[0,360)
                                                const zone::common::Point   *wgs84_pts,           // 要转换的原始wgs84点列
                                                const uint32_t               pts_number,          // 要转换的原始wgs84点数目
                                                std::vector<int32_t>        &vehicle_pts_offset,  // 输出点列的offset，相对自车位置
                                                std::vector<zone::common::Point> &vehicle_pts,    // 输出点列，和vehicle_pts_offset一一对应
                                                const std::vector<float64_t>     &distance_vec)   
{   
#if HDM_TEST_OUTPUT_DEBUG
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "CHdmGeometry::CaclVehiclePointsAndOffsets, pts_number: %d.\n", pts_number);
    
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, 
              "input  pts_number: %d, min_offset_range: %d, max_offset_range: %d, start_offset: %d, end_offset: %d, heading_angle: %f, vehicle_pts_offset_size[%u],",
              pts_number, min_offset_range, max_offset_range, start_offset, end_offset, heading_angle, vehicle_pts_offset.size());
    zone::common::Point wgs84_point;
         CHdmGeometry::Gcj02ToWgs84(wgs84_vehicle_pos.x, wgs84_vehicle_pos.y, wgs84_point.x, wgs84_point.y);
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, 
              "wgs84_vehicle_pos[x:%f, y:%f]\n", wgs84_point.x, wgs84_point.y); 
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "inputpoint:");
    for (uint32_t i = 0; i < pts_number; ++i) {
         zone::common::Point wgs84_point;
         CHdmGeometry::Gcj02ToWgs84(wgs84_pts[i].x, wgs84_pts[i].y, wgs84_point.x, wgs84_point.y);
         HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "[%d: %f, %f]", i, wgs84_point.x, wgs84_point.y);    
    }
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "\n");
#endif

    if (nullptr == wgs84_pts) {
        HDM_ERROR("CHdmGeometry::CaclVehiclePointsAndOffsets","wgs84_pts or utm_pts is nullptr.\n");
        return false;
    }

    if (pts_number <= 0) {
        HDM_ERROR("CHdmGeometry::CaclVehiclePointsAndOffsets", "pts_number <= 0.\n");
        return false;
    }
    
    if (start_offset >= end_offset) {
#if HDM_TEST_OUTPUT_DEBUG
        HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true,
                  "start_offset >= end_offset: start_offset:%d, end_offset:%d\n", start_offset, end_offset);
#endif
        return false;
    }

    if (distance_vec.size() != pts_number) {
      HDM_ERROR("CHdmGeometry::CaclVehiclePointsAndOffsets", "distance_vec size:%lu, pts_number:%d\n", distance_vec.size(), pts_number);
      return false;
    }

    std::vector<int32_t>               temp_vehicle_pts_offset;
    std::vector<zone::common::Point>   temp_vehicle_pts;
    temp_vehicle_pts_offset.reserve(pts_number);
    temp_vehicle_pts.resize(pts_number);

    /// 求出总长度
    float64_t               sum_dis      = 0;
    float64_t               delta_offset = end_offset - start_offset;
    // 计算offset换算比例
    sum_dis = distance_vec.back();
    float64_t percent = delta_offset / sum_dis;
    // 计算输出点列的offset
    for (const float64_t &distance : distance_vec) {
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

    uint32_t point_num = max_index - min_index;

    geographic_transform::GeoPoint vehicle(wgs84_vehicle_pos.y, wgs84_vehicle_pos.x);

    if (point_xy_max_index == 0 && point_xy_list.size() == 0)
    {
        point_xy_list.resize(pts_number);
 
        // wgs84转换车身坐标系
        for (int32_t i = min_index; i <= max_index; i++) {

          double map_point_x, map_point_y;

          geographic_transform::GeoPoint map_line_point(wgs84_pts[i].y , wgs84_pts[i].x);

          vehicle.offsetOnGeoid(map_line_point, map_point_x, map_point_y);

          point_xy_list[i].x = map_point_x;
          point_xy_list[i].y = map_point_y;          
 
          double temp_map_heading = heading_angle <= 90 ? 90 - heading_angle : 360 - (heading_angle - 90); 

          data_helper::RotFrame2D(map_point_x, map_point_y, temp_map_heading, map_point_x, map_point_y);

          temp_vehicle_pts[i].x = map_point_x;
          temp_vehicle_pts[i].y = map_point_y;
         }

       point_xy_min_index = min_index;
       point_xy_max_index = max_index;
    }
    else {

 
        if (point_xy_list.size() != pts_number) {
            point_xy_list.resize(pts_number);
        }
 
        double map_point_x, map_point_y;
        geographic_transform::GeoPoint map_line_point(wgs84_pts[min_index].y , wgs84_pts[min_index].x);
        vehicle.offsetOnGeoid(map_line_point, map_point_x, map_point_y);
        for (int32_t i = (min_index + 1); i <= point_xy_max_index; i++) {
 
            point_xy_list[i].x = map_point_x +  (point_xy_list[i].x - point_xy_list[min_index].x);
            point_xy_list[i].y = map_point_y +  (point_xy_list[i].y - point_xy_list[min_index].y);
        }
 
        point_xy_list[min_index].x = map_point_x;
        point_xy_list[min_index].y = map_point_y;        

        double temp_map_heading = heading_angle <= 90 ? 90 - heading_angle : 360 - (heading_angle - 90); 

        for (int32_t i = min_index; i <= point_xy_max_index; i++) { 
 
            data_helper::RotFrame2D(point_xy_list[i].x, point_xy_list[i].y, temp_map_heading, map_point_x, map_point_y);

            temp_vehicle_pts[i].x = map_point_x;
            temp_vehicle_pts[i].y = map_point_y;
        }


        if (max_index > point_xy_max_index) {

          for (int32_t i = (point_xy_max_index + 1); i <= max_index; ++i) {
            double map_point_x, map_point_y;

            geographic_transform::GeoPoint map_line_point(wgs84_pts[i].y , wgs84_pts[i].x);

            vehicle.offsetOnGeoid(map_line_point, map_point_x, map_point_y);
 
            point_xy_list[i].x = map_point_x;
            point_xy_list[i].y = map_point_y;          
   
 
            data_helper::RotFrame2D(map_point_x, map_point_y, temp_map_heading, map_point_x, map_point_y);

            temp_vehicle_pts[i].x = map_point_x;
            temp_vehicle_pts[i].y = map_point_y;
          }

        }

       point_xy_min_index = min_index;
       point_xy_max_index = max_index;

 
    }

    // 更新输出点列的offset和输出的点列
    size_t size = max_index - min_index + 1;
    vehicle_pts_offset.reserve(vehicle_pts_offset.size() + size);
    std::copy(temp_vehicle_pts_offset.begin() + min_index, temp_vehicle_pts_offset.begin() + max_index + 1,
              std::back_inserter(vehicle_pts_offset));
    // 输出的点列
    vehicle_pts.reserve(vehicle_pts.size() + size);
    std::copy(temp_vehicle_pts.begin() + min_index, temp_vehicle_pts.begin() + max_index + 1,
              std::back_inserter(vehicle_pts));

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
    CHdmGeometry::VehicleToWgs84(&temp_vehicle_pts[min_index], temp_number, heading_angle,
                                 wgs84_vehicle_pos, &temp_wgs84_pts[0]);
    for (size_t i = 0; i < temp_wgs84_pts.size(); ++i) {
        zone::common::Point wgs84_point;
        CHdmGeometry::Gcj02ToWgs84(temp_wgs84_pts[i].x, temp_wgs84_pts[i].y, wgs84_point.x, wgs84_point.y);
        HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "[%d: %f, %f]", i, wgs84_point.x, wgs84_point.y);  
    }
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "\n");
    // all vehicle 2 wgs84
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "ALL output2wgs84:", vehicle_pts_offset.size());
    std::vector<zone::common::Point>  all_wgs84_pts;
    all_wgs84_pts.reserve(vehicle_pts.size());
    all_wgs84_pts.resize(vehicle_pts.size());
    CHdmGeometry::VehicleToWgs84(&vehicle_pts[0], vehicle_pts.size(), heading_angle,
                                 wgs84_vehicle_pos, &all_wgs84_pts[0]);
    for (size_t i = 0; i < all_wgs84_pts.size(); ++i) {
        zone::common::Point wgs84_point;
        CHdmGeometry::Gcj02ToWgs84(all_wgs84_pts[i].x, all_wgs84_pts[i].y, wgs84_point.x, wgs84_point.y);
        HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "[%d: %f, %f]", i, wgs84_point.x, wgs84_point.y);  
    }
    HDM_WRITE("CaclVehiclePointsAndOffsets.txt", true, "\n");
#endif
    return true;
}                                               

// 用原始的wgs84坐标点列计算出带offset的平面直角坐标系（以自车为原点，不考虑自车航向角）点列,参考定位方法
bool CHdmGeometry::CaclCartesianPointsAndOffsets(const int32_t  min_offset_range,   // 最小范围offset，单位厘米，规控要求自车后50m,（-10000）
                                          const int32_t  max_offset_range,   // 最大范围offset，单位厘米，规控要求自车前100m,（20000）
                                          const int32_t  start_offset,       // 输入wgs84_pts起点相对于自车位置偏移，单位厘米
                                          const int32_t  end_offset,         // 输入wgs84_pts终点相对于自车位置偏移，单位厘米
                                          const zone::common::Point& wgs84_vehicle_pos,    // 自车当前wgs84坐标
                                          const zone::common::Point* wgs84_pts,            // 要转换的原始wgs84点列
                                          const uint32_t               pts_number,         // 要转换的原始wgs84点数目
                                          std::vector<int32_t>& vehicle_pts_offset,        // 输出点列的offset，相对自车位置
                                          std::vector<zone::common::Point>& vehicle_pts,   // 输出点列，和vehicle_pts_offset一一对应
                                          const std::vector<float64_t>& distance_vec)      // 输入点列的间距，单位厘米
{
    if (nullptr == wgs84_pts) {
        HDM_ERROR("CHdmGeometry::CaclVehiclePointsAndOffsets", "wgs84_pts or utm_pts is nullptr.\n");
        return false;
    }

    if (pts_number <= 0) {
        HDM_ERROR("CHdmGeometry::CaclVehiclePointsAndOffsets", "pts_number <= 0.\n");
        return false;
    }

    if (start_offset >= end_offset) {
        HDM_ERROR("CHdmGeometry::CaclVehiclePointsAndOffsets", "start_offset >= end_offset: start_offset:%d, end_offset:%d\n", start_offset, end_offset);
        return false;
    }

    if (distance_vec.size() != pts_number) {
        HDM_ERROR("CHdmGeometry::CaclVehiclePointsAndOffsets", "distance_vec size:%lu, pts_number:%d\n", distance_vec.size(), pts_number);
        return false;
    }

    std::vector<int32_t>               temp_vehicle_pts_offset;
    std::vector<zone::common::Point>   temp_vehicle_pts;
    temp_vehicle_pts_offset.reserve(pts_number);
    temp_vehicle_pts.resize(pts_number);

    /// 求出总长度
    float64_t               sum_dis = 0;
    float64_t               delta_offset = end_offset - start_offset;
    // 计算offset换算比例
    sum_dis = distance_vec.back();
    float64_t percent = delta_offset / sum_dis;
    // 计算输出点列的offset
    for (const float64_t& distance : distance_vec) {
        int32_t offset = start_offset + distance * percent;
        temp_vehicle_pts_offset.push_back(offset);
    }
    /// 不在范围内
    if (temp_vehicle_pts_offset[0] > max_offset_range) {  // 在range的前方
#if HDM_TEST_OUTPUT_DEBUG
        HDM_WRITE("CaclCartesianPointsAndOffsets.txt", true,
            "OutRange: temp_vehicle_pts_offset[0]:%d, max_offset_range:%d\n", temp_vehicle_pts_offset[0], max_offset_range);
        HDM_WRITE("CaclCartesianPointsAndOffsets.txt", true, "offset: ");
        for (int i = 0; i < temp_vehicle_pts_offset.size(); ++i) {
            HDM_WRITE("CaclCartesianPointsAndOffsets.txt", true, "[%d: %d]", i, temp_vehicle_pts_offset[i]);
}
        HDM_WRITE("CaclCartesianPointsAndOffsets.txt", true, "\n");
#endif
        return true;
}
    if (temp_vehicle_pts_offset.back() < min_offset_range) {  // 在range的后方
#if HDM_TEST_OUTPUT_DEBUG
        HDM_WRITE("CaclCartesianPointsAndOffsets.txt", true,
            "OutRange: temp_vehicle_pts_offset[-1]:%d, min_offset_range:%d\n", temp_vehicle_pts_offset.back(), min_offset_range);
#endif
        return true;
    }
    int32_t min_index = 0;
    int32_t max_index = pts_number - 1;
    // 在范围内
    for (int32_t i = 0; i < static_cast<int32_t>(pts_number - 1); i++) {
        if ((temp_vehicle_pts_offset[i] <= min_offset_range) &&
            (min_offset_range < temp_vehicle_pts_offset[i + 1])) {
            min_index = i;
        } else if ((temp_vehicle_pts_offset[i] < max_offset_range) &&
            (max_offset_range <= temp_vehicle_pts_offset[i + 1])) {
            max_index = i + 1;
            break;
        } else {
            continue;
        }
    }

    // wgs84转换成以自车为原点的平面直角坐标系
    for (int32_t i = min_index; i <= max_index; i++) {
        geographic_transform::GeoPoint map_line_point(wgs84_pts[i].y, wgs84_pts[i].x);
        geographic_transform::GeoPoint vehicle(wgs84_vehicle_pos.y, wgs84_vehicle_pos.x);
        vehicle.offsetOnGeoid(map_line_point, temp_vehicle_pts[i].x, temp_vehicle_pts[i].y);
    }

    // 更新输出点列的offset和输出的点列
    size_t size = max_index - min_index + 1;
    vehicle_pts_offset.reserve(vehicle_pts_offset.size() + size);
    std::copy(temp_vehicle_pts_offset.begin() + min_index, temp_vehicle_pts_offset.begin() + max_index + 1,
        std::back_inserter(vehicle_pts_offset));
    // 输出的点列
    vehicle_pts.reserve(vehicle_pts.size() + size);
    std::copy(temp_vehicle_pts.begin() + min_index, temp_vehicle_pts.begin() + max_index + 1,
        std::back_inserter(vehicle_pts));

#if HDM_TEST_OUTPUT_DEBUG
    HDM_WRITE("CaclCartesianPointsAndOffsets.txt", true, "output size: %d, outputpoint: ", vehicle_pts_offset.size());
    for (size_t i = min_index; i < max_index + 1; ++i) {
        HDM_WRITE("CaclCartesianPointsAndOffsets.txt", true, "[%d:%d, %f, %f]", i, temp_vehicle_pts_offset[i], temp_vehicle_pts[i].x, temp_vehicle_pts[i].y);
    }
    HDM_WRITE("CaclCartesianPointsAndOffsets.txt", true, "\n");
#endif
    return true;
}

// 用平面直角坐标系和wgs84原点计算出当前自车坐标系点列和offset
bool CHdmGeometry::CaclVehiclePointsAndOffsetsByCartesian(
    const std::vector<zone::common::Point>& input_pts,          // 平面直角坐标系的点列
    const std::vector<int32_t>&             input_pts_offset,   // 平面直角坐标系的点列相对于原点的offset
    const zone::common::Point&              wgs84_origin_pos,   // 平面直角坐标系原点的wgs84坐标
    const uint32_t                          origin_offset,      // 参考原点相对于path起点的距离
    const zone::common::Point&              wgs84_vehicle_pos,  // 自车当前wgs84坐标
    const uint32_t                          vehicle_offset,     // 自车相对于path起点的距离
    const float64_t                         heading_angle,      // 自车航向角度, 单位度，以正北方向为0度，顺时钟，[0,360)
    std::vector<zone::common::Point>&       vehicle_pts,        // 转换后的自车坐标系点列
    std::vector<int32_t>&                   vehicle_pts_offset) // 输出点列的offset，相对自车位置
{
    vehicle_pts.resize(input_pts.size());
    vehicle_pts_offset.resize(input_pts_offset.size());
    if (input_pts.size() == 0 || (input_pts.size() != input_pts_offset.size())) {
        HDM_ERROR("CHdmGeometry::CaclVehiclePointsByCartesian", "input_pts size:%lu, input_pts_offset size:%lu\n", input_pts.size(), input_pts_offset.size());
        return false;;
    }

    // 计算当前自车位置相对于原点的偏移量
    double offset_x;
    double offset_y;
    geographic_transform::GeoPoint origin_vehicle_pos(wgs84_origin_pos.y, wgs84_origin_pos.x);
    geographic_transform::GeoPoint current_vehicle_pos(wgs84_vehicle_pos.y, wgs84_vehicle_pos.x);
    origin_vehicle_pos.offsetOnGeoid(current_vehicle_pos, offset_x, offset_y);
    int32_t offset_distance = (int32_t)vehicle_offset - (int32_t)origin_offset;
    
    // 以当前自车位置为原点，更新点列，并旋转到自车坐标系
    double map_heading = heading_angle <= 90 ? 90 - heading_angle : 360 - (heading_angle - 90);
    for (size_t i = 0; i < input_pts.size(); i++) {
        vehicle_pts[i].x = input_pts[i].x - offset_x;
        vehicle_pts[i].y = input_pts[i].y - offset_y;
        vehicle_pts[i].z = input_pts[i].z;
        data_helper::RotFrame2D(vehicle_pts[i].x, vehicle_pts[i].y, map_heading, vehicle_pts[i].x, vehicle_pts[i].y);
        vehicle_pts_offset[i] = input_pts_offset[i] - offset_distance;

#if HDM_TEST_OUTPUT_DEBUG
        HDM_WRITE("CaclVehiclePointsByCartesian.txt", true, "index[%lu], input_pts[%f, %f], output_pts[%f, %f], input_offset[%d], output_offset[%d]",
            i, input_pts[i].x, input_pts[i].y, vehicle_pts[i].x, vehicle_pts[i].y, input_pts_offset[i], vehicle_pts_offset[i]);
#endif
    }

    return true;
}

// 计算新增输入点
bool CHdmGeometry::CaclPointByVehiclePointsAndOffsets(const int32_t              *vehicle_pts_offset,
                                                      const zone::common::Point  *vehicle_pts,
                                                      uint32_t                    vehicle_pts_num,
                                                      int32_t                     point_offset,
                                                      zone::common::Point        &point,
                                                      int32_t                    &index,
                                                      bool                       &is_origin,
                                                      bool                       start_flag)
{
    return CGeographicGeometry::CaclPointByVehiclePointsAndOffsets(vehicle_pts_offset, vehicle_pts, vehicle_pts_num, point_offset,
                                                      point, index, is_origin, start_flag);
}

/**
 * 根据start_offset和end_offset，切割它俩之间的的所有形状点，
 * 如果start_offset不在形状点列内，要把start_offset对应的点加入起点，
 * 如果end_offset不在形状点列内，要把start_offset对应的点加入终点。
 */
bool CHdmGeometry::SplitVehiclePoints(const int32_t             *vehicle_pts_offset,  // 输入坐标点偏移
                                      const zone::common::Point *vehicle_pts,         // 输入坐标点
                                      uint32_t                   vehicle_pts_num,     // 输入坐标点数目                                    
                                      int32_t                    start_offset,        // 切割的开始offset，单位厘米，相对自车位置
                                      int32_t                    end_offset,          // 切割的终止offset，单位厘米，相对自车位置                                     
                                      std::vector<zone::common::Point> &split_vehicle_pts)
{
    std::vector<int32_t>  split_vehicle_pts_offset;
    return SplitVehiclePoints(vehicle_pts_offset,
                              vehicle_pts,
                              vehicle_pts_num,
                              start_offset, 
                              end_offset,  
                              split_vehicle_pts_offset,                 
                              split_vehicle_pts);
}

/**
 * 根据start_offset和end_offset，切割所有之前的所有形状点，
 * 如果start_offset不在形状点列内，要把start_offset对应的点加入起点，
 * 如果end_offset不在形状点列内，要把start_offset对应的点加入终点。
 */
bool CHdmGeometry::SplitVehiclePoints(const int32_t             *vehicle_pts_offset,
                                      const zone::common::Point *vehicle_pts,
                                      uint32_t                   vehicle_pts_num,
                                      int32_t                    start_offset, 
                                      int32_t                    end_offset,
                                      std::vector<int32_t>      &split_vehicle_pts_offset,
                                      std::vector<zone::common::Point> &split_vehicle_pts)
{    
    // TODO(hcz): test log
#if HDM_TEST_OUTPUT_DEBUG
    HDM_WRITE("to_em_data1.txt", "CHdmGeometry::SplitVehiclePoints, vehicle_pts_num: %d, start_offset: %d, end_offset: %d.\n",
              vehicle_pts_num, start_offset, end_offset);
    HDM_WRITE("to_em_data1.txt", "CHdmGeometry::SplitVehiclePoints, vehicle_pts_offset, ");
    for (int32_t i = 0; i < vehicle_pts_num; ++i) {
        HDM_WRITE("to_em_data1.txt", "%d : %d, ", i, vehicle_pts_offset[i]);
    }
    HDM_WRITE("to_em_data1.txt", "\n");

    HDM_WRITE("to_em_data1.txt", "CHdmGeometry::SplitVehiclePoints, vehicle_pts, ");
    for (int32_t i = 0; i < vehicle_pts_num;  ++i) {
        HDM_WRITE("to_em_data1.txt", "%d : [%lf, %lf], ", i, vehicle_pts[i].x, vehicle_pts[i].y);
    }
    HDM_WRITE("to_em_data1.txt", "\n");
#endif
    bool return_value = CGeographicGeometry::SplitVehiclePoints(vehicle_pts_offset, vehicle_pts, vehicle_pts_num, start_offset, end_offset,
                                      split_vehicle_pts_offset, split_vehicle_pts);
    // TODO(hcz): test log
#if HDM_TEST_OUTPUT_DEBUG
    HDM_WRITE("to_em_data1.txt", "CHdmGeometry::SplitVehiclePoints, split_vehicle_pts_offset, ");
    for (int32_t i = 0; i < split_vehicle_pts.size(); ++i) {
        HDM_WRITE("to_em_data1.txt", "%d : %d, ", i, split_vehicle_pts_offset[i]);
    }
    HDM_WRITE("to_em_data1.txt", "\n");

    HDM_WRITE("to_em_data1.txt", "CHdmGeometry::SplitVehiclePoints, split_vehicle_pts, ");
    for (int32_t i = 0; i < split_vehicle_pts.size(); ++i) {
        HDM_WRITE("to_em_data1.txt", "%d : [%lf, %lf], ", i, split_vehicle_pts[i].x, split_vehicle_pts[i].y);
    }
    HDM_WRITE("to_em_data1.txt", "\n");
#endif
    return return_value;
}

// 切割成虚线线段
bool CHdmGeometry::Split2DashedLineSegments(const std::vector<int32_t>              &vehicle_pts_offset,
                                            const std::vector<zone::common::Point>  &vehicle_pts,
                                            int32_t                                 first_seg_length,
                                            int32_t                                 second_seg_length, 
                                            std::vector<SHdmSplitSegment>           *segments)
{
  if (vehicle_pts_offset.size() < 2 || vehicle_pts_offset.size() != vehicle_pts.size()) {
    return false;
  }
  if (!segments) {
    return false;
  }
  int32_t   curr_offset = vehicle_pts_offset[0];
  int32_t   last_offset = vehicle_pts_offset.back();
  bool      rst         = false;
  std::vector<int32_t>  split_vehicle_pts_offset;
  while (curr_offset < last_offset) {
    /** First Segment */
    SHdmSplitSegment first_segment;
    int32_t seg_start_offset = curr_offset;
    int32_t seg_end_offset   = curr_offset + first_seg_length;
    split_vehicle_pts_offset.clear();
    rst = CGeographicGeometry::SplitVehiclePoints(vehicle_pts_offset.data(), vehicle_pts.data(),
                                                  vehicle_pts.size(), seg_start_offset, seg_end_offset,
                                                  split_vehicle_pts_offset, first_segment.points);
    if (!rst) {
      return false;
    }
    first_segment.length = split_vehicle_pts_offset.back() - split_vehicle_pts_offset.front();
    segments->push_back(first_segment);

    curr_offset += first_seg_length;
    /** Second Segment */
    SHdmSplitSegment second_segment;
    if (curr_offset >= last_offset) {
      break;
    }
    seg_start_offset = curr_offset;
    seg_end_offset   = curr_offset + second_seg_length;
    split_vehicle_pts_offset.clear();
    rst = CGeographicGeometry::SplitVehiclePoints(vehicle_pts_offset.data(), vehicle_pts.data(),
                                                  vehicle_pts.size(), seg_start_offset, seg_end_offset,
                                                  split_vehicle_pts_offset, second_segment.points);
    if (!rst) {
      return false;
    }
    curr_offset += second_seg_length;
    second_segment.length = split_vehicle_pts_offset.back() - split_vehicle_pts_offset.front();
    segments->push_back(second_segment);
  }
  // TODO(hcz): 段之间头尾点坐标精度误差
  return true;
}

int CHdmGeometry::calculateLineSide(const zone::common::Point& pt1, const zone::common::Point& pt2, const zone::common::Point& pt)
{
    const double side = ((pt2.x - pt1.x) * (pt.y - pt1.y) - (pt.x - pt1.x) * (pt2.y - pt1.y));
    if (abs(side) < FPRECISION)
    {
        return 0;
    }
    else
    {
        if (side < 0)
        {
            return 1;
        }
        if (side > 0)
        {
            return -1;
        }
        return 0;
    }
}

} /* hdm_utility */