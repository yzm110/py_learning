#include "gtest/gtest.h"
#include "hdm_utility/hdm_geometry.h"

using namespace hdm_utility;

class CHdmGeometryTest : public ::testing::Test
{
    public:


    virtual void SetUp() 
    { 

    }

    virtual void TearDown() 
    { 

    }
};


TEST_F(CHdmGeometryTest, Wgs84ToGcj02)
{
    float64_t wgLon = 116.3912334328963;
    float64_t wgLat = 39.9072885060602;
    float64_t mgLon = 0.f;
    float64_t mgLat = 0.f;
    CHdmGeometry::Wgs84ToGcj02(wgLon, wgLat, mgLon, mgLat);
    // std::cout << "Wgs84ToGcj02, mgLon: " << mgLon << " mgLat:" << mgLat << std::endl;
    EXPECT_NEAR(mgLon, 116.3974745525927, 0.000001);
    EXPECT_NEAR(mgLat, 39.9086897410389, 0.000001);
    // EXPECT_FLOAT_EQ(mgLon, 116.3974745525927);
    // EXPECT_FLOAT_EQ(mgLat, 39.9086897410389);
}


TEST_F(CHdmGeometryTest, Gcj02ToWgs84)
{
    float64_t lon = 115.93668983;
    float64_t lat = 28.66540502;
    float64_t mgLon;
    float64_t mgLat;
    CHdmGeometry::Gcj02ToWgs84(lon, lat, mgLon, mgLat);
    // std::cout << "Gcj02ToWgs84, mgLon: " << mgLon << " mgLat:" << mgLat << std::endl;
    EXPECT_NEAR(mgLon, 115.931812, 0.0001);
    EXPECT_NEAR(mgLat, 28.66876, 0.0001);
    // EXPECT_FLOAT_EQ(mgLon, 115.931812);
    // EXPECT_FLOAT_EQ(mgLat, 28.66876);
}


TEST_F(CHdmGeometryTest, CalcProjectionPoint)
{
    zone::common::Point p1;
    p1.x = 72.1235;
    p1.y = 42.3521;
    p1.z = 0;
    zone::common::Point p2;
    p2.x = 72.1235;
    p2.y = 42.3530;
    p2.z = 0;
    zone::common::Point p;
    p.x = 121.190201;
    p.y = 31.276981;
    p.z = 0;
    zone::common::Point foot_point;
    foot_point.x = 0.f;
    foot_point.y = 0.f;
    bool is_inside = false;
    CHdmGeometry::CalcProjectionPoint(p1, p2, p, foot_point, is_inside);
    // std::cout << "CalcProjectionPoint, foot_point, x: " << foot_point.x << " y:" << foot_point.y << std::endl;
    EXPECT_DOUBLE_EQ(foot_point.x, 72.1235);
    EXPECT_DOUBLE_EQ(foot_point.y, 31.276981);
    EXPECT_FALSE(is_inside);

    p2.x = 72.1240;
    p2.y = 42.3521;
    CHdmGeometry::CalcProjectionPoint(p1, p2, p, foot_point, is_inside);
    // std::cout << "CalcProjectionPoint, foot_point, x: " << foot_point.x << " y:" << foot_point.y << std::endl;
    EXPECT_DOUBLE_EQ(foot_point.x, 121.190201);
    EXPECT_DOUBLE_EQ(foot_point.y, 42.3521);
    EXPECT_FALSE(is_inside);

    p2.y = 42.3530;
    CHdmGeometry::CalcProjectionPoint(p1, p2, p, foot_point, is_inside);
    // std::cout << "CalcProjectionPoint, foot_point, x: " << foot_point.x << " y:" << foot_point.y << std::endl;
    EXPECT_NEAR(foot_point.x, 78.994133, 0.000001);
    EXPECT_NEAR(foot_point.y, 54.719240, 0.000001);
    EXPECT_FALSE(is_inside);
}


TEST_F(CHdmGeometryTest, GetNearestPoint)
{
    std::vector<zone::common::Point> point_list;
    zone::common::Point p;
    p.x = 121.190486;
    p.y = 31.277252;
    point_list.push_back(p);
    p.x = 121.190201;
    p.y = 31.276981;
    point_list.push_back(p);
    p.x = 121.190183;
    p.y = 31.276965;
    point_list.push_back(p);
    p.x = 121.190153;
    p.y = 31.276945;
    point_list.push_back(p);
    p.x = 121.189968;
    p.y = 31.276762;
    point_list.push_back(p);
    p.x = 121.189737;
    p.y = 31.276543;
    point_list.push_back(p);
    p.x = 121.189447;
    p.y = 31.276268;
    point_list.push_back(p);
    p.x = 121.189411;
    p.y = 31.276230;
    point_list.push_back(p);
    p.x = 121.189381;
    p.y = 31.276189;
    point_list.push_back(p);
    p.x = 121.189360;
    p.y = 31.276157;
    point_list.push_back(p);
    p.x = 121.189342;
    p.y = 31.276123;
    point_list.push_back(p);
    zone::common::Point p1;
    p1.x = 121.190201;
    p1.y = 31.276981;
    p1.z = 0.0;
    uint16_t index = 0;
    zone::common::Point foot_point;
    foot_point.x = 0.0;
    foot_point.y = 0.0;
    float64_t distance = 0.0;
    bool find_inside = false;
    CHdmGeometry::GetNearestPoint(point_list, p1, index, foot_point, distance, find_inside);
    std::cout << "GetNearestPoint, foot_point, x: " << foot_point.x << " y:" << foot_point.y << std::endl;
    EXPECT_DOUBLE_EQ(foot_point.x, 121.190201);
    EXPECT_DOUBLE_EQ(foot_point.y, 31.276981);
    EXPECT_DOUBLE_EQ(distance, 0);
    EXPECT_TRUE(find_inside);
    p.x = 121.0;
    p.y = 31.0;
    CHdmGeometry::GetNearestPoint(point_list, p1, index, foot_point, distance, find_inside);
    std::cout << "GetNearestPoint, foot_point, x: " << foot_point.x << " y:" << foot_point.y << std::endl;
    EXPECT_DOUBLE_EQ(foot_point.x, 121.190201);
    EXPECT_DOUBLE_EQ(foot_point.y, 31.276981);
    EXPECT_DOUBLE_EQ(distance, 0);
    EXPECT_TRUE(find_inside);
}


// TEST_F(CHdmGeometryTest, GetNearestPoint2)
// {
//     // TODO(hcz)
//     // CHdmGeometry::GetNearestPoint
// }


TEST_F(CHdmGeometryTest, IsPointInside)
{
    zone::common::Point p1;
    p1.x = 72.1235;
    p1.y = 42.3521;
    p1.z = 0;
    zone::common::Point p2;
    p2.x = 72.1240;
    p2.y = 42.3530;
    p2.z = 0;
    zone::common::Point p;
    p.x = 72.1237;
    p.y = 42.3526;
    p.z = 0;
    bool rst = false;
    rst = CHdmGeometry::IsPointInside(p1, p2, p);
    EXPECT_TRUE(rst);

    p.x = 72.1250;
    p.y = 42.3526;
    rst = CHdmGeometry::IsPointInside(p1, p2, p);
    EXPECT_FALSE(rst);
}


TEST_F(CHdmGeometryTest, CalcWgs84Distance) 
{ 
    zone::common::Point p1;
    p1.x = 72.1235;
    p1.y = 42.3521;
    p1.z = 0;
    zone::common::Point p2;
    p2.x = 72.1240;
    p2.y = 42.3530;
    p2.z = 0;
    float64_t dist = CHdmGeometry::CalcWgs84Distance(p1, p2);
    // std::cout << "CalcWgs84Distance, dist1: " << dist << std::endl;
    // EXPECT_EQ(dist, 108.12737831331606);
    EXPECT_DOUBLE_EQ(dist, 108.12737831331606);

    p2.x = 100.1240;
    p2.y = 60.3530;
    dist = CHdmGeometry::CalcWgs84Distance(p1, p2);
    // std::cout << "CalcWgs84Distance, dist2: " << dist << std::endl;
    EXPECT_DOUBLE_EQ(dist, 2755100.387943864);
}


TEST_F(CHdmGeometryTest, CalculatePointByDistance)
{
    std::vector<zone::common::Point> point_list;
    zone::common::Point p;
    p.x = 121.190486;
    p.y = 31.277252;
    point_list.push_back(p);
    p.x = 121.190183;
    p.y = 31.276965;
    point_list.push_back(p);
    p.x = 121.190153;
    p.y = 31.276945;
    point_list.push_back(p);
    p.x = 121.189968;
    p.y = 31.276762;
    point_list.push_back(p);
    p.x = 121.189737;
    p.y = 31.276543;
    point_list.push_back(p);
    p.x = 121.189447;
    p.y = 31.276268;
    point_list.push_back(p);
    p.x = 121.189411;
    p.y = 31.276230;
    point_list.push_back(p);
    p.x = 121.189381;
    p.y = 31.276189;
    point_list.push_back(p);
    p.x = 121.189360;
    p.y = 31.276157;
    point_list.push_back(p);
    p.x = 121.189342;
    p.y = 31.276123;
    point_list.push_back(p);
    float64_t distance = 50;
    zone::common::Point insert_point;
    insert_point.x = 0;
    insert_point.y = 0;
    uint16_t insert_index = -1;
    CHdmGeometry::CalculatePointByDistance(point_list, distance, insert_point, insert_index);
    EXPECT_NEAR(insert_point.x, 121.1901294, 0.0000001);
    EXPECT_NEAR(insert_point.y, 31.2769216, 0.0000001);
    EXPECT_EQ(insert_index, 2);
}


TEST_F(CHdmGeometryTest, CalcVehicleDistance)
{ 
    zone::common::Point p1;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;
    zone::common::Point p2;
    p2.x = 1;
    p2.y = 1;
    p2.z = 0;
    float64_t dist = CHdmGeometry::CalcVehicleDistance(p1, p2);
    // std::cout << dist;
    // EXPECT_EQ(dist, 1.4142135623730951);
    EXPECT_DOUBLE_EQ(dist, 1.4142135623730951);
}


TEST_F(CHdmGeometryTest, Wgs84ToUtm)
{
    float64_t  lon    = 121.4685 ;
    float64_t  lat    = 31.2297;
    float64_t  height = 0.0;
	float64_t  dx     = 0.0;
    float64_t  dy     = 0.0;
    float64_t  dz     = 0.0;
    // CHdmGeometry::Wgs84ToUtm(lon, lat, height, dx, dy, dz);
    
    lon    = 100;
    lat    = -10;
    CHdmGeometry::Wgs84ToUtm(lon, lat, height, dx, dy, dz);
    // std::cout << "Wgs84ToUtm, dx1:" << dx << std::endl;
    // std::cout << "Wgs84ToUtm, dy1:" << dy << std::endl;

    // 广州
    lon    = 113.595417400;
    lat    = 22.744435950;
    CHdmGeometry::Wgs84ToUtm(lon, lat, height, dx, dy, dz);
    // std::cout << "Wgs84ToUtm, GZ dx:" << dx << ", dy" << dy << std::endl;
    EXPECT_NEAR(dx, 766544.7801555358, 0.000001);
    EXPECT_NEAR(dy, 2517564.4969607797, 0.000001);
    EXPECT_DOUBLE_EQ(dz, 0.0);
    // 50N
    lon    = 116.242161333333;
    lat    = 40.0691643333333;
    CHdmGeometry::Wgs84ToUtm(lon, lat, height, dx, dy, dz);
    // std::cout << "Wgs84ToUtm, GZ dx:" << dx << ", dy" << dy << std::endl;
    EXPECT_NEAR(dx, 435376.10572293965, 0.000001);
    EXPECT_NEAR(dy, 4435708.949468517, 0.000001);
    EXPECT_DOUBLE_EQ(dz, 0.0);
    // 上海
    lon    = 121.4685;
    lat    = 31.2297;
    CHdmGeometry::Wgs84ToUtm(lon, lat, height, dx, dy, dz);
    // std::cout << "Wgs84ToUtm, SH dx:" << dx << ", dy" << dy << std::endl;
    EXPECT_DOUBLE_EQ(dx, 354137.26529183576);
    EXPECT_DOUBLE_EQ(dy, 3456069.7907637004);
    EXPECT_DOUBLE_EQ(dz, 0.0);
}


TEST_F(CHdmGeometryTest, Wgs84ToVehicle)
{
    std::vector<zone::common::Point>   actual_vehicle_pts;
    zone::common::Point p;
    p.x = -42.8997928753;
    p.y = -2.143107154;
    actual_vehicle_pts.push_back(p);
    p.x = 0.0;
    p.y = 0.0;
    actual_vehicle_pts.push_back(p);
    p.x = 3.588422045;
    p.y = -0.44829185;
    actual_vehicle_pts.push_back(p);
    p.x = 30.389831382;
    p.y = 1.46981519;
    actual_vehicle_pts.push_back(p);
    p.x = 63.111208995;
    p.y = 3.118998052;
    actual_vehicle_pts.push_back(p);
    p.x = 104.195139322;
    p.y = 5.194353134;
    actual_vehicle_pts.push_back(p);
    p.x = 109.597677715;
    p.y = 5.75506369;
    actual_vehicle_pts.push_back(p);
    p.x = 114.830738398;
    p.y = 6.954791243;
    actual_vehicle_pts.push_back(p);
    p.x = 118.752322716;
    p.y = 8.053579917;
    actual_vehicle_pts.push_back(p);
    p.x = 122.628325971;
    p.y = 9.511116911;
    actual_vehicle_pts.push_back(p);
    std::vector<zone::common::Point>   wgs84_pts;
    // zone::common::Point p;
    p.x = 121.190486;
    p.y = 31.277252;
    wgs84_pts.push_back(p);
    p.x = 121.190183;
    p.y = 31.276965;
    wgs84_pts.push_back(p);
    p.x = 121.190153;
    p.y = 31.276945;
    wgs84_pts.push_back(p);
    p.x = 121.189968;
    p.y = 31.276762;
    wgs84_pts.push_back(p);
    p.x = 121.189737;
    p.y = 31.276543;
    wgs84_pts.push_back(p);
    p.x = 121.189447;
    p.y = 31.276268;
    wgs84_pts.push_back(p);
    p.x = 121.189411;
    p.y = 31.276230;
    wgs84_pts.push_back(p);
    p.x = 121.189381;
    p.y = 31.276189;
    wgs84_pts.push_back(p);
    p.x = 121.189360;
    p.y = 31.276157;
    wgs84_pts.push_back(p);
    p.x = 121.189342;
    p.y = 31.276123;
    wgs84_pts.push_back(p);
    uint32_t pts_number       = wgs84_pts.size();
    float64_t   heading_angle    = -134.000000;
    zone::common::Point       wgs84_vehicle_pos;
    wgs84_vehicle_pos.x       = 121.190183;
    wgs84_vehicle_pos.y       = 31.276965;
    zone::common::Point   vehicle_pts[10];
    // vehicle_pts.resize(wgs84_pts.size());
    CHdmGeometry::Wgs84ToVehicle(wgs84_pts.data(), pts_number, heading_angle, wgs84_vehicle_pos, vehicle_pts);
    for (uint32_t i = 0; i < pts_number; ++i)
    {
        //std::cout << "Wgs84ToVehicle, vehicle_pts[i] x:" << vehicle_pts[i].x << ", y:" << vehicle_pts[i].y << std::endl;
        EXPECT_NEAR(vehicle_pts[i].x, actual_vehicle_pts[i].x, 0.0000001);
        EXPECT_NEAR(vehicle_pts[i].y, actual_vehicle_pts[i].y, 0.0000001);
    }
}

TEST_F(CHdmGeometryTest, VehicleToWgs84)
{
    const uint32_t       pts_number     = 11;
    float64_t               heading_angle  = -134.000000;
    std::vector<zone::common::Point>  vehicle_pts_vec;
    vehicle_pts_vec.resize(pts_number);
    zone::common::Point p;
    vehicle_pts_vec[0].x = -42.8998;
    vehicle_pts_vec[0].y = -2.14311;
    vehicle_pts_vec[1].x = -2.46631;
    vehicle_pts_vec[1].y = -0.0449437;
    vehicle_pts_vec[2].x = 0.0;
    vehicle_pts_vec[2].y = 0.0;
    vehicle_pts_vec[3].x = 3.58842;
    vehicle_pts_vec[3].y = -0.448292;
    vehicle_pts_vec[4].x = 30.3898;
    vehicle_pts_vec[4].y = 1.46982;
    vehicle_pts_vec[5].x = 63.1112;
    vehicle_pts_vec[5].y = 3.119;
    vehicle_pts_vec[6].x = 104.195;
    vehicle_pts_vec[6].y = 5.19435;
    vehicle_pts_vec[7].x = 109.598;
    vehicle_pts_vec[7].y = 5.75506;
    vehicle_pts_vec[8].x = 114.831;
    vehicle_pts_vec[8].y = 6.95479;
    vehicle_pts_vec[9].x = 118.752;
    vehicle_pts_vec[9].y = 8.05358;
    vehicle_pts_vec[10].x = 122.628;
    vehicle_pts_vec[10].y = 9.51112;
    
    zone::common::Point  wgs84_vehicle;
    wgs84_vehicle.x       = 121.190183;
    wgs84_vehicle.y       = 31.276965;

    zone::common::Point    wgs84_pts[pts_number];
    memset(wgs84_pts, 0, sizeof(wgs84_pts));

    CHdmGeometry::VehicleToWgs84(vehicle_pts_vec.data(), 
                                 pts_number,
                                 heading_angle,
                                 wgs84_vehicle,
					             wgs84_pts);
    // std::cout << "Wgs84ToVehicle, zone:" << (int32_t(wgs84_vehicle.x / 6) + 31) << std::endl;
    int32_t i = 0;
    for (auto & v : wgs84_pts) {
        printf("VehicleToWgs84, wgs84_pts[%d], x=%.06f, y=%.06f\n", i, v.x, v.y);
        i++;
    }
    EXPECT_NEAR(wgs84_pts[0].x, 121.190486, 0.000001);
    EXPECT_NEAR(wgs84_pts[0].y, 31.277252, 0.000001);
    EXPECT_NEAR(wgs84_pts[1].x, 121.190201, 0.000001);
    EXPECT_NEAR(wgs84_pts[1].y, 31.276981, 0.000001);
    EXPECT_NEAR(wgs84_pts[2].x, 121.190183, 0.000001);
    EXPECT_NEAR(wgs84_pts[2].y, 31.276965, 0.000001);
    EXPECT_NEAR(wgs84_pts[3].x, 121.190153, 0.000001);
    EXPECT_NEAR(wgs84_pts[3].y, 31.276945, 0.000001);
    EXPECT_NEAR(wgs84_pts[4].x, 121.189968, 0.000001);
    EXPECT_NEAR(wgs84_pts[4].y, 31.276762, 0.000001);
    EXPECT_NEAR(wgs84_pts[5].x, 121.189737, 0.000001);
    EXPECT_NEAR(wgs84_pts[5].y, 31.276543, 0.000001);
    EXPECT_NEAR(wgs84_pts[6].x, 121.189447, 0.000001);
    EXPECT_NEAR(wgs84_pts[6].y, 31.276268, 0.000001);
    EXPECT_NEAR(wgs84_pts[7].x, 121.189411, 0.000001);
    EXPECT_NEAR(wgs84_pts[7].y, 31.276230, 0.000001);
    EXPECT_NEAR(wgs84_pts[8].x, 121.189381, 0.000001);
    EXPECT_NEAR(wgs84_pts[8].y, 31.276189, 0.000001);
    EXPECT_NEAR(wgs84_pts[9].x, 121.189360, 0.000001);
    EXPECT_NEAR(wgs84_pts[9].y, 31.276157, 0.000001);
    EXPECT_NEAR(wgs84_pts[10].x, 121.189342, 0.000001);
    EXPECT_NEAR(wgs84_pts[10].y, 31.276123, 0.000001);
}


TEST_F(CHdmGeometryTest, CalcAngle)
{
    zone::common::Point cur_point;
    zone::common::Point ahead_point;
    float64_t angle = 0.0;
    cur_point.x   = 1.11;
    cur_point.y   = 1.0;
    ahead_point.x = 1.11;
    ahead_point.y = 1.0;
    angle = CHdmGeometry::CalcAngle(cur_point, ahead_point);
    // std::cout << "CalcAngle, angle0: " << angle << std::endl;
    EXPECT_DOUBLE_EQ(angle, 0.0);

    cur_point.x   = 1.0;
    cur_point.y   = 1.0;
    ahead_point.x = 2.0;
    ahead_point.y = 2.0;
    angle = CHdmGeometry::CalcAngle(cur_point, ahead_point);
    // std::cout << "CalcAngle, angle1: " << angle << std::endl;
    EXPECT_DOUBLE_EQ(angle, 45);

    cur_point.x   = 25.0;
    cur_point.y   = 45.0;
    ahead_point.x = 75.0;
    ahead_point.y = 100.0;
    angle = CHdmGeometry::CalcAngle(cur_point, ahead_point);
    // std::cout << "CalcAngle, angle2: " << angle << std::endl;
    EXPECT_DOUBLE_EQ(angle, 42.273689006093733);

    cur_point.x   = 75.0;
    cur_point.y   = 100.0;
    ahead_point.x = 25.0;
    ahead_point.y = 45.0;
    angle = CHdmGeometry::CalcAngle(cur_point, ahead_point);
    // std::cout << "CalcAngle, angle3: " << angle << std::endl;
    EXPECT_DOUBLE_EQ(angle, 222.27368900609372); // 222.273689006094

    cur_point.x   = 25.0;
    cur_point.y   = 45.0;
    ahead_point.x = -75.0;
    ahead_point.y = 100.0;
    angle = CHdmGeometry::CalcAngle(cur_point, ahead_point);
    // std::cout << "CalcAngle, angle4: " << angle << std::endl;
    EXPECT_DOUBLE_EQ(angle, 298.81079374297303);
}

TEST_F(CHdmGeometryTest, CaclVehiclePointsAndOffsets2)
{
    int32_t  min_offset_range = -10000;
    int32_t  max_offset_range = 20000;
    int32_t  start_offset     = -1533;
    int32_t  end_offset       = 10773;
    zone::common::Point       wgs84_vehicle_pos;
    wgs84_vehicle_pos.x       = 121.190183;
    wgs84_vehicle_pos.y       = 31.276965;
    float64_t   heading_angle    = -134.000000;
    std::vector<zone::common::Point>   wgs84_pts;
    zone::common::Point p;
    p.x = 121.190486;
    p.y = 31.277252;
    wgs84_pts.push_back(p);
    p.x = 121.190201;
    p.y = 31.276981;
    wgs84_pts.push_back(p);
    uint32_t  pts_number = wgs84_pts.size();
    std::vector<int32_t> vehicle_pts_offset;
    std::vector<zone::common::Point>   vehicle_pts;
    CHdmGeometry::CaclVehiclePointsAndOffsets(min_offset_range, max_offset_range, start_offset, end_offset,
                wgs84_vehicle_pos, heading_angle, wgs84_pts.data(), pts_number, vehicle_pts_offset, vehicle_pts);
    EXPECT_EQ(2, vehicle_pts.size());
    EXPECT_EQ(2, vehicle_pts.size());
}

TEST_F(CHdmGeometryTest, CaclVehiclePointsAndOffsets)
{ 
    int32_t  min_offset_range = -200;
    int32_t  max_offset_range = 14000;
    int32_t  start_offset     = -228;
    int32_t  end_offset       = 22989;
    zone::common::Point       wgs84_vehicle_pos;
    wgs84_vehicle_pos.x       = 121.190183;
    wgs84_vehicle_pos.y       = 31.276965;
    float64_t   heading_angle    = -134.000000;
    std::vector<zone::common::Point>   wgs84_pts;
    zone::common::Point p;
    p.x = 121.190486;
    p.y = 31.277252;
    wgs84_pts.push_back(p);
    p.x = 121.190201;
    p.y = 31.276981;
    wgs84_pts.push_back(p);
    p.x = 121.190183;
    p.y = 31.276965;
    wgs84_pts.push_back(p);
    p.x = 121.190153;
    p.y = 31.276945;
    wgs84_pts.push_back(p);
    p.x = 121.189968;
    p.y = 31.276762;
    wgs84_pts.push_back(p);
    p.x = 121.189737;
    p.y = 31.276543;
    wgs84_pts.push_back(p);
    p.x = 121.189447;
    p.y = 31.276268;
    wgs84_pts.push_back(p);
    p.x = 121.189411;
    p.y = 31.276230;
    wgs84_pts.push_back(p);
    p.x = 121.189381;
    p.y = 31.276189;
    wgs84_pts.push_back(p);
    p.x = 121.189360;
    p.y = 31.276157;
    wgs84_pts.push_back(p);
    p.x = 121.189342;
    p.y = 31.276123;
    wgs84_pts.push_back(p);
    uint32_t                           pts_number = wgs84_pts.size();
    std::vector<int32_t>               vehicle_pts_offset;
    std::vector<zone::common::Point>   vehicle_pts;
    CHdmGeometry::CaclVehiclePointsAndOffsets(min_offset_range, max_offset_range, start_offset, end_offset,
                wgs84_vehicle_pos, heading_angle, wgs84_pts.data(), pts_number, vehicle_pts_offset, vehicle_pts);
    std::vector<int32_t> temp_vehicle_pts_offset;
    temp_vehicle_pts_offset.reserve(pts_number);
    std::vector<zone::common::Point>   temp_vehicle_pts;
    temp_vehicle_pts.reserve(pts_number);
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
    }
    // 计算offset换算比例
    float64_t percent = delta_offset / sum_dis;
    // 计算输出点列的offset
    for (float64_t& distance : distance_vec) {
        uint32_t offset = start_offset + distance * percent;
        temp_vehicle_pts_offset.push_back(offset);
    }
    // 在正常范围内
    for (uint32_t i = pts_number; i < vehicle_pts_offset.size(); i++)
    {
        EXPECT_DOUBLE_EQ(vehicle_pts_offset[pts_number], temp_vehicle_pts_offset[i - pts_number]);
    }
    CHdmGeometry::Wgs84ToVehicle(wgs84_pts.data(), pts_number, heading_angle, wgs84_vehicle_pos,
                                 &temp_vehicle_pts[0]);
    for (uint32_t i = pts_number; i < vehicle_pts_offset.size(); i++)
    {
        EXPECT_DOUBLE_EQ(vehicle_pts[pts_number].x, temp_vehicle_pts[i - pts_number].x);
        EXPECT_DOUBLE_EQ(vehicle_pts[pts_number].y, temp_vehicle_pts[i - pts_number].y);
    }

    //不在范围内
    bool  rst = false;
    min_offset_range = -10000;
    max_offset_range = -200;
    rst = CHdmGeometry::CaclVehiclePointsAndOffsets(min_offset_range, max_offset_range, start_offset, end_offset,
                wgs84_vehicle_pos, heading_angle, wgs84_pts.data(), pts_number, vehicle_pts_offset, vehicle_pts);
    //std::cout << "CaclVehiclePointsAndOffsets, vehicle_pts_offset[0]: " << vehicle_pts_offset[0] << std::endl;
    EXPECT_TRUE(rst);

    min_offset_range = 20000;
    max_offset_range = 30000;
    rst = CHdmGeometry::CaclVehiclePointsAndOffsets(min_offset_range, max_offset_range, start_offset, end_offset,
                wgs84_vehicle_pos, heading_angle, wgs84_pts.data(), pts_number, vehicle_pts_offset, vehicle_pts);
    //std::cout << "CaclVehiclePointsAndOffsets, vehicle_pts_offset.back(): " << vehicle_pts_offset.back() << std::endl;
    EXPECT_TRUE(rst);

    // pts_number < 0
    min_offset_range = -200;
    max_offset_range = 14000;
    pts_number = 0;
    rst = CHdmGeometry::CaclVehiclePointsAndOffsets(min_offset_range, max_offset_range, start_offset, end_offset,
                wgs84_vehicle_pos, heading_angle, wgs84_pts.data(), pts_number, vehicle_pts_offset, vehicle_pts);
    EXPECT_FALSE(rst);

    //start_offset >= end_offset
    start_offset     = 22989;
    end_offset       = 20000;
    pts_number = 11;
    rst = CHdmGeometry::CaclVehiclePointsAndOffsets(min_offset_range, max_offset_range, start_offset, end_offset,
                wgs84_vehicle_pos, heading_angle, wgs84_pts.data(), pts_number, vehicle_pts_offset, vehicle_pts);
    EXPECT_FALSE(rst);

    // wgs84点列指针是空
    rst = CHdmGeometry::CaclVehiclePointsAndOffsets(min_offset_range, max_offset_range, start_offset, end_offset,
                wgs84_vehicle_pos, heading_angle, wgs84_pts.data(), pts_number, vehicle_pts_offset, vehicle_pts);
    EXPECT_FALSE(rst);
}


TEST_F(CHdmGeometryTest, CaclPointByVehiclePointsAndOffsets)
{ 
    std::vector<int32_t> vehicle_pts_offset{-224, 3808, 4055, 4413, 7076, 10315, 14389, 14921, 14921, 15461, 15859, 16271};
    std::vector<zone::common::Point> vehicle_pts;
    zone::common::Point p;
    p.x = -2.378641;
    p.y = -1.683084;
    vehicle_pts.push_back(p);
    p.x = 38.307287;
    p.y = 0.489329;
    vehicle_pts.push_back(p);
    p.x = 40.807667;
    p.y = 0.554269;
    vehicle_pts.push_back(p);
    p.x = 44.388003;
    p.y = 0.040404;
    vehicle_pts.push_back(p);
    p.x = 71.230045;
    p.y = 1.966867;
    vehicle_pts.push_back(p);
    p.x = 103.915718;
    p.y = 3.602162;
    vehicle_pts.push_back(p);
    p.x = 145.027641;
    p.y = 5.703701;
    vehicle_pts.push_back(p);
    p.x = 145.027641;
    p.y = 5.703701;
    vehicle_pts.push_back(p);
    p.x = 150.377084;
    p.y = 6.273315;
    vehicle_pts.push_back(p);
    p.x = 155.691330;
    p.y = 7.508228;
    vehicle_pts.push_back(p);
    p.x = 159.565392;
    p.y = 8.565494;
    vehicle_pts.push_back(p);
    p.x = 163.449781;
    p.y = 10.085088;
    vehicle_pts.push_back(p);
    uint32_t            vehicle_pts_num = vehicle_pts.size();
    int32_t             point_offset = 7076;
    zone::common::Point point;
    point.x = 0;
    point.y = 0;
    int32_t             index = -1;
    bool                is_origin = false;
    bool                start_flag = false;
    // 在区域内
    CHdmGeometry::CaclPointByVehiclePointsAndOffsets(vehicle_pts_offset.data(), vehicle_pts.data(), vehicle_pts_num, 
                     point_offset, point, index, is_origin, start_flag);
    //std::cout << "CaclPointByVehiclePointsAndOffsets, point, x:" << point.x << ", y:" << point.y << std::endl;
    EXPECT_EQ(index, 4);
    EXPECT_DOUBLE_EQ(point.x, vehicle_pts[4].x);
    EXPECT_DOUBLE_EQ(point.y, vehicle_pts[4].y);
    EXPECT_TRUE(is_origin);

    point_offset = 12000;
    CHdmGeometry::CaclPointByVehiclePointsAndOffsets(vehicle_pts_offset.data(), vehicle_pts.data(), vehicle_pts_num, 
                     point_offset, point, index, is_origin, start_flag);
    //std::cout << "CaclPointByVehiclePointsAndOffsets, point, x:" << point.x << ", y:" << point.y << std::endl;
    EXPECT_EQ(index, 5);
    EXPECT_NEAR(point.x, 120.9195448, 0.0000001);
    EXPECT_NEAR(point.y, 4.471355229, 0.0000001);
    EXPECT_FALSE(is_origin);

    // 出现重复形点
    point_offset = 14921;
    CHdmGeometry::CaclPointByVehiclePointsAndOffsets(vehicle_pts_offset.data(), vehicle_pts.data(), vehicle_pts_num, 
                     point_offset, point, index, is_origin, start_flag);
    //std::cout << "CaclPointByVehiclePointsAndOffsets, point, x:" << point.x << ", y:" << point.y << std::endl;
    EXPECT_EQ(index, 7);
    EXPECT_EQ(point.x, vehicle_pts[7].x);
    EXPECT_EQ(point.y, vehicle_pts[7].y);
    EXPECT_TRUE(is_origin);

    // (不在区域内, 没有交集)在最后一个点的后面
    point_offset = -300;
    CHdmGeometry::CaclPointByVehiclePointsAndOffsets(vehicle_pts_offset.data(), vehicle_pts.data(), vehicle_pts_num, 
                     point_offset, point, index, is_origin, start_flag);
    //std::cout << "CaclPointByVehiclePointsAndOffsets, point, x:" << point.x << ", y:" << point.y << std::endl;
    EXPECT_EQ(point.x, vehicle_pts[0].x);
    EXPECT_EQ(point.y, vehicle_pts[0].y);
    EXPECT_EQ(index, 0);
    EXPECT_TRUE(is_origin);

    // (不在区域内, 没有交集)在最后一个点的前方
    point_offset = 17000;
    CHdmGeometry::CaclPointByVehiclePointsAndOffsets(vehicle_pts_offset.data(), vehicle_pts.data(), vehicle_pts_num, 
                     point_offset, point, index, is_origin, start_flag);
    //std::cout << "CaclPointByVehiclePointsAndOffsets, point, x:" << point.x << ", y:" << point.y << std::endl;
    EXPECT_EQ(point.x, vehicle_pts[vehicle_pts_num - 1].x);
    EXPECT_EQ(point.y, vehicle_pts[vehicle_pts_num - 1].y);
    EXPECT_EQ(index, 11);
    EXPECT_TRUE(is_origin);

    //vehicle_pts_num <= 1
    bool rst = false;
    point_offset = 12000;
    vehicle_pts_num = 1;
    rst = CHdmGeometry::CaclPointByVehiclePointsAndOffsets(vehicle_pts_offset.data(), vehicle_pts.data(), vehicle_pts_num, 
                     point_offset, point, index, is_origin, start_flag);
    EXPECT_FALSE(rst);

    //nullptr == vehicle_pts
    vehicle_pts_num = 11;
    rst = CHdmGeometry::CaclPointByVehiclePointsAndOffsets(vehicle_pts_offset.data(), nullptr, vehicle_pts_num, 
                     point_offset, point, index, is_origin, start_flag);
    EXPECT_FALSE(rst);
}

TEST_F(CHdmGeometryTest, SplitVehiclePoints)
{ 
    std::vector<int32_t> vehicle_pts_offset{-224, 3808, 4055, 4413, 7076, 10315, 14389, 14921, 15461, 15859, 16271};
    std::vector<zone::common::Point> vehicle_pts;
    zone::common::Point p;
    p.x = -2.378641;
    p.y = -1.683084;
    vehicle_pts.push_back(p);
    p.x = 38.307287;
    p.y = 0.489329;
    vehicle_pts.push_back(p);
    p.x = 40.807667;
    p.y = 0.554269;
    vehicle_pts.push_back(p);
    p.x = 44.388003;
    p.y = 0.040404;
    vehicle_pts.push_back(p);
    p.x = 71.230045;
    p.y = 1.966867;
    vehicle_pts.push_back(p);
    p.x = 103.915718;
    p.y = 3.602162;
    vehicle_pts.push_back(p);
    p.x = 145.027641;
    p.y = 5.703701;
    vehicle_pts.push_back(p);
    p.x = 150.377084;
    p.y = 6.273315;
    vehicle_pts.push_back(p);
    p.x = 155.691330;
    p.y = 7.508228;
    vehicle_pts.push_back(p);
    p.x = 159.565392;
    p.y = 8.565494;
    vehicle_pts.push_back(p);
    p.x = 163.449781;
    p.y = 10.085088;
    vehicle_pts.push_back(p);
    uint32_t vehicle_pts_num = 11;
    int32_t start_offset = 3808;
    int32_t end_offset = 14000;
    std::vector<int32_t>      split_vehicle_pts_offset;
    std::vector<zone::common::Point> split_vehicle_pts;
    zone::common::Point point;
    point.x = 0;
    point.y = 0;
    int32_t             index = -1;
    bool                is_origin = false;
    bool                start_flag = false;
    //可行域且start_offset < end_offset;
    CHdmGeometry::SplitVehiclePoints(vehicle_pts_offset.data(), vehicle_pts.data(), vehicle_pts_num, start_offset, end_offset, 
                                    split_vehicle_pts_offset, split_vehicle_pts);
    // std::cout << "SplitVehiclePoints, split_vehicle_pts[0], x:" << split_vehicle_pts[0].x << ", y:" << split_vehicle_pts[0].y << std::endl;
    // std::cout << "SplitVehiclePoints, split_vehicle_pts[5], x:" << split_vehicle_pts[5].x << ", y:" << split_vehicle_pts[5].y << std::endl;
    CHdmGeometry::CaclPointByVehiclePointsAndOffsets(vehicle_pts_offset.data(), vehicle_pts.data(), vehicle_pts_num, 
                end_offset, point, index, is_origin, start_flag);
    for (uint32_t i = 1; i < 6; i++)
    {
        EXPECT_DOUBLE_EQ(split_vehicle_pts_offset[i-1], vehicle_pts_offset[i]);
        EXPECT_DOUBLE_EQ(split_vehicle_pts[i-1].x, vehicle_pts[i].x);
        EXPECT_DOUBLE_EQ(split_vehicle_pts[i-1].y, vehicle_pts[i].y);
    }
    EXPECT_DOUBLE_EQ(split_vehicle_pts_offset[5], 14000);
    EXPECT_DOUBLE_EQ(split_vehicle_pts[5].x, point.x);
    EXPECT_DOUBLE_EQ(split_vehicle_pts[5].y, point.y);

    // 整体不在范围内
    bool rst = false;
    start_offset =20000;
    end_offset = 24000;
    rst = CHdmGeometry::SplitVehiclePoints(vehicle_pts_offset.data(), vehicle_pts.data(), vehicle_pts_num, start_offset, end_offset, split_vehicle_pts);
    //std::cout << "SplitVehiclePoints, vehicle_pts_offset[10], offset:" << vehicle_pts_offset[10] << std::endl;
    EXPECT_FALSE(rst);

    // start_offset >= end_offset
    start_offset = 14808;
    end_offset = 14000;
    rst = CHdmGeometry::SplitVehiclePoints(vehicle_pts_offset.data(), vehicle_pts.data(), vehicle_pts_num, start_offset, end_offset, split_vehicle_pts);
    EXPECT_TRUE(rst);

    // TODO
    // index不在范围内
    // int32_t start_offset = 3808;
    // int32_t end_offset = 14000;
    // rst = CHdmGeometry::SplitVehiclePoints(vehicle_pts_offset.data(), vehicle_pts.data(), vehicle_pts_num, start_offset, end_offset, split_vehicle_pts);
    // EXPECT_FALSE(rst);


    // 0 >= vehicle_pts_num
    vehicle_pts_num = 0;
    rst = CHdmGeometry::SplitVehiclePoints(vehicle_pts_offset.data(), vehicle_pts.data(), vehicle_pts_num, start_offset, end_offset, split_vehicle_pts);
    EXPECT_FALSE(rst);

    // (nullptr == vehicle_pts_offset) || (nullptr == vehicle_pts)
    rst = CHdmGeometry::SplitVehiclePoints(vehicle_pts_offset.data(), vehicle_pts.data(), vehicle_pts_num, start_offset, end_offset, split_vehicle_pts);
    EXPECT_FALSE(rst);
}

TEST_F(CHdmGeometryTest, SplitVehiclePoints2)
{ 
    int32_t start_offset = -1674;
    int32_t end_offset   = 818;
    std::vector<int32_t> vehicle_pts_offset{-1674, -728, 818};
    std::vector<zone::common::Point> vehicle_pts;
    zone::common::Point p;
    p.x = -17.392792;
    p.y = -3.793621;
    vehicle_pts.push_back(p);
    p.x = -7.719385;
    p.y = -3.804927;
    vehicle_pts.push_back(p);
    p.x = 8.099108;
    p.y = -3.829217;
    vehicle_pts.push_back(p);
    uint32_t vehicle_pts_num = vehicle_pts.size();
    
    std::vector<int32_t>             split_vehicle_pts_offset;
    std::vector<zone::common::Point> split_vehicle_pts;
    // 可行域且start_offset < end_offset;
    // std::cout << "SplitVehiclePoints2, vehicle_pts_num:" << vehicle_pts_num << std::endl;
    CHdmGeometry::SplitVehiclePoints(vehicle_pts_offset.data(), vehicle_pts.data(), vehicle_pts_num, start_offset, end_offset, 
                                     split_vehicle_pts_offset, split_vehicle_pts);
    for (uint32_t i = 0; i < vehicle_pts_num; i++) {
        EXPECT_DOUBLE_EQ(split_vehicle_pts_offset[i], vehicle_pts_offset[i]);
        EXPECT_DOUBLE_EQ(split_vehicle_pts[i].x, vehicle_pts[i].x);
        EXPECT_DOUBLE_EQ(split_vehicle_pts[i].y, vehicle_pts[i].y);
        // std::cout << "SplitVehiclePoints2, i: " << i << std::endl;
        // std::cout << "SplitVehiclePoints2, " 
        //           << "split offset: " << split_vehicle_pts_offset[i] 
        //           << ", vehicle_pts_offset: " << vehicle_pts_offset[i] << std::endl;
        // std::cout << "split point x: " << split_vehicle_pts[i].x << ", split point y: " << split_vehicle_pts[i].y
        //           << "; vehicle_pts x: " << vehicle_pts[i].x << ", vehicle_pts y: " << vehicle_pts[i].y << std::endl;
    }
}


TEST_F(CHdmGeometryTest, Split2DashedLineSegments)
{
    std::vector<int32_t> vehicle_pts_offset{-1674, -728, 818};
    std::vector<zone::common::Point> vehicle_pts;
    zone::common::Point p;
    p.x = -17.392792;
    p.y = -3.793621;
    vehicle_pts.push_back(p);
    p.x = -7.719385;
    p.y = -3.804927;
    vehicle_pts.push_back(p);
    p.x = 8.099108;
    p.y = -3.829217;
    vehicle_pts.push_back(p);
    uint32_t vehicle_pts_num = vehicle_pts.size();
    std::vector<uint32_t> seg_length{600, 900};
    int32_t seg_start_offset = vehicle_pts_offset[0];
    std::vector<int32_t> split_vehicle_pts_offset;
    std::vector<zone::common::Point> split_vehicle_pts;
    // uint32_t first_seg_length  = 600;  // 6米
    // uint32_t second_seg_length = 900;  // 9米 
    // int32_t seg_start_offset = vehicle_pts_offset[0];
    std::vector<SHdmSplitSegment> segments;
    CHdmGeometry::Split2DashedLineSegments(vehicle_pts_offset, vehicle_pts,
                                           seg_length[0], seg_length[1], 
                                           &segments);
    for (uint32_t i = 0; i < segments.size(); i++) {
        int32_t seg_end_offset   = seg_start_offset + segments[i].length;
        CHdmGeometry::SplitVehiclePoints(vehicle_pts_offset.data(), vehicle_pts.data(),
                                         vehicle_pts.size(), seg_start_offset, seg_end_offset,
                                         split_vehicle_pts_offset, split_vehicle_pts);

        seg_start_offset = seg_end_offset;
        EXPECT_DOUBLE_EQ(split_vehicle_pts_offset.back() - split_vehicle_pts_offset.front(), segments[i].length);
        if (split_vehicle_pts.size() == segments[i].points.size()) {
            for (uint32_t j = 0; j < split_vehicle_pts.size(); ++j) {
            EXPECT_DOUBLE_EQ(split_vehicle_pts[j].x, segments[i].points[j].x);
            EXPECT_DOUBLE_EQ(split_vehicle_pts[j].y, segments[i].points[j].y);
            }
        }
        split_vehicle_pts_offset.clear();
        split_vehicle_pts.clear();
    }
    bool rst = false;
    vehicle_pts_offset.pop_back();
    rst = CHdmGeometry::Split2DashedLineSegments(vehicle_pts_offset, vehicle_pts,
                                           seg_length[0], seg_length[1], 
                                           &segments);
    EXPECT_FALSE(rst);

    rst = CHdmGeometry::Split2DashedLineSegments(vehicle_pts_offset, vehicle_pts,
                                           seg_length[0], seg_length[1], 
                                           nullptr);
    EXPECT_FALSE(rst);
}
