#include "hdm_utility/hdm_date_time.h"
#include "hdm_utility/hdm_dbg_log.h"
#include "hdm_utility/hdm_ehr_output.h"
#include "common/data/ehr_car_position_data.h"
#include "common/data/ehr_section_data.h"
#include "common/data/ehr_to_em_data.h"
#include "hdm_utility/hdm_geometry.h"


using namespace zone::common;
using namespace hdm_utility;

namespace hdm_utility
{	

bool CHdmEhrOutput::OutputCarPosition(const char_t *file_path, const std::shared_ptr<zone::common::EHRCarPosition> &car_position)
{
    std::string str_date_time = CHdmDateTime::GetFormatDateTimeString();
    HDM_WRITE(file_path, true,  "date_time[%s]***********CarPosition[%p]********exe_run_time[%u]******\n", str_date_time.c_str(), car_position.get(), CHdmDateTime::GetExeRunTime());
    if (nullptr == car_position) {  
        return false;
    }
    HDM_WRITE(file_path, false,  "\ttimestamp_[%llu]\n", car_position->timestamp_);
    HDM_WRITE(file_path, false,  "\tabs_coord_[%.8f, %.8f, %.8f]\n", car_position->abs_coord_.x, car_position->abs_coord_.y, car_position->abs_coord_.z);
    HDM_WRITE(file_path, false, "\tenu_coord_[%.8f, %.8f, %.8f]\n",car_position->enu_coord_.x, car_position->enu_coord_.y, car_position->enu_coord_.z);
    HDM_WRITE(file_path, false,  "\tsection_id_[%llu]\n", car_position->section_id_);
    HDM_WRITE(file_path, false,  "\tpath_id_[%u]\n", car_position->path_id_);
    HDM_WRITE(file_path, false,  "\toffset_[%u]\n", car_position->offset_);
    HDM_WRITE(file_path, false,  "\tconfidence_[%f]\n",car_position->confidence_);
    HDM_WRITE(file_path, false,  "\theading_[%f]\n", car_position->heading_);
    HDM_WRITE(file_path, false,  "\tprefered_path_id_[%u]\n", car_position->prefered_path_id_);
    HDM_WRITE(file_path, false,  "\tlane_id_[%u]\n", car_position->lane_id_); 
    HDM_WRITE(file_path, false, "\tcurrent_lane_width_[%u,%u,%u,%u,%u]\n",
                car_position->current_lane_width_.trans_width_offset_,
                car_position->current_lane_width_.trans_width_end_offset_,
                car_position->current_lane_width_.start_width_,
                car_position->current_lane_width_.end_width_,
                car_position->current_lane_width_.trans_width_); 

    HDM_WRITE(file_path, false, "\tleft_lane_width_[%u,%u,%u,%u,%u]\n",
                car_position->left_lane_width_.trans_width_offset_,
                car_position->left_lane_width_.trans_width_end_offset_,
                car_position->left_lane_width_.start_width_,
                car_position->left_lane_width_.end_width_,
                car_position->left_lane_width_.trans_width_); 
    
    HDM_WRITE(file_path, true, "\tright_lane_width_[%u,%u,%u,%u,%u]\n",
                car_position->right_lane_width_.trans_width_offset_,
                car_position->right_lane_width_.trans_width_end_offset_,
                car_position->right_lane_width_.start_width_,
                car_position->right_lane_width_.end_width_,
                car_position->right_lane_width_.trans_width_);  

    return true;
}

bool CHdmEhrOutput::OutputSectionList(const char_t *file_path, const std::shared_ptr<zone::common::EHRSectionList> &section_list)
{    
    std::string str_date_time = CHdmDateTime::GetFormatDateTimeString();
    if (nullptr == section_list) {
        HDM_WRITE(file_path, true,  "date_time[%s]***********SectionList[%p]*********exe_run_time[%u]*****\n", str_date_time.c_str(), section_list.get(), CHdmDateTime::GetExeRunTime());
        return false;
    }
    HDM_WRITE(file_path, false,  "date_time[%s]***********SectionList[%p]***Size[%lu]*********\n", 
            str_date_time.c_str(), 
            section_list.get(), 
            section_list->section_list_.size());

    for (size_t i = 0; i < section_list->section_list_.size(); ++i) {
        auto &section = section_list->section_list_[i];
        HDM_WRITE(file_path, false,  "\tSection[%lu]=section_id_[%llu],section_index_[%u], path_id_[%u], offset[%u=>%u], frc_[%u], fow_[%u], part_of_route_[%d],total_lane_num_[%hhu], is_complex_intersection_[%d]\n", 
            i, 
            section->section_id_,
            section->section_index_, 
            section->path_id_, 
            section->start_offset_, 
            section->end_offset_, 
            section->frc_, 
            section->fow_, 
            section->part_of_route_,
            section->total_lane_num_, 
            section->is_complex_intersection_);
        //   std::vector<EHRTrafficSign> traffic_sign_list_;            // 交通标志牌
        for(size_t j = 0; j < section->traffic_sign_list_.size(); ++j){
            EHRTrafficSign & trafficsign=section->traffic_sign_list_[j];
            HDM_WRITE(file_path, false,  "\t\trafficsign[%lu]=traffic_sign_type[%d],offset[%u=>%u]\n", 
                    j,
                    (int32_t)trafficsign.type_ ,
                    trafficsign.start_offset_, 
                    trafficsign.end_offset_);
        }
        //   std::vector<EHRTrafficLight> traffic_light_list_;          // 交通灯
        for(size_t j = 0; j < section->traffic_light_list_.size(); ++j){
            EHRTrafficLight & traffic_light=section->traffic_light_list_[j];
            HDM_WRITE(file_path, false,  "\t\traffic_light[%lu]=traffic_light_type[%d],offset[%u=>%u],coord_[%lf,%lf,%lf],bouding_box_vector_[%lf,%lf,%lf] \n", 
                    j,
                    (int32_t)traffic_light.type_ ,
                    traffic_light.start_offset_, 
                    traffic_light.end_offset_,
                    traffic_light.coord_.x,
                    traffic_light.coord_.y,
                    traffic_light.coord_.z,
                    traffic_light.bouding_box_vector_.x,
                    traffic_light.bouding_box_vector_.y,
                    traffic_light.bouding_box_vector_.z);
        }
        //   std::vector<EHRSpecialSituation> special_situation_list_;  // 特殊位置信息
        for(size_t j = 0; j < section->special_situation_list_.size(); ++j){
            EHRSpecialSituation & specialsituation=section->special_situation_list_[j];
            HDM_WRITE(file_path, false,  "\t\tspecialsituation[%lu]=special_situation_type[%d],offset[%u=>%u]\n",
                    j,
                    specialsituation.type_,
                    specialsituation.start_offset_,
                    specialsituation.end_offset_);
        }
        //   std::vector<EHRSubPath> subpath_list_;                     // 分岔信息
        for (size_t j = 0; j < section->subpath_list_.size(); ++j) {
            EHRSubPath & subpath = section->subpath_list_[j];
            HDM_WRITE(file_path, false,  "\t\tSubpath[%lu]=subpath_id_[%u], turn_angle_[%f], offset[%u=>%u]\n", 
                    j, 
                    subpath.subpath_id_, 
                    subpath.turn_angle_, 
                    subpath.start_offset_, 
                    subpath.end_offset_);
        }
        //   std::vector<EHRPointHeading> point_heading_list_;          // 航向角信息
        // for (size_t j = 0; j < section->point_heading_list_.size(); ++j) {
        //     EHRPointHeading & pointheading=section->point_heading_list_[j];
        //     HDM_WRITE(file_path, false,  "\t\tpointheading[%u]=point_id[%u] ,offset[%u],heading_[%f],curvature_[%f],slope_[%f],super_elevation_[%d]\n",
        //             j,
        //             pointheading.point_id_,
        //             pointheading.offset_,
        //             pointheading.heading_,
        //             pointheading.curvature_,
        //             pointheading.slope_,
        //             (int32_t)pointheading.super_elevation_);
        // }
        //   std::vector<EHRLane> lane_list_;                           // 车道信息
        for (size_t j = 0; j < section->lane_list_.size(); ++j) {
            EHRLane &  ehrlane=section->lane_list_[j];
            HDM_WRITE(file_path, false,  "\t\tlane[%lu]=lane_id[%hhu],lane_index[%hhu],section_id[%llu],path_id[%u],\
                    speed[%hhu =>%hhu],lane_direction[%d],lane_type[%d],lanewith[%u,%u,%u,%u.%u],\
                    lanetransition_type[%d],lane_access_characterstic_[%d,%d],speed_limit_change_point_list[%lu,%lu]\n",
                    j,
                    ehrlane.lane_id_,
                    ehrlane.lane_index_,
                    ehrlane.section_id_,
                    ehrlane.path_id_,
                    ehrlane.min_speed_limit_,
                    ehrlane.max_speed_limit_,
                    (int32_t)ehrlane.direction_,
                    (int32_t)ehrlane.lane_type_,
                    ehrlane.lane_width_.trans_width_offset_,
                    ehrlane.lane_width_.trans_width_end_offset_,
                    ehrlane.lane_width_.start_width_,
                    ehrlane.lane_width_.end_width_,
                    ehrlane.lane_width_.trans_width_,
                    (int32_t)ehrlane.lane_transition_,
                    (int32_t)ehrlane.lane_access_characterstic_.vehicle_type_,
                    ehrlane.lane_access_characterstic_.has_access_characterstic_,
                    ehrlane.max_speed_limit_change_point_list_.size(),
                    ehrlane.min_speed_limit_change_point_list_.size()
            );

            for(size_t k=0;k < ehrlane.offset_heading_list_.size();++k){
                EHROffsetHeading & offset_heading=ehrlane.offset_heading_list_[k];
                HDM_WRITE(file_path, false, "\t\t\toffset_headig_list_[%lu]=offset_[%u],heading_[%f]\n",
                    k,
                    offset_heading.offset_,
                    offset_heading.heading_);
            }
            //   EHRLaneAccessCharacterstic lane_access_characterstic_;     // 车道可行驶时间
            // for(size_t k=0;k < ehrlane.lane_access_characterstic_.valid_period_.size();++k){

            // }
            // std::vector<EHRLaneConnectivity> in_lane_conn_list_;       // 进车道拓扑关系
            for(size_t k=0;k < ehrlane.in_lane_conn_list_.size();++k){
                EHRLaneConnectivity & laneconnectivity=ehrlane.in_lane_conn_list_[k];
                HDM_WRITE(file_path, false, "\t\t\tinlaneconnectivity[%lu]=section_id_[%llu],path_id_[%u],lane_id_[%hhu]\n",
                    k,
                    laneconnectivity.section_id_,
                    laneconnectivity.path_id_,
                    laneconnectivity.lane_id_);
            }
            //   std::vector<EHRLaneConnectivity> out_lane_conn_list_;      // 出车道拓扑关系
            for(size_t k=0;k < ehrlane.out_lane_conn_list_.size();++k){
                EHRLaneConnectivity & laneconnectivity=ehrlane.out_lane_conn_list_[k];
                HDM_WRITE(file_path, false, "\t\t\toutlaneconnectivity[%lu]=section_id_[%llu],path_id_[%u],lane_id_[%hhu]\n",
                    k,
                    laneconnectivity.section_id_,
                    laneconnectivity.path_id_,
                    laneconnectivity.lane_id_);
            }
            //   std::vector<EHRLineSection> line_section_list_;            // 车道线
            for(size_t k=0;k < ehrlane.line_section_list_.size();++k){
                EHRLineSection & ehrlinesection=ehrlane.line_section_list_[k];
                HDM_WRITE(file_path, false, "\t\t\tehrlinesection[%lu]=LINE_IN_LANE[%d],color[%d],line_marking[%d],offset[%u =>%u],line_marking_width[%u] \n",
                    k,
                    (int32_t)ehrlinesection.line_in_lane_,
                    (int32_t)ehrlinesection.line_marking_colour_,
                    (int32_t)ehrlinesection.line_marking_type_,
                    ehrlinesection.start_offset_,
                    ehrlinesection.end_offset_,
                    ehrlinesection.line_marking_width_);
                    HDM_WRITE(file_path, false, "\t\t\t\tpoint_list_=");
                    for(uint32_t m = 0; m < ehrlinesection.point_list_.size(); ++m){
                        HDM_WRITE(file_path, false,  "[%u:%f,%f,%f], ", m, ehrlinesection.point_list_[m].x, ehrlinesection.point_list_[m].y, ehrlinesection.point_list_[m].z);
                    }
                    HDM_WRITE(file_path, false, "\n");
            }
            //   std::vector<EHRSpeedLimitChangePoint>
            //       max_speed_limit_change_point_list_;                    // 最高限速值变化点
            //   std::vector<EHRSpeedLimitChangePoint>
            //       min_speed_limit_change_point_list_;                    // 最低限速值变化点

            //   std::vector<EHRLaneSpecialBoundary> special_bounday_list_; // 特殊路沿
            //   std::vector<EHRCHPoint> ch_point_list_;                    // 坡度航向曲率信息
            for(size_t k=0;k < ehrlane.ch_point_list_.size();++k){
                EHRCHPoint & CEHRpoint =  ehrlane.ch_point_list_[k];
                HDM_WRITE(file_path, false, "\t\t\tCEHRpoint[%lu]=point_id_[%u],offset_[%u],heading_[%f],curvature_[%f] \n",
                    k,
                    CEHRpoint.point_id_,
                    CEHRpoint.offset_,
                    CEHRpoint.heading_,
                    CEHRpoint.curvature_);
            }
        }


    }
    HDM_WRITE(file_path, true, "%s", "");   
    return true;
}

bool CHdmEhrOutput::OutputSection(const char_t *file_path, const zone::common::EHRSection & section)
{    
    std::string str_date_time = CHdmDateTime::GetFormatDateTimeString();
    HDM_WRITE(file_path, true,  "date_time[%s]***********Section**************\n", str_date_time.c_str());
    HDM_WRITE(file_path, false,  "\tsection_id_[%llu],section_index_[%u], path_id_[%u], offset[%u=>%u], frc_[%u], fow_[%u], part_of_route_[%d],total_lane_num_[%hhu], is_complex_intersection_[%d]\n", 
        section.section_id_,
        section.section_index_, 
        section.path_id_, 
        section.start_offset_, 
        section.end_offset_, 
        section.frc_, 
        section.fow_, 
        section.part_of_route_,
        section.total_lane_num_, 
        section.is_complex_intersection_);
    //   std::vector<EHRTrafficSign> traffic_sign_list_;            // 交通标志牌
    for(size_t j = 0; j < section.traffic_sign_list_.size(); ++j){
       const EHRTrafficSign & trafficsign=section.traffic_sign_list_[j];
        HDM_WRITE(file_path, false,  "\t\trafficsign[%lu]=traffic_sign_type[%d],offset[%u=>%u]\n", 
                j,
                (int32_t)trafficsign.type_ ,
                trafficsign.start_offset_, 
                trafficsign.end_offset_);
    }
    //   std::vector<EHRTrafficLight> traffic_light_list_;          // 交通灯
    for(size_t j = 0; j < section.traffic_light_list_.size(); ++j){
        const EHRTrafficLight & traffic_light=section.traffic_light_list_[j];
        HDM_WRITE(file_path, false,  "\t\traffic_light[%lu]=traffic_light_type[%d],offset[%u=>%u],coord_[%lf,%lf,%lf],bouding_box_vector_[%lf,%lf,%lf] \n", 
                j,
                (int32_t)traffic_light.type_ ,
                traffic_light.start_offset_, 
                traffic_light.end_offset_,
                traffic_light.coord_.x,
                traffic_light.coord_.y,
                traffic_light.coord_.z,
                traffic_light.bouding_box_vector_.x,
                traffic_light.bouding_box_vector_.y,
                traffic_light.bouding_box_vector_.z);
    }
    //   std::vector<EHRSpecialSituation> special_situation_list_;  // 特殊位置信息
    for(size_t j = 0; j < section.special_situation_list_.size(); ++j){
        const EHRSpecialSituation & specialsituation=section.special_situation_list_[j];
        HDM_WRITE(file_path, false,  "\t\tspecialsituation[%lu]=special_situation_type[%d],offset[%u=>%u]\n",
                j,
                specialsituation.type_,
                specialsituation.start_offset_,
                specialsituation.end_offset_);
    }
    //   std::vector<EHRSubPath> subpath_list_;                     // 分岔信息
    for (size_t j = 0; j < section.subpath_list_.size(); ++j) {
        const EHRSubPath & subpath = section.subpath_list_[j];
        HDM_WRITE(file_path, false,  "\t\tSubpath[%lu]=subpath_id_[%u], turn_angle_[%f], offset[%u=>%u]\n", 
                j, 
                subpath.subpath_id_, 
                subpath.turn_angle_, 
                subpath.start_offset_, 
                subpath.end_offset_);
    }
    //   std::vector<EHRPointHeading> point_heading_list_;          // 航向角信息
    // for (size_t j = 0; j < section.point_heading_list_.size(); ++j) {
    //     EHRPointHeading & pointheading=section.point_heading_list_[j];
    //     HDM_WRITE(file_path, false,  "\t\tpointheading[%u]=point_id[%u] ,offset[%u],heading_[%f],curvature_[%f],slope_[%f],super_elevation_[%d]\n",
    //             j,
    //             pointheading.point_id_,
    //             pointheading.offset_,
    //             pointheading.heading_,
    //             pointheading.curvature_,
    //             pointheading.slope_,
    //             (int32_t)pointheading.super_elevation_);
    // }
    //   std::vector<EHRLane> lane_list_;                           // 车道信息
    for (size_t j = 0; j < section.lane_list_.size(); ++j) {
        const EHRLane &  ehrlane=section.lane_list_[j];
        HDM_WRITE(file_path, false,  "\t\tlane[%lu]=lane_id[%hhu],lane_index[%hhu],section_id[%llu],path_id[%u],speed[%hhu =>%hhu],lane_direction[%d],lane_type[%d],lanewith[%u,%u,%u,%u.%u],lanetransition_type[%d],lane_access_characterstic_[%d,%d],speed_limit_change_point_list[%lu,%lu]\n",
                j,
                ehrlane.lane_id_,
                ehrlane.lane_index_,
                ehrlane.section_id_,
                ehrlane.path_id_,
                ehrlane.min_speed_limit_,
                ehrlane.max_speed_limit_,
                (int32_t)ehrlane.direction_,
                (int32_t)ehrlane.lane_type_,
                ehrlane.lane_width_.trans_width_offset_,
                ehrlane.lane_width_.trans_width_end_offset_,
                ehrlane.lane_width_.start_width_,
                ehrlane.lane_width_.end_width_,
                ehrlane.lane_width_.trans_width_,
                (int32_t)ehrlane.lane_transition_,
                (int32_t)ehrlane.lane_access_characterstic_.vehicle_type_,
                ehrlane.lane_access_characterstic_.has_access_characterstic_,
                ehrlane.max_speed_limit_change_point_list_.size(),
                ehrlane.min_speed_limit_change_point_list_.size()
        );
        //   EHRLaneAccessCharacterstic lane_access_characterstic_;     // 车道可行驶时间
        // for(size_t k=0;k < ehrlane.lane_access_characterstic_.valid_period_.size();++k){

        // }
        // std::vector<EHRLaneConnectivity> in_lane_conn_list_;       // 进车道拓扑关系
        for(size_t k=0;k < ehrlane.in_lane_conn_list_.size();++k){
            const EHRLaneConnectivity & laneconnectivity=ehrlane.in_lane_conn_list_[k];
            HDM_WRITE(file_path, false, "\t\t\tinlaneconnectivity[%lu]=section_id_[%llu],path_id_[%u],lane_id_[%hhu]\n",
                k,
                laneconnectivity.section_id_,
                laneconnectivity.path_id_,
                laneconnectivity.lane_id_);
        }
        //   std::vector<EHRLaneConnectivity> out_lane_conn_list_;      // 出车道拓扑关系
        for(size_t k=0;k < ehrlane.out_lane_conn_list_.size();++k){
            const EHRLaneConnectivity & laneconnectivity=ehrlane.out_lane_conn_list_[k];
            HDM_WRITE(file_path, false, "\t\t\toutlaneconnectivity[%lu]=section_id_[%llu],path_id_[%u],lane_id_[%hhu]\n",
                k,
                laneconnectivity.section_id_,
                laneconnectivity.path_id_,
                laneconnectivity.lane_id_);
        }
        //   std::vector<EHRLineSection> line_section_list_;            // 车道线
        for(size_t k=0;k < ehrlane.line_section_list_.size();++k){
            const EHRLineSection & ehrlinesection=ehrlane.line_section_list_[k];
            HDM_WRITE(file_path, false, "\t\t\tehrlinesection[%lu]=LINE_IN_LANE[%d],color[%d],line_marking[%d],offset[%u =>%u],line_marking_width[%u] \n",
                k,
                (int32_t)ehrlinesection.line_in_lane_,
                (int32_t)ehrlinesection.line_marking_colour_,
                (int32_t)ehrlinesection.line_marking_type_,
                ehrlinesection.start_offset_,
                ehrlinesection.end_offset_,
                ehrlinesection.line_marking_width_);
                HDM_WRITE(file_path, false, "\t\t\t\tpoint_list_=");
                for(uint32_t m = 0; m < ehrlinesection.point_list_.size(); ++m){
                    HDM_WRITE(file_path, false,  "[%u:%f,%f,%f], ", m, ehrlinesection.point_list_[m].x, ehrlinesection.point_list_[m].y, ehrlinesection.point_list_[m].z);
                }
                HDM_WRITE(file_path, false, "\n");
        }
        //   std::vector<EHRSpeedLimitChangePoint>
        //       max_speed_limit_change_point_list_;                    // 最高限速值变化点
        //   std::vector<EHRSpeedLimitChangePoint>
        //       min_speed_limit_change_point_list_;                    // 最低限速值变化点

        //   std::vector<EHRLaneSpecialBoundary> special_bounday_list_; // 特殊路沿
        //   std::vector<EHRCHPoint> ch_point_list_;                    // 坡度航向曲率信息
        // for(size_t k=0;k < ehrlane.ch_point_list_.size();++k){
        //     EHRCHPoint & CEHRpoint =  ehrlane.ch_point_list_[k];
        //     HDM_WRITE(file_path, false, "\t\t\tCEHRpoint[%u]=point_id_[%u],offset_[%u],heading_[%f],curvature_[%f] \n",
        //         k,
        //         CEHRpoint.point_id_,
        //         CEHRpoint.offset_,
        //         CEHRpoint.heading_,
        //         CEHRpoint.curvature_);
        // }
    } 
    HDM_WRITE(file_path, true, "%s", "");
    return true;
}

void CHdmEhrOutput::OutputPtsOffset(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\tpts_offset_num_[%u]\n", to_em_data->pts_offset_num_);
    HDM_WRITE(file_path, false, "\tpts_offset_=");
    for (uint32_t i = 0; i < to_em_data->pts_offset_num_; ++i)
    {
        HDM_WRITE(file_path, false, "[%u:%d], ", i, to_em_data->pts_offset_[i]);
    }
}

void CHdmEhrOutput::OutputMapPts(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\tmap_pts_num_[%u]\n", to_em_data->map_pts_num_);
    HDM_WRITE(file_path, false, "\tmap_pts_=");
    for (uint32_t i = 0; i < to_em_data->map_pts_num_; ++i)
    {
        HDM_WRITE(file_path, false, "[%u:%f,%f], ", i, to_em_data->map_pts_[i].x, to_em_data->map_pts_[i].y);
    }
}

void CHdmEhrOutput::OutputIsOnRout(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\tis_on_rout_num_[%u]\n", to_em_data->is_on_rout_num_);
    HDM_WRITE(file_path, false, "\tis_on_rout_=");
    for (uint32_t i = 0; i < to_em_data->is_on_rout_num_; ++i)
    {
        HDM_WRITE(file_path, false, "(%u:%d,%d,%d), ", i, to_em_data->is_on_rout_[i].start_offset_, to_em_data->is_on_rout_[i].end_offset_, to_em_data->is_on_rout_[i].is_on_route_);
    }
}

void CHdmEhrOutput::OutputIsInIntersection(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\tis_in_intersection_num_[%u]\n", to_em_data->is_in_intersection_num_);
    HDM_WRITE(file_path, false, "\tis_in_intersection_=");
    for (uint32_t i = 0; i < to_em_data->is_in_intersection_num_; ++i)
    {
        HDM_WRITE(file_path, false, "(%u:%d,%d,%d), ", i, to_em_data->is_in_intersection_[i].start_offset_, to_em_data->is_in_intersection_[i].end_offset_, to_em_data->is_in_intersection_[i].is_in_intersection_);
    }
}

void CHdmEhrOutput::OutputSpeedLimit(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\tspeed_limit_num_[%u]\n", to_em_data->speed_limit_num_);
    HDM_WRITE(file_path, false, "\tspeed_limit_=");
    for (uint32_t i = 0; i < to_em_data->speed_limit_num_; ++i)
    {
        HDM_WRITE(file_path, false, "(%u:%d,%d,%u,%u), ", i, to_em_data->speed_limit_[i].start_offset_, to_em_data->speed_limit_[i].end_offset_, to_em_data->speed_limit_[i].min_speed_limit_, to_em_data->speed_limit_[i].max_speed_limit_);
    }
}

void CHdmEhrOutput::OutputLaneType(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\tlane_type_num_[%u]\n", to_em_data->lane_type_num_);
    HDM_WRITE(file_path, false, "\tlane_type_=");
    for (uint32_t i = 0; i < to_em_data->lane_type_num_; ++i)
    {
        HDM_WRITE(file_path, false, "(%u:%d,%d,%u), ", i, to_em_data->lane_type_[i].start_offset_, to_em_data->lane_type_[i].end_offset_, to_em_data->lane_type_[i].lane_type_);
    }
}

void CHdmEhrOutput::OutputLaneSlop(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\tlane_slop_num_[%u]\n", to_em_data->lane_slop_num_);
    HDM_WRITE(file_path, false, "\tlane_slop_=");
    for (uint32_t i = 0; i < to_em_data->lane_slop_num_; ++i)
    {
        HDM_WRITE(file_path, false, "(%u:%d,%d,%f,%f), ", i, to_em_data->lane_slop_[i].start_offset_, to_em_data->lane_slop_[i].end_offset_, to_em_data->lane_slop_[i].start_slop_, to_em_data->lane_slop_[i].end_slop_);
    }
}

void CHdmEhrOutput::OutputSuperElevation(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\tsuper_elevation_num_[%u]\n", to_em_data->super_elevation_num_);
    HDM_WRITE(file_path, false, "\tsuper_elevation_=");
    for (uint32_t i = 0; i < to_em_data->super_elevation_num_; ++i)
    {
        HDM_WRITE(file_path, false, "(%u:%d,%d,%d), ", i, to_em_data->super_elevation_[i].start_offset_, to_em_data->super_elevation_[i].end_offset_, (int32_t)to_em_data->super_elevation_[i].super_elevation_class_);
    }
}

void CHdmEhrOutput::OutputLaneCurvature(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\tlane_curvature_num_[%u]\n", to_em_data->lane_curvature_num_);
    HDM_WRITE(file_path, false, "\tlane_curvature_");
    for (uint32_t i = 0; i < to_em_data->lane_curvature_num_; ++i)
    {
        HDM_WRITE(file_path, false, "(%u:%d,%d,%f,%f), ", i, to_em_data->lane_curvature_[i].start_offset_, to_em_data->lane_curvature_[i].end_offset_, to_em_data->lane_curvature_[i].start_curvature_, to_em_data->lane_curvature_[i].end_curvature_);
    }
}

void CHdmEhrOutput::OutputLaneHeading(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\tlane_heading_num_[%u]\n", to_em_data->lane_heading_num_);
    HDM_WRITE(file_path, false, "\tlane_heading_=");
    for (uint32_t i = 0; i < to_em_data->lane_heading_num_; ++i)
    {
        HDM_WRITE(file_path, false, "(%u:%d,%d,%f,%f), ", i, to_em_data->lane_heading_[i].start_offset_, to_em_data->lane_heading_[i].end_offset_, to_em_data->lane_heading_[i].start_heading_, to_em_data->lane_heading_[i].end_heading_);
    }
}

void CHdmEhrOutput::OutputLaneWidth(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\tlane_width_num_[%u]\n", to_em_data->lane_width_num_);
    HDM_WRITE(file_path, false, "\tlane_width_=");
    for (uint32_t i = 0; i < to_em_data->lane_width_num_; ++i)
    {
        HDM_WRITE(file_path, false, "(%u:%d,%d,%u,%u), ", i, to_em_data->lane_width_[i].start_offset_, to_em_data->lane_width_[i].end_offset_, to_em_data->lane_width_[i].start_width_, to_em_data->lane_width_[i].end_width_);
    }
}

void CHdmEhrOutput::OutputLaneTransitionDirection(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\tlane_transition_direction_num_[%u]\n", to_em_data->lane_transition_direction_num_);
    HDM_WRITE(file_path, false, "\tlane_transition_direction_=");
    for (uint32_t i = 0; i < to_em_data->lane_transition_direction_num_; ++i)
    {
        HDM_WRITE(file_path, false, "(%u:%d,%d,%d), ", i, to_em_data->lane_transition_direction_[i].start_offset_, to_em_data->lane_transition_direction_[i].end_offset_, (int32_t)to_em_data->lane_transition_direction_[i].lane_transition_direction_);
    }
}

void CHdmEhrOutput::OutputLaneMarking(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\tlane_marking_num_[%u]\n", to_em_data->lane_marking_num_);
    HDM_WRITE(file_path, false, "\tlane_marking_=");
    for (uint32_t i = 0; i < to_em_data->lane_marking_num_; ++i)
    {
        HDM_WRITE(file_path, false, "(%u:%d,%d), ", i, to_em_data->lane_marking_[i].offset_, (int32_t)to_em_data->lane_marking_[i].lane_marking_type_);
    }
}

void CHdmEhrOutput::OutputSpecialSituation(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\tspecial_situation_num_[%u]\n", to_em_data->special_situation_num_);
    HDM_WRITE(file_path, false, "\tspecial_situation_");
    for (uint32_t i = 0; i < to_em_data->special_situation_num_; ++i)
    {
        HDM_WRITE(file_path, false, "(%u:%d,%d,%d), ", i, to_em_data->special_situation_[i].start_offset_, to_em_data->special_situation_[i].end_offset_, (int32_t)to_em_data->special_situation_[i].special_situation_type_);
    }
}

void CHdmEhrOutput::OutputLaneBoundarySegment(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\tlane_boundary_segment_num_[%u]\n", to_em_data->lane_boundary_segment_num_);
    HDM_WRITE(file_path, false, "\tlane_boundary_segment_=");
    for (uint32_t i = 0; i < to_em_data->lane_boundary_segment_num_; ++i)
    {
        HDM_WRITE(file_path, false, "[%u:%d, %d, %d, %d], ", i, to_em_data->lane_boundary_segment_[i].start_offset_, to_em_data->lane_boundary_segment_[i].end_offset_, (int32_t)to_em_data->lane_boundary_segment_[i].lane_boudary_color_, (int32_t)to_em_data->lane_boundary_segment_[i].lane_boudary_type_);
    }
}

void CHdmEhrOutput::OutputRoadEdgeSegment(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\troad_edge_segment_num_[%u]\n", to_em_data->road_edge_segment_num_);
    HDM_WRITE(file_path, false, "\troad_edge_segment_=");
    for (uint32_t i = 0; i < to_em_data->road_edge_segment_num_; ++i)
    {
        HDM_WRITE(file_path, false, "[%u:%d,%d,%d], ", i, to_em_data->road_edge_segment_[i].start_offset_, to_em_data->road_edge_segment_[i].end_offset_, (int32_t)to_em_data->road_edge_segment_[i].road_edge_type_);
    }
}

void CHdmEhrOutput::OutputLaneElements(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\tlane_elements_num_[%u]\n", to_em_data->lane_elements_num_);
    for (uint32_t i = 0; i < to_em_data->lane_elements_num_; ++i)
    {
        HDM_WRITE(file_path, false, "\tlane_elements_(%u:%d,%d), ", i, to_em_data->lane_elements_[i].route_left_dist_, to_em_data->lane_elements_[i].is_dest_lane_element_);
        HDM_WRITE(file_path, false, "left_boundary_[%u:%d,(%d,%d),(%d,%d),(%d,%d)], ", i, to_em_data->lane_elements_[i].left_boundary_.existence_,
                  to_em_data->lane_elements_[i].left_boundary_.pts_offset_range_.start_index_, to_em_data->lane_elements_[i].left_boundary_.pts_offset_range_.total_number_,
                  to_em_data->lane_elements_[i].left_boundary_.map_pts_range_.start_index_, to_em_data->lane_elements_[i].left_boundary_.map_pts_range_.total_number_,
                  to_em_data->lane_elements_[i].left_boundary_.lane_boundary_segment_range_.start_index_, to_em_data->lane_elements_[i].left_boundary_.lane_boundary_segment_range_.total_number_);

        HDM_WRITE(file_path, false, "right_boundary_[%u:%d,(%d,%d),(%d,%d),(%d,%d)], ", i, to_em_data->lane_elements_[i].right_boundary_.existence_,
                  to_em_data->lane_elements_[i].right_boundary_.pts_offset_range_.start_index_, to_em_data->lane_elements_[i].right_boundary_.pts_offset_range_.total_number_,
                  to_em_data->lane_elements_[i].right_boundary_.map_pts_range_.start_index_, to_em_data->lane_elements_[i].right_boundary_.map_pts_range_.total_number_,
                  to_em_data->lane_elements_[i].right_boundary_.lane_boundary_segment_range_.start_index_, to_em_data->lane_elements_[i].right_boundary_.lane_boundary_segment_range_.total_number_);
        HDM_WRITE(file_path, false, "left_road_edge_[%u:%d,(%d,%d),(%d,%d),(%d,%d)], ", i, to_em_data->lane_elements_[i].left_road_edge_.existence_,
                  to_em_data->lane_elements_[i].left_road_edge_.pts_offset_range_.start_index_, to_em_data->lane_elements_[i].left_road_edge_.pts_offset_range_.total_number_,
                  to_em_data->lane_elements_[i].left_road_edge_.map_pts_range_.start_index_, to_em_data->lane_elements_[i].left_road_edge_.map_pts_range_.total_number_,
                  to_em_data->lane_elements_[i].left_road_edge_.road_edge_segment_range_.start_index_, to_em_data->lane_elements_[i].left_road_edge_.road_edge_segment_range_.total_number_);
        HDM_WRITE(file_path, false, "right_road_edge_[%u：%d,(%d,%d),(%d,%d),(%d,%d)], ", i, to_em_data->lane_elements_[i].right_road_edge_.existence_,
                  to_em_data->lane_elements_[i].right_road_edge_.pts_offset_range_.start_index_, to_em_data->lane_elements_[i].right_road_edge_.pts_offset_range_.total_number_,
                  to_em_data->lane_elements_[i].right_road_edge_.map_pts_range_.start_index_, to_em_data->lane_elements_[i].right_road_edge_.map_pts_range_.total_number_,
                  to_em_data->lane_elements_[i].right_road_edge_.road_edge_segment_range_.start_index_, to_em_data->lane_elements_[i].right_road_edge_.road_edge_segment_range_.total_number_);
        HDM_WRITE(file_path, false, "center_line_[%u:(%d,%d),(%d,%d),(%d,%d),(%d,%d),(%d,%d),(%d,%d),(%d,%d),(%d,%d),(%d,%d),(%d,%d),(%d,%d),(%d,%d),(%d,%d),(%d,%d)]\n", i,
                  to_em_data->lane_elements_[i].center_line_.pts_offset_range_.start_index_, to_em_data->lane_elements_[i].center_line_.pts_offset_range_.total_number_,
                  to_em_data->lane_elements_[i].center_line_.map_pts_range_.start_index_, to_em_data->lane_elements_[i].center_line_.map_pts_range_.total_number_,
                  to_em_data->lane_elements_[i].center_line_.is_on_rout_range_.start_index_, to_em_data->lane_elements_[i].center_line_.is_on_rout_range_.total_number_,
                  to_em_data->lane_elements_[i].center_line_.is_in_intersection_range_.start_index_, to_em_data->lane_elements_[i].center_line_.is_in_intersection_range_.total_number_,
                  to_em_data->lane_elements_[i].center_line_.speed_limit_range_.start_index_, to_em_data->lane_elements_[i].center_line_.speed_limit_range_.total_number_,
                  to_em_data->lane_elements_[i].center_line_.lane_type_range_.start_index_, to_em_data->lane_elements_[i].center_line_.lane_type_range_.total_number_,
                  to_em_data->lane_elements_[i].center_line_.lane_slop_range_.start_index_, to_em_data->lane_elements_[i].center_line_.lane_slop_range_.total_number_,
                  to_em_data->lane_elements_[i].center_line_.super_elevation_range_.start_index_, to_em_data->lane_elements_[i].center_line_.super_elevation_range_.total_number_,
                  to_em_data->lane_elements_[i].center_line_.lane_curvature_range_.start_index_, to_em_data->lane_elements_[i].center_line_.lane_curvature_range_.total_number_,
                  to_em_data->lane_elements_[i].center_line_.lane_heading_range_.start_index_, to_em_data->lane_elements_[i].center_line_.lane_heading_range_.total_number_,
                  to_em_data->lane_elements_[i].center_line_.lane_width_range_.start_index_, to_em_data->lane_elements_[i].center_line_.lane_width_range_.total_number_,
                  to_em_data->lane_elements_[i].center_line_.lane_transition_direction_range_.start_index_, to_em_data->lane_elements_[i].center_line_.lane_transition_direction_range_.total_number_,
                  to_em_data->lane_elements_[i].center_line_.lane_marking_range_.start_index_, to_em_data->lane_elements_[i].center_line_.lane_marking_range_.total_number_,
                  to_em_data->lane_elements_[i].center_line_.special_situation_range_.start_index_, to_em_data->lane_elements_[i].center_line_.special_situation_range_.total_number_);
    }
}

void CHdmEhrOutput::OutputGuidePoint2(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\n\tguide_point_num_[%u]\n", to_em_data->guide_point_num_);
    HDM_WRITE(file_path, false, "\tguide_point_=");
    for (uint32_t i = 0; i < to_em_data->guide_point_num_; ++i)
    {
        HDM_WRITE(file_path, false, "(%u:%d,%d,%d,%f,%d,(%lf,%lf,%lf)), ", i, to_em_data->guide_point_[i].start_offset_,
                  to_em_data->guide_point_[i].end_offset_, to_em_data->guide_point_[i].is_on_route_, to_em_data->guide_point_[i].turn_angle_,
                  (int32_t)to_em_data->guide_point_[i].guide_point_type_, to_em_data->guide_point_[i].position_.x,
                  to_em_data->guide_point_[i].position_.y, to_em_data->guide_point_[i].position_.z);
    }
}

void CHdmEhrOutput::OutputTrafficLight(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    // HDM_WRITE(file_path, false, "\n\ttraffic_light_num_[%u]\n", to_em_data->traffic_light_num_);
    HDM_WRITE(file_path, false, "\ttraffic_light_");
    // for (uint32_t i = 0; i < to_em_data->traffic_light_num_; ++i)
    // {
    //     HDM_WRITE(file_path, false, "[%u:%d], ", i, to_em_data->traffic_light_[i].valid_);
    // }
}

void CHdmEhrOutput::OutputLaneData(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    HDM_WRITE(file_path, false, "\tlane_data_num_[%u]\n", to_em_data->lane_data_num_);
    HDM_WRITE(file_path, false, "\tlane_data_=");
    for (uint32_t i = 0; i < to_em_data->lane_data_num_; ++i)
    {
        HDM_WRITE(file_path, false, "(%u:%d,%d), ", i, to_em_data->lane_data_[i].lane_elements_range_.start_index_, to_em_data->lane_data_[i].lane_elements_range_.total_number_);
    }
}

bool CHdmEhrOutput::OutputToEmData(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    std::string str_date_time = CHdmDateTime::GetFormatDateTimeString();
    if (nullptr == to_em_data) {
        HDM_WRITE(file_path, true, "\ndate_time[%s]***********to_em_data[%p]**************\n", str_date_time.c_str(), to_em_data.get());
        return false;
    }
    HDM_WRITE(file_path, false, "\ndate_time[%s]******to_em_data_id_[%u]***section_list_data_id_[%u]***return_code_[0x%x-%x-%x-%x]*********\n", str_date_time.c_str(), to_em_data->to_em_data_id_, to_em_data->section_list_data_id_,
        to_em_data->return_code_[0], to_em_data->return_code_[1], to_em_data->return_code_[2], to_em_data->return_code_[3]);
    HDM_WRITE(file_path, false, "\tcar_position_=abs_coord_[%lf,%lf,%lf], timestamp_[%lf], confidence_[%f], heading_[%f]\n", to_em_data->car_position_.abs_coord_.x, to_em_data->car_position_.abs_coord_.y, to_em_data->car_position_.abs_coord_.z,
        to_em_data->car_position_.timestamp_, to_em_data->car_position_.confidence_, to_em_data->car_position_.heading_);
    HDM_WRITE(file_path, false, "\tgeofence_data_,(%d,%d,%d)",to_em_data->geofence_data_.is_in_map_,to_em_data->geofence_data_.is_on_intersection_,to_em_data->geofence_data_.is_on_ramp_);
    HDM_WRITE(file_path, false, "\n\thost_lane_idx_[%d]\n",to_em_data->host_lane_idx_);

    OutputPtsOffset(file_path, to_em_data);
    OutputMapPts(file_path, to_em_data);
    OutputIsOnRout(file_path, to_em_data);
    OutputIsInIntersection(file_path, to_em_data);
    OutputSpeedLimit(file_path, to_em_data);
    OutputLaneType(file_path, to_em_data);
    OutputLaneSlop(file_path, to_em_data);
    OutputSuperElevation(file_path, to_em_data);
    OutputLaneCurvature(file_path, to_em_data);
    OutputLaneHeading(file_path, to_em_data);
    OutputLaneWidth(file_path, to_em_data);
    OutputLaneTransitionDirection(file_path, to_em_data);
    OutputLaneMarking(file_path, to_em_data);
    OutputSpecialSituation(file_path, to_em_data);
    OutputLaneBoundarySegment(file_path, to_em_data);
    OutputRoadEdgeSegment(file_path, to_em_data);
    OutputLaneElements(file_path, to_em_data);
    OutputGuidePoint2(file_path, to_em_data);
    OutputTrafficLight(file_path, to_em_data);
    OutputLaneData(file_path, to_em_data);

    HDM_WRITE(file_path, true, "%s", "");
    return true;
}


bool CHdmEhrOutput::OutputGuidePoint(const char_t *file_path, const std::shared_ptr<zone::common::EhrToEmData> &to_em_data)
{
    for(uint32_t i=0;i < to_em_data->guide_point_num_; ++i ){
        Point wgs84_point;

        CHdmGeometry::Gcj02ToWgs84(to_em_data->guide_point_[i].position_.x, to_em_data->guide_point_[i].position_.y, wgs84_point.x, wgs84_point.y);

        HDM_WRITE(file_path, false, "%u,%u,%d,%d,%d,%f,%d,%lf,%lf,%lf\n",i,to_em_data->to_em_data_id_, to_em_data->guide_point_[i].start_offset_,
            to_em_data->guide_point_[i].end_offset_,to_em_data->guide_point_[i].is_on_route_,to_em_data->guide_point_[i].turn_angle_,
            (int32_t)to_em_data->guide_point_[i].guide_point_type_, to_em_data->guide_point_[i].position_.x, 
            to_em_data->guide_point_[i].position_.y, to_em_data->guide_point_[i].position_.z);        
    }    
    HDM_WRITE(file_path, true, "%s", "");
    return true;
}


bool CHdmEhrOutput::OutputEhpInputInfo(const char_t *file_path, const std::shared_ptr<zone::common::EhpInputInfo> &ehp_input_info)
{
    std::string str_date_time = CHdmDateTime::GetFormatDateTimeString();
    if (nullptr == ehp_input_info) {
        HDM_WRITE(file_path, true, "\ndate_time[%s]***********ehp_input_info[%p]**************\n", str_date_time.c_str(), ehp_input_info.get());
        return false;
    }
    HDM_WRITE(file_path, false, "\tgnss_time: [%f]\n" , ehp_input_info->gnss_time);
    HDM_WRITE(file_path, false, "\tcar_point.x: [%f]\n" , ehp_input_info->car_point.x);
    HDM_WRITE(file_path, false, "\tcar_point.y: [%f]\n" , ehp_input_info->car_point.y);		
    HDM_WRITE(file_path, false, "\theading: [%f]\n" , ehp_input_info->heading);
    HDM_WRITE(file_path, false, "\tpitch: [%f]\n" , ehp_input_info->pitch);
    HDM_WRITE(file_path, false, "\tdir_link_geo_id: [%llu]\n" , ehp_input_info->dir_link_geo_id);
    HDM_WRITE(file_path, false, "\tlinkGeoMeshId: [%u]\n" , ehp_input_info->linkGeoMeshId);
    HDM_WRITE(file_path, false, "\tlinkGeoRegion: [%u]\n" , ehp_input_info->linkGeoRegion);
    HDM_WRITE(file_path, false, "\tlinkGeoVersion: [%u]\n" , ehp_input_info->linkGeoVersion);	
    HDM_WRITE(file_path, false, "\tlinkGeoPos: [%u]\n" , ehp_input_info->linkGeoPos);														
    HDM_WRITE(file_path, false, "\tlaneGroupMeshId: [%u]\n" , ehp_input_info->laneGroupMeshId);														
    HDM_WRITE(file_path, false, "\tlaneGroupRegion: [%u]\n" , ehp_input_info->laneGroupRegion);														
    HDM_WRITE(file_path, false, "\tlaneGroupVersion: [%u]\n" , ehp_input_info->laneGroupVersion);														
    HDM_WRITE(file_path, false, "\tlaneGroupId: [%llu]\n" , ehp_input_info->laneGroupId);														
    HDM_WRITE(file_path, false, "\tlaneNo: [%u]\n" , ehp_input_info->laneNo);														
    HDM_WRITE(file_path, false, "\tlink_foot_point.x: [%f]\n" , ehp_input_info->link_foot_point.x);														
    HDM_WRITE(file_path, false, "\tlink_foot_point.y: [%f]\n" , ehp_input_info->link_foot_point.y);														
    HDM_WRITE(file_path, false, "\tlink_geo_num: [%u]\n" , ehp_input_info->link_geo_num);														
    HDM_WRITE(file_path, false, "\tlane_foot_point.x: [%f]\n" , ehp_input_info->lane_foot_point.x);														
    HDM_WRITE(file_path, false, "\tlane_foot_point.y: [%f]\n" , ehp_input_info->lane_foot_point.y);														
    HDM_WRITE(file_path, false, "\tlane_geo_num: [%u]\n" , ehp_input_info->lane_geo_num);														
    HDM_WRITE(file_path, false, "\tleft_lane_boundary_angle: [%f]\n" , ehp_input_info->left_lane_boundary_angle);														
    HDM_WRITE(file_path, false, "\tright_lane_boundary_dis: [%f]\n" , ehp_input_info->right_lane_boundary_dis);														
    HDM_WRITE(file_path, false, "\tleft_lane_boundary_dis: [%f]\n" , ehp_input_info->left_lane_boundary_dis);														
    HDM_WRITE(file_path, false, "\tleft_lane_boundary_dis_conf: [%f]\n" , ehp_input_info->left_lane_boundary_dis_conf);														
    HDM_WRITE(file_path, false, "\tright_lane_boundary_dis_conf: [%f]\n" , ehp_input_info->right_lane_boundary_dis_conf);														
    HDM_WRITE(file_path, false, "\tright_lane_boundary_angle_conf: [%f]\n" , ehp_input_info->right_lane_boundary_angle_conf);														
    HDM_WRITE(file_path, false, "\tleft_lane_boundary_angle_conf: [%f]\n" , ehp_input_info->left_lane_boundary_angle_conf);														
    HDM_WRITE(file_path, false, "\thdm_status: [%u]\n" , ehp_input_info->hdm_status);														
    HDM_WRITE(file_path, false, "\ton_db_road_status: [%u]\n" , ehp_input_info->on_db_road_status);														
    HDM_WRITE(file_path, false, "\tupdate_frequency: [%u]\n" , ehp_input_info->update_frequency);														
    HDM_WRITE(file_path, false, "\taccuracy: [%f]\n" , ehp_input_info->accuracy);														
    HDM_WRITE(file_path, false, "\tlateral_accuracy: [%f]\n" , ehp_input_info->lateral_accuracy);														
    HDM_WRITE(file_path, false, "\tlongitudinal_accuracy: [%f]\n" , ehp_input_info->longitudinal_accuracy);														
    HDM_WRITE(file_path, false, "\tspeed_accuracy: [%f]\n" , ehp_input_info->speed_accuracy);														
    HDM_WRITE(file_path, false, "\tloc_side: [%d]\n" , ehp_input_info->loc_side);														
    HDM_WRITE(file_path, true, "\tinner_state: [%u]\n" , ehp_input_info->inner_state);	
    return true;
}

}
/* EOF */