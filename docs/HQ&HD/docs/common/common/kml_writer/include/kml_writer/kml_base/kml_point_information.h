/**
 * kml_point_information.h
 *
 * Kml point information
 *
 * Created by Cheng Liu on 09/22/2021
 * Copyright 2021 Z-ONE Inc. All rights reserved.
 *
 */

#ifndef _KML_POINT_INFORMATION_H_
#define _KML_POINT_INFORMATION_H_

#include <vector>

#include "kml_point.h"
#include "kml_extra_data_information.h"
#include "kml_style.h"

namespace kml_writer
{

class PointInformation
{
public:
    void setPoint(KmlPoint point)
    {
        this->point = point;
    }

    void setPoint(double lon, double lat, double alt = 0)
    {
        KmlPoint point;
        point.setX(lon);
        point.setY(lat);
        point.setZ(alt);

        setPoint(point);
    }

    void addExtraData(ExtraData extraData)
    {
        v_extraData.push_back(extraData);
    }

    void addExtraData(std::string name, std::string value)
    {
        ExtraData extraData;
        extraData.name = name;
        extraData.value = value;

        addExtraData(extraData);
    }

    void addExtraData(std::string name, int value)
    {
        addExtraData(name, std::to_string((int)value));
    }

    void setStyle(Style style)
    {
        this->style = style;
    }

    std::string name;
    KmlPoint point;
    Style style;
    std::vector<ExtraData> v_extraData;
};

} // namespace kml_writer

#endif // _KML_POINT_INFORMATION_H_