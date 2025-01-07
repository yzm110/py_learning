/**
 * kml_line_information.h
 *
 * Line information
 *
 * Created by Cheng Liu on 09/22/2021
 * Copyright 2021 Z-ONE Inc. All rights reserved.
 *
 */

#ifndef _KML_LINE_INFORMATION_H_
#define _KML_LINE_INFORMATION_H_

#include <string>

#include "kml_point.h"
#include "kml_extra_data_information.h"

namespace kml_writer
{

class LineInformation
{
public:
    void addPoint(KmlPoint point)
    {
        v_points.push_back(point);
    }

    void addPoint(double lon, double lat, double alt = 0)
    {
        KmlPoint point;
        point.setX(lon);
        point.setY(lat);
        point.setZ(alt);

        addPoint(point);
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

public:
    std::string name;
    std::vector<KmlPoint> v_points;
    std::vector<ExtraData> v_extraData;
};

} // namespace kml_writer

#endif // _KML_LINE_INFORMATION_H_