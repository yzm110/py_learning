/**
 * kml_tool.cpp
 *
 * KML tool
 *
 * Created by Cheng Liu on 09/22/2021
 * Copyright 2021 Z-ONE Inc. All rights reserved.
 *
 */
#include "kml_writer/kml_base/kml_tool.h"

namespace kml_writer
{

double KmlTool::headingConvert2KMLHeading(const double heading)
{
    // KML direction is opposite to engine direction, KML:[-180, 180], MM Engine:[0, 360]
    double headingFixed = 0.0;
    if (heading > 0)
    {
        headingFixed = heading - 180.0;
    }
    else
    {
        headingFixed = heading + 180.0;
    }

    return headingFixed;
}

} // namespace kml_writer