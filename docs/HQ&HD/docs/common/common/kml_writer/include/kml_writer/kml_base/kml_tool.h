/**
 * kml_tool.h
 *
 * KML tool
 *
 * Created by Cheng Liu on 09/22/2021
 * Copyright 2021 Z-ONE Inc. All rights reserved.
 *
 */

#ifndef _KML_TOOL_H_
#define _KML_TOOL_H_

namespace kml_writer
{

class KmlTool
{
public:
    static double headingConvert2KMLHeading(const double heading);

private:
    KmlTool() = delete;
};

} // namespace kml_writer

#endif // _KML_TOOL_H_