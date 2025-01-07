/**
 * kml_writer.h
 *
 * KML writer
 *
 * Created by Cheng Liu on 09/22/2021
 * Copyright 2021 Z-ONE Inc. All rights reserved.
 *
 */

#ifndef _KML_WRITER_H_
#define _KML_WRITER_H_

#include <vector>
#include <fstream>

#include "kml_base/kml_point.h"
#include "kml_base/kml_line_information.h"
#include "kml_base/kml_point_information.h"

namespace kml_writer
{

class KmlWriter
{
public:
    KmlWriter() = default;
    virtual ~KmlWriter() { }

    virtual void writeLinesToKml(const std::string fileName, const std::vector<LineInformation>& v_lineInformation) = 0;
    virtual void writePointsToKml(const std::string fileName, const std::vector<PointInformation>& v_pointInformation) = 0;

public:
    virtual void writeLineInformation(const LineInformation& lineInformation) = 0;
    virtual void writePointInformation(const PointInformation& pointInformation) = 0;

    virtual void writeSinglePoint(const KmlPoint& point) = 0;

    virtual void writePoint(const KmlPoint& point) = 0;
    virtual void writeLine(const std::vector<KmlPoint>& v_points) = 0;

    virtual void writeExtraData(const ExtraData& extraData) = 0;
    virtual void writeExtraDatas(const std::vector<ExtraData>& v_extraData) = 0;

    virtual void writeKMLHead() = 0;
    virtual void writeKMLTail() = 0;
};

} // namespace kml_writer

#endif // _KML_WRITER_H_