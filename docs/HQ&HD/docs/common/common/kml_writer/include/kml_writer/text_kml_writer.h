/**
 * text_kml_writer.h
 *
 * Text kml writer
 *
 * Created by Cheng Liu on 09/22/2021
 * Copyright 2021 Z-ONE Inc. All rights reserved.
 *
 */

#ifndef _TEXT_KML_WRITER_H_
#define _TEXT_KML_WRITER_H_

#include <vector>
#include <fstream>

#include "kml_base/kml_point.h"
#include "kml_base/kml_line_information.h"
#include "kml_base/kml_point_information.h"
#include "kml_wirter.h"

namespace kml_writer
{

class TextKmlWriter : public KmlWriter
{
public:
    TextKmlWriter() = default;
    void writeLinesToKml(const std::string fileName, const std::vector<LineInformation>& v_lineInformation) override;
    void writePointsToKml(const std::string fileName, const std::vector<PointInformation>& v_pointInformation) override;

public:
    virtual void writeLineInformation(const LineInformation& lineInformation) override;
    virtual void writePointInformation(const PointInformation& pointInformation) override;

    virtual void writeSinglePoint(const KmlPoint& point) override;

    virtual void writePoint(const KmlPoint& point) override;
    virtual void writeLine(const std::vector<KmlPoint>& v_points) override;

    virtual void writeExtraData(const ExtraData& extraData) override;
    virtual void writeExtraDatas(const std::vector<ExtraData>& v_extraData) override;

    virtual void writeKMLHead() override;
    virtual void writeKMLTail() override;

    void writeStyle(const Style& style);

private:
    std::ofstream  m_ofs;
};

} // namespace kml_writer

#endif // _TEXT_KML_WRITER_H_