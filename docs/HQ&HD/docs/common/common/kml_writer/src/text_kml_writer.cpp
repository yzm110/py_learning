/**
 * text_kml_writer.cpp
 *
 * Text KML writer
 *
 * Created by Cheng Liu on 09/22/2021
 * Copyright 2021 Z-ONE Inc. All rights reserved.
 *
 */

#include "kml_writer/text_kml_writer.h"

#include <string>

namespace kml_writer
{

void TextKmlWriter::writeLinesToKml(const std::string fileName, const std::vector<LineInformation>& v_lineInformation)
{
    m_ofs.precision(10);
    m_ofs.open(fileName, std::ios::out);

    writeKMLHead();
    for (auto lineInformation : v_lineInformation)
    {
        writeLineInformation(lineInformation);
    }
    writeKMLTail();

    m_ofs.close();
}

void TextKmlWriter::writePointsToKml(const std::string fileName, const std::vector<PointInformation>& v_pointInformation)
{
    m_ofs.precision(10);
    m_ofs.open(fileName, std::ios::out);

    writeKMLHead();
    for (auto pointInformation : v_pointInformation)
    {
        writePointInformation(pointInformation);
    }
    writeKMLTail();

    m_ofs.close();
}

void TextKmlWriter::writeKMLHead()
{
    m_ofs << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl;
    m_ofs << "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\">" << std::endl;
    m_ofs << "\t<Document id=\"1\">" << std::endl;
    m_ofs << "\t\t<Folder id=\"2\">" << std::endl;
    m_ofs << "\t\t\t<name>Position Folder</name>" << std::endl;
}

void TextKmlWriter::writeKMLTail()
{
    m_ofs << "\t\t</Folder>" << std::endl;
    m_ofs << "\t</Document>" << std::endl;
    m_ofs << "</kml>" << std::endl;
}

void TextKmlWriter::writePointInformation(const PointInformation& pointInformation)
{
    std::string name = pointInformation.name;
    KmlPoint point = pointInformation.point;
    Style style = pointInformation.style;
    std::vector<ExtraData> v_extraData = pointInformation.v_extraData;

    m_ofs << "\t\t\t<Placemark>" << std::endl;
    m_ofs << "\t\t\t\t<name>" << name << "</name>" << std::endl;

    writePoint(point);
    writeStyle(style);
    writeExtraDatas(v_extraData);

    m_ofs << "\t\t\t</Placemark>" << std::endl;
}

void TextKmlWriter::writeLineInformation(const LineInformation& lineInformation)
{
    std::string name = lineInformation.name;
    std::vector<KmlPoint> v_points = lineInformation.v_points;
    std::vector<ExtraData> v_extraData = lineInformation.v_extraData;

    m_ofs << "\t\t\t<Placemark>" << std::endl;
    m_ofs << "\t\t\t\t<name>" << name << "</name>" << std::endl;

    writeLine(v_points);
    writeExtraDatas(v_extraData);

    m_ofs << "\t\t\t</Placemark>" << std::endl;
}

void TextKmlWriter::writeSinglePoint(const KmlPoint& point)
{
    double x = point.x();
    double y = point.y();

    m_ofs << "\t\t\t\t\t\t" << x << "," << y << ",0.0" << std::endl;
}

void TextKmlWriter::writeLine(const std::vector<KmlPoint>& v_points)
{
    m_ofs << "\t\t\t\t<LineString>" << std::endl;
    m_ofs << "\t\t\t\t\t<coordinates>" << std::endl;

    for (auto point : v_points)
    {
        writeSinglePoint(point);
    }

    m_ofs << "\t\t\t\t\t</coordinates>" << std::endl;
    m_ofs << "\t\t\t\t</LineString>" << std::endl;
}

void TextKmlWriter::writePoint(const KmlPoint& point)
{
    m_ofs << "\t\t\t\t<Point>" << std::endl;
    m_ofs << "\t\t\t\t\t<coordinates>" << std::endl;

    writeSinglePoint(point);

    m_ofs << "\t\t\t\t\t</coordinates>" << std::endl;
    m_ofs << "\t\t\t\t</Point>" << std::endl;
}

void TextKmlWriter::writeExtraData(const ExtraData& extraData)
{
    m_ofs << "\t\t\t\t\t<Data name=\"" << extraData.name << "\">" << std::endl;
    m_ofs << "\t\t\t\t\t\t<value>" << extraData.value << "</value>" << std::endl;
    m_ofs << "\t\t\t\t\t</Data>" << std::endl;
}

void TextKmlWriter::writeExtraDatas(const std::vector<ExtraData>& v_extraData)
{
    if (v_extraData.size() > 0)
    {
        m_ofs << "\t\t\t\t<ExtendedData>" << std::endl;

        for (auto extraData : v_extraData)
        {
            writeExtraData(extraData);
        }

        m_ofs << "\t\t\t\t</ExtendedData>" << std::endl;
    }
}

void TextKmlWriter::writeStyle(const Style& style)
{
    if (style.valid)
    {
        m_ofs << "\t\t\t\t<Style>" << std::endl;
        m_ofs << "\t\t\t\t\t<IconStyle>" << std::endl;
        m_ofs << "\t\t\t\t\t\t<color>" << style.iconStyle.color << "</color>" << std::endl;
        m_ofs << "\t\t\t\t\t\t<colorMode>" << style.iconStyle.colorMode << "</colorMode>" << std::endl;
        m_ofs << "\t\t\t\t\t\t<scale>" << style.iconStyle.scale << "</scale>" << std::endl;
        m_ofs << "\t\t\t\t\t\t<heading>" << style.iconStyle.heading << "</heading>" << std::endl;
        m_ofs << "\t\t\t\t\t\t<Icon>" << std::endl;
        m_ofs << "\t\t\t\t\t\t\t<href>" << style.iconStyle.icon.href << "</href>" << std::endl;
        m_ofs << "\t\t\t\t\t\t</Icon>" << std::endl;
        m_ofs << "\t\t\t\t\t</IconStyle>" << std::endl;
        m_ofs << "\t\t\t\t</Style>" << std::endl;
    }
}

} // namespace kml_writer
