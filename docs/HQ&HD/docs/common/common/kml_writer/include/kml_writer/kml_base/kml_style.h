/**
 * kml_extra_data_information.h
 *
 * Extra data
 *
 * Created by Cheng Liu on 09/22/2021
 * Copyright 2021 Z-ONE Inc. All rights reserved.
 *
 */

#ifndef _KML_STYLE_H_
#define _KML_STYLE_H_

#include <string>

namespace kml_writer
{

struct Icon
{
    std::string href;
};

enum IconStyleColor {
    ICON_STYLE_COLOR_NULL,
    ICON_STYLE_COLOR_RED,
    ICON_STYLE_COLOR_GREEN,
    ICON_STYLE_COLOR_BLUE,
    ICON_STYLE_COLOR_YELLOW,
    ICON_STYLE_COLOR_CYAN,
    ICON_STYLE_COLOR_PURPLE,
    ICON_STYLE_COLOR_MAX
};

static inline std::string getIconStyleColor(IconStyleColor color)
{
    std::string kml_color_string = "empty";
    switch(color)
    {
    case ICON_STYLE_COLOR_NULL:
        kml_color_string = "null";
        break;
    case ICON_STYLE_COLOR_RED:
        kml_color_string = "FF0000FF";
        break;
    case ICON_STYLE_COLOR_GREEN:
        kml_color_string = "FF00FF00";
        break;
    case ICON_STYLE_COLOR_BLUE:
        kml_color_string = "FFFF0000";
        break;
    case ICON_STYLE_COLOR_YELLOW:
        kml_color_string = "FF00FFFF";
    case ICON_STYLE_COLOR_PURPLE:
        kml_color_string = "FFFF00FF";
    case ICON_STYLE_COLOR_CYAN:
        kml_color_string = "FFFFFF00";
        break;
    case ICON_STYLE_COLOR_MAX:
        kml_color_string = "max";
        break;
    default:
        ;
    }

    return kml_color_string;
}

struct IconStyle
{
    // Color: ABGR
    // Example:
    // FFFF0000 -> blue
    // FF00FF00 -> green
    // FF0000FF -> red
    std::string color;
    std::string colorMode;

    std::string scale;
    std::string heading;

    Icon icon;
};

struct Style
{
    IconStyle iconStyle;
    bool valid = false;
};

} // namespace kml_writer

#endif // _KML_STYLE_H_