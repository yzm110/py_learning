/**
 * kml_extra_data_information.h
 *
 * Extra data
 *
 * Created by Cheng Liu on 09/22/2021
 * Copyright 2021 Z-ONE Inc. All rights reserved.
 *
 */

#ifndef _KML_EXTRA_DATA_INFORMATION_H_
#define _KML_EXTRA_DATA_INFORMATION_H_

#include <string>

namespace kml_writer
{

struct ExtraData
{
    std::string name;
    std::string value;
};

} // namespace kml_writer

#endif // _KML_EXTRA_DATA_INFORMATION_H_
