/**
 * kml_point.h
 *
 * Point
 *
 * Created by Cheng Liu on 09/22/2021
 * Copyright 2021 Z-ONE Inc. All rights reserved.
 *
 */

#ifndef _KML_POINT_H_
#define _KML_POINT_H_

namespace kml_writer
{

class KmlPoint
{
public:
    KmlPoint()
        : _x(0.0)
        , _y(0.0)
        , _z(0.0)
        , _advanced(false)
    {

    }

    KmlPoint(double x, double y)
    {
        this->_x = x;
        this->_y = y;
    }

    KmlPoint(double x, double y, double z)
    {
        this->_x = x;
        this->_y = y;
        this->_z = z;
    }

    KmlPoint(const KmlPoint& rhs)
    {
        _x = rhs._x;
        _y = rhs._y;
        _z = rhs._z;
        _advanced = rhs._advanced;
    }

    virtual ~KmlPoint()
    {
        _x = 0.0;
        _y = 0.0;
        _z = 0.0;
        _advanced = false;
    }

    inline double x() const { return _x; }
    inline double y() const { return _y; }
    inline double z() const { return _z; }

    inline void setX(double newVal) { _x = newVal; }
    inline void setY(double newVal) { _y = newVal; }
    inline void setZ(double newVal) { _z = newVal; _advanced = true; }
    inline bool isAdvanced() const { return _advanced; }

    bool operator==(const KmlPoint& rhs)
    {
        return ((_x == rhs._x) &&
            (_y == rhs._y) &&
            (_z == rhs._z) &&
            (_advanced == rhs._advanced));
    }

    KmlPoint& operator=(const KmlPoint& rhs)
    {
        _x = rhs._x;
        _y = rhs._y;
        _z = rhs._z;
        _advanced = rhs._advanced;

        return *this;
    }

private:
    double _x;
    double _y;
    double _z;
    bool _advanced;
};

} // namespace kml_writer

#endif // _KML_POINT_H_