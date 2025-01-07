#include "ehradapterhelper.h"

void EhrAdapterHelper::BSpineInterpolation(bc::TFixedVector<bc::float64_t, 1000>& x,
                                           bc::TFixedVector<bc::float64_t, 1000>& y,
                                           bc::TFixedVector<bc::float64_t, 1000>& rx_,
                                           bc::TFixedVector<bc::float64_t, 1000>& ry_, bc::uint32_t& idx,
                                           uint32_t& current_idx)
{
  // bc::TFixedVector<bc::float64_t, 1000> rx, ry, rs, rh, rc;
  bc::TFixedVector<bc::float64_t, 1000> rx, ry;
  // bc::TFixedVector<bc::float64_t, 1000> m_length;
  bc::int32_t m_InsertNum = 0;
  bc::float64_t a0 = 0, a1 = 0, a2 = 0, a3 = 0, b0 = 0, b1 = 0, b2 = 0, b3 = 0;  // bspine coeff
  bc::float64_t insert_x, insert_y;
  bc::float64_t xt, yt, xtt, ytt;  // tangential, (second derivative);
  bc::float64_t xlast = 0, ylast = 0;
  // bc::float64_t temcurvature = 0.0;
  // bc::float64_t tempyaw;
  bc::float64_t dx, dy;
  bc::int32_t ww = 0;
  // bc::float64_t lengthall = 0.0;
  if (x.size() < 2) return;
  dx = x[1] - x[0];
  dy = y[1] - y[0];

  bc::float64_t tmpWaypointx, tmpWaypointy;
  tmpWaypointx = x[0] - dx;
  tmpWaypointy = y[0] - dy;

  x.insert(x.begin(), tmpWaypointx);
  y.insert(y.begin(), tmpWaypointy);
  // filter_llanetype.inser
  dx = x[x.size() - 1] - x[x.size() - 2];
  dy = y[y.size() - 1] - y[y.size() - 2];

  tmpWaypointx = x.back() + dx;
  tmpWaypointy = y.back() + dy;

  x.push_back(tmpWaypointx);
  y.push_back(tmpWaypointy);

  for (bc::int32_t uu = 0; uu < x.size() - 3; uu++)
  {
    a0 = 1 / 6.0 * (x[uu + 0] + 4 * x[uu + 1] + x[uu + 2]);                   // 1/6 *(1p0 + 4p1 + 1p3)
    a1 = -1 / 2.0 * (x[uu + 0] - x[uu + 2]);                                  // 1/6 *( -3p0 +0p1 + 3p2 +0p3)
    a2 = 1 / 2.0 * (x[uu + 0] - 2 * x[uu + 1] + x[uu + 2]);                   // 1/6*(3p0 - 6p1 + 3p2 +0p3)
    a3 = -1 / 6.0 * (x[uu + 0] - 3 * x[uu + 1] + 3 * x[uu + 2] - x[uu + 3]);  // 1/6*(-1p0 - 3p1 - 3p2 +1p3)

    b0 = 1 / 6.0 * (y[uu + 0] + 4 * y[uu + 1] + y[uu + 2]);
    b1 = -1 / 2.0 * (y[uu + 0] - y[uu + 2]);
    b2 = 1 / 2.0 * (y[uu + 0] - 2 * y[uu + 1] + y[uu + 2]);
    b3 = -1 / 6.0 * (y[uu + 0] - 3 * y[uu + 1] + 3 * y[uu + 2] - y[uu + 3]);

    bc::float64_t dis = hypot(x[uu + 0] - x[uu + 1], y[uu + 0] - y[uu + 1]) +
                        hypot(x[uu + 1] - x[uu + 2], y[uu + 1] - y[uu + 2]) +
                        hypot(x[uu + 2] - x[uu + 3], y[uu + 2] - y[uu + 3]);

    m_InsertNum = static_cast<bc::int32_t>(dis / 2.f) + 1;
    xlast = x[uu];
    ylast = y[uu];
    for (ww = 1; ww < m_InsertNum; ww++)
    {
      bc::float64_t u = ww / bc::float64_t(m_InsertNum);
      bc::float64_t u_square = pow(u, 2);
      bc::float64_t u_cube = pow(u, 3);

      insert_x = a0 + a1 * u + a2 * u_square + a3 * u_cube;
      insert_y = b0 + b1 * u + b2 * u_square + b3 * u_cube;

      xt = a1 + 2 * a2 * u + 3 * u_square * a3;
      xtt = 2 * a2 + 6 * u_square * a3;

      yt = b1 + 2 * b2 * u + 3 * u_square * b3;
      ytt = 2 * b2 + 6 * u * b3;

      // temcurvature = (xt * ytt - yt * xtt) / pow((xt * xt + yt * yt), 1.5);
      // tempyaw = atan2(insert_y - ylast, insert_x - xlast);

      rx.push_back(insert_x);
      ry.push_back(insert_y);
      // rh.push_back(tempyaw);
      // rc.push_back(temcurvature);
      // lengthall += sqrt((insert_x - xlast) * (insert_x - xlast) +
      //                   (insert_y - ylast) * (insert_y - ylast));
      // rs.push_back(lengthall);
      xlast = insert_x;
      ylast = insert_y;
    }
  }
  insert_x = a0 + a1 + a2 + a3;
  insert_y = b0 + b1 + b2 + b3;
  xt = a1 + 2 * a2 + 3 * a3;
  xtt = 2 * a2 + 6 * a3;
  yt = b1 + 2 * b2 + 3 * b3;
  ytt = 2 * b2 + 6 * b3;
  // temcurvature = (xt * ytt - yt * xtt) / pow((xt * xt + yt * yt), 1.5);
  // tempyaw = atan2(insert_y - ylast, insert_x - xlast);

  rx.push_back(insert_x);
  ry.push_back(insert_y);
  // rh.push_back(tempyaw);
  // rc.push_back(temcurvature);

  // lengthall += sqrt((insert_x - xlast) * (insert_x - xlast) +
  //                   (insert_y - ylast) * (insert_y - ylast));
  // rs.push_back(lengthall);

  bc::float64_t tt_x, tt_y;
  tt_x = rx[0];
  tt_y = ry[0];

  rx_.push_back(tt_x);
  ry_.push_back(tt_y);
  // rh_.push_back(rh[0]);
  // rc_.push_back(rc[0]);
  float32_t min_dis = 20;
  // uint32_t current_idx = 0;
  if ((hypot(rx[0], ry[0]) < min_dis) && (rx[0] > 0.0) && rx[0] < 5)
  {
    min_dis = hypot(rx[0], ry[0]);
    idx = current_idx;
  }
  for (bc::int32_t j = 1; j < rx.size(); j++)
  {
    if ((hypot(tt_x - rx[j], tt_y - ry[j]) < 4.0) || (rx[j] < -50.0))
    {
      continue;
    }
    if ((hypot(rx[j], ry[j]) < min_dis) && (rx[j] > 0.0) && rx[j] < 5)
    {
      min_dis = hypot(rx[j], ry[j]);
      idx = current_idx;
    }
    tt_x = rx[j];
    tt_y = ry[j];

    rx_.push_back(tt_x);
    ry_.push_back(tt_y);
    // rs_.push_back(rs[j]);
    // rh_.push_back(rh[j]);
    // rc_.push_back(rc[j]);
    current_idx++;
  }
  if (rx.size() > 0) current_idx++;
}

void EhrAdapterHelper::BSpineInterpolationCenterline(bc::TFixedVector<bc::float64_t, 1000>& poins_offset,
                                                     bc::TFixedVector<bc::float64_t, 1000>& rpoins_offset_,
                                                     bc::TFixedVector<bc::float64_t, 1000>& x,
                                                     bc::TFixedVector<bc::float64_t, 1000>& y,
                                                     bc::TFixedVector<bc::float64_t, 1000>& rx_,
                                                     bc::TFixedVector<bc::float64_t, 1000>& ry_, bc::uint32_t& idx,
                                                     uint32_t& current_idx)
{
  // bc::TFixedVector<bc::float64_t, 1000> rx, ry, rs, rh, rc, roffset;
  bc::TFixedVector<bc::float64_t, 1000> rx, ry, roffset;
  // bc::TFixedVector<bc::float64_t, 1000> m_length;
  bc::int32_t m_InsertNum = 0;
  bc::float64_t a0 = 0.f, a1 = 0.f, a2 = 0.f, a3 = 0.f, b0 = 0.f, b1 = 0.f, b2 = 0.f, b3 = 0.f;  // bspine coeff
  bc::float64_t c0 = 0.f, c1 = 0.f, c2 = 0.f, c3 = 0.f;
  bc::float64_t insert_x = 0.f, insert_y = 0.f, insert_offset = 0.f;
  bc::float64_t xt = 0.f, yt = 0.f, xtt = 0.f, ytt = 0.f;  // tangential, (second derivative);
  bc::float64_t xlast = 0, ylast = 0;
  // bc::float64_t temcurvature = 0.0;
  // bc::float64_t tempyaw = 0.f;
  bc::float64_t dx = 0.f, dy = 0.f;
  bc::int32_t ww = 0;
  // bc::float64_t lengthall = 0.0;
  if (x.size() < 2) return;
  dx = x[1] - x[0];
  dy = y[1] - y[0];

  bc::float64_t be_dis = hypot(dx, dy);
  bc::float64_t be_offest = poins_offset[0] - be_dis;
  poins_offset.insert(poins_offset.begin(), be_offest);

  bc::float64_t tmpWaypointx, tmpWaypointy;
  tmpWaypointx = x[0] - dx;
  tmpWaypointy = y[0] - dy;

  x.insert(x.begin(), tmpWaypointx);
  y.insert(y.begin(), tmpWaypointy);
  // filter_llanetype.inser
  dx = x[x.size() - 1] - x[x.size() - 2];
  dy = y[y.size() - 1] - y[y.size() - 2];

  tmpWaypointx = x.back() + dx;
  tmpWaypointy = y.back() + dy;

  poins_offset.push_back(poins_offset.back() + hypot(dx, dy));

  x.push_back(tmpWaypointx);
  y.push_back(tmpWaypointy);

  // bc::float64_t af_dis = hypot(dx, dy);

  for (bc::int32_t uu = 0; uu < x.size() - 3; uu++)
  {
    a0 = 1 / 6.0 * (x[uu + 0] + 4 * x[uu + 1] + x[uu + 2]);                   // 1/6 *(1p0 + 4p1 + 1p3)
    a1 = -1 / 2.0 * (x[uu + 0] - x[uu + 2]);                                  // 1/6 *( -3p0 +0p1 + 3p2 +0p3)
    a2 = 1 / 2.0 * (x[uu + 0] - 2 * x[uu + 1] + x[uu + 2]);                   // 1/6*(3p0 - 6p1 + 3p2 +0p3)
    a3 = -1 / 6.0 * (x[uu + 0] - 3 * x[uu + 1] + 3 * x[uu + 2] - x[uu + 3]);  // 1/6*(-1p0 - 3p1 - 3p2 +1p3)

    b0 = 1 / 6.0 * (y[uu + 0] + 4 * y[uu + 1] + y[uu + 2]);
    b1 = -1 / 2.0 * (y[uu + 0] - y[uu + 2]);
    b2 = 1 / 2.0 * (y[uu + 0] - 2 * y[uu + 1] + y[uu + 2]);
    b3 = -1 / 6.0 * (y[uu + 0] - 3 * y[uu + 1] + 3 * y[uu + 2] - y[uu + 3]);

    c0 = 1 / 6.0 * (poins_offset[uu + 0] + 4 * poins_offset[uu + 1] + poins_offset[uu + 2]);
    c1 = -1 / 2.0 * (poins_offset[uu + 0] - poins_offset[uu + 2]);
    c2 = 1 / 2.0 * (poins_offset[uu + 0] - 2 * poins_offset[uu + 1] + poins_offset[uu + 2]);
    c3 = -1 / 6.0 * (poins_offset[uu + 0] - 3 * poins_offset[uu + 1] + 3 * poins_offset[uu + 2] - poins_offset[uu + 3]);

    bc::float64_t dis = hypot(x[uu + 0] - x[uu + 1], y[uu + 0] - y[uu + 1]) +
                        hypot(x[uu + 1] - x[uu + 2], y[uu + 1] - y[uu + 2]) +
                        hypot(x[uu + 2] - x[uu + 3], y[uu + 2] - y[uu + 3]);

    m_InsertNum = static_cast<bc::int32_t>(dis / 2.f) + 1;
    xlast = x[uu];
    ylast = y[uu];
    for (ww = 1; ww < m_InsertNum; ww++)
    {
      bc::float64_t u = ww / bc::float64_t(m_InsertNum);
      bc::float64_t u_square = pow(u, 2);
      bc::float64_t u_cube = pow(u, 3);

      insert_x = a0 + a1 * u + a2 * u_square + a3 * u_cube;
      insert_y = b0 + b1 * u + b2 * u_square + b3 * u_cube;
      insert_offset = c0 + c1 * u + c2 * u_square + c3 * u_cube;

      xt = a1 + 2 * a2 * u + 3 * u_square * a3;
      xtt = 2 * a2 + 6 * u_square * a3;

      yt = b1 + 2 * b2 * u + 3 * u_square * b3;
      ytt = 2 * b2 + 6 * u * b3;

      // temcurvature = (xt * ytt - yt * xtt) / pow((xt * xt + yt * yt), 1.5);
      // tempyaw = atan2(insert_y - ylast, insert_x - xlast);

      rx.push_back(insert_x);
      ry.push_back(insert_y);
      // rh.push_back(tempyaw);
      // rc.push_back(temcurvature);
      roffset.push_back(insert_offset);

      // lengthall += sqrt((insert_x - xlast) * (insert_x - xlast) +
      //                   (insert_y - ylast) * (insert_y - ylast));
      // rs.push_back(lengthall);
      xlast = insert_x;
      ylast = insert_y;
    }
  }
  insert_x = a0 + a1 + a2 + a3;
  insert_y = b0 + b1 + b2 + b3;
  insert_offset = c0 + c1 + c2 + c3;

  xt = a1 + 2 * a2 + 3 * a3;
  xtt = 2 * a2 + 6 * a3;
  yt = b1 + 2 * b2 + 3 * b3;
  ytt = 2 * b2 + 6 * b3;
  // temcurvature = (xt * ytt - yt * xtt) / pow((xt * xt + yt * yt), 1.5);
  // tempyaw = atan2(insert_y - ylast, insert_x - xlast);

  rx.push_back(insert_x);
  ry.push_back(insert_y);
  roffset.push_back(insert_offset);
  // rh.push_back(tempyaw);
  // rc.push_back(temcurvature);

  // lengthall += sqrt((insert_x - xlast) * (insert_x - xlast) +
  //                   (insert_y - ylast) * (insert_y - ylast));
  // rs.push_back(lengthall);

  bc::float64_t tt_x, tt_y, tt_offset;
  tt_x = rx[0];
  tt_y = ry[0];
  tt_offset = roffset[0];

  rx_.push_back(tt_x);
  ry_.push_back(tt_y);
  // rh_.push_back(rh[0]);
  // rc_.push_back(rc[0]);
  rpoins_offset_.push_back(tt_offset);

  float32_t min_dis = 20;
  // uint32_t current_idx = 0;
  if ((hypot(rx[0], ry[0]) < min_dis) && (rx[0] > 0.0) && rx[0] < 5)
  {
    min_dis = hypot(rx[0], ry[0]);
    idx = current_idx;
  }
  for (bc::int32_t j = 1; j < rx.size(); j++)
  {
    if ((hypot(tt_x - rx[j], tt_y - ry[j]) < 4.0) || (rx[j] < -50.0))
    {
      continue;
    }
    if ((hypot(rx[j], ry[j]) < min_dis) && (rx[j] > 0.0) && rx[j] < 5)
    {
      min_dis = hypot(rx[j], ry[j]);
      idx = current_idx;
    }
    tt_x = rx[j];
    tt_y = ry[j];

    rx_.push_back(tt_x);
    ry_.push_back(tt_y);
    // rs_.push_back(rs[j]);
    // rh_.push_back(rh[j]);
    // rc_.push_back(rc[j]);
    rpoins_offset_.push_back(roffset[j]);
    current_idx++;
  }
  if (rx.size() > 0) current_idx++;
}
EhrAdapterHelper::EhrAdapterHelper(/* args */) {}

EhrAdapterHelper::~EhrAdapterHelper() {}
