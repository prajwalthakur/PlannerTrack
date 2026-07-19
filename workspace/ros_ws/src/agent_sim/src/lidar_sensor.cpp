// Author Prajwal Thakur 
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "agent_sim/lidar_sensor.hpp"

#include <algorithm>
#include <cmath>

namespace lidar_sim
{

static constexpr float kInf = std::numeric_limits<float>::infinity();

// ---------------------------------------------------------------------------
// Ray–Circle intersection
// ---------------------------------------------------------------------------
float LidarSensor::rayCircle(
  float ox, float oy, float dx, float dy,
  float cx, float cy, float r,
  float range_min) const
{
  const float vx = ox - cx;
  const float vy = oy - cy;
  const float b  = 2.0f * (vx * dx + vy * dy);
  const float c  = vx * vx + vy * vy - r * r;
  const float disc = b * b - 4.0f * c;
  if (disc < 0.0f) {
    return kInf;
  }
  const float t = (-b - std::sqrt(disc)) * 0.5f;
  return (t > range_min) ? t : kInf;
}

// ---------------------------------------------------------------------------
// Ray–Line Segment intersection (Cramer's rule)
// ---------------------------------------------------------------------------
float LidarSensor::raySegment(
  float ox, float oy, float dx, float dy,
  float x1, float y1, float x2, float y2,
  float range_min) const
{
  const float ex = x2 - x1;
  const float ey = y2 - y1;
  const float fx = ox - x1;
  const float fy = oy - y1;

  // denom = (Q-P) × d  (2D cross product); sign gives ray/segment orientation
  const float denom = dy * ex - dx * ey;
  if (std::fabs(denom) < 1e-9f) {
    return kInf;  // ray parallel to segment
  }

  const float t = (fx * ey - fy * ex) / denom;
  const float s = (fx * dy - fy * dx) / denom;

  if (t > range_min && s >= 0.0f && s <= 1.0f) {
    return t;
  }
  return kInf;
}

// ---------------------------------------------------------------------------
// Ray–OBB intersection (slab method in local frame)
// ---------------------------------------------------------------------------
float LidarSensor::rayOBB(
  float ox, float oy, float dx, float dy,
  const WorldOBB & obb,
  float range_min) const
{
  // Transform ray origin and direction into OBB local frame.
  const float cos_phi = std::cos(obb.phi);
  const float sin_phi = std::sin(obb.phi);

  const float tx = ox - obb.cx;
  const float ty = oy - obb.cy;

  const float lox =  cos_phi * tx + sin_phi * ty;
  const float loy = -sin_phi * tx + cos_phi * ty;
  const float ldx =  cos_phi * dx + sin_phi * dy;
  const float ldy = -sin_phi * dx + cos_phi * dy;

  // Slab test on axis 0 (half-length hl, along local-x)
  float t_near, t_far;
  if (std::fabs(ldx) < 1e-9f) {
    if (lox < -obb.hl || lox > obb.hl) {
      return kInf;
    }
    t_near = -kInf;
    t_far  =  kInf;
  } else {
    float t0 = (-obb.hl - lox) / ldx;
    float t1 = ( obb.hl - lox) / ldx;
    if (t0 > t1) {
      std::swap(t0, t1);
    }
    t_near = t0;
    t_far  = t1;
  }

  // Slab test on axis 1 (half-width hw, along local-y)
  if (std::fabs(ldy) < 1e-9f) {
    if (loy < -obb.hw || loy > obb.hw) {
      return kInf;
    }
  } else {
    float t0 = (-obb.hw - loy) / ldy;
    float t1 = ( obb.hw - loy) / ldy;
    if (t0 > t1) {
      std::swap(t0, t1);
    }
    t_near = std::max(t_near, t0);
    t_far  = std::min(t_far,  t1);
  }

  if (t_far < t_near || t_near < range_min) {
    return kInf;
  }
  return t_near;
}

// ---------------------------------------------------------------------------
// Main scan computation
// ---------------------------------------------------------------------------
std::vector<float> LidarSensor::computeScan(
  float ox, float oy, float phi,
  const std::vector<WorldOBB> & obbs,
  const std::vector<WorldCircle> & circles,
  const std::vector<WorldSegment> & segments,
  const LidarParams & params) const
{
  const float angle_inc = (params.num_beams > 1)
    ? (params.angle_max - params.angle_min) / static_cast<float>(params.num_beams)
    : 0.0f;

  std::vector<float> ranges(params.num_beams, params.range_max);

  for (int k = 0; k < params.num_beams; ++k) {
    const float world_angle = phi + params.angle_min + static_cast<float>(k) * angle_inc;
    const float dx = std::cos(world_angle);
    const float dy = std::sin(world_angle);

    float t_min = params.range_max;

    for (const auto & obb : obbs) {
      const float t = rayOBB(ox, oy, dx, dy, obb, params.range_min);
      if (t < t_min) {
        t_min = t;
      }
    }

    for (const auto & circ : circles) {
      const float t = rayCircle(ox, oy, dx, dy, circ.x, circ.y, circ.r, params.range_min);
      if (t < t_min) {
        t_min = t;
      }
    }

    for (const auto & seg : segments) {
      const float t = raySegment(
        ox, oy, dx, dy, seg.x1, seg.y1, seg.x2, seg.y2, params.range_min);
      if (t < t_min) {
        t_min = t;
      }
    }

    ranges[k] = std::min(t_min, params.range_max);
  }

  return ranges;
}

}  // namespace lidar_sim
