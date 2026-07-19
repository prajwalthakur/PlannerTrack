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

#pragma once

#include <cmath>
#include <limits>
#include <vector>

namespace lidar_sim
{

struct LidarParams
{
  int   num_beams{360};
  float angle_min{-static_cast<float>(M_PI)};
  float angle_max{ static_cast<float>(M_PI)};
  float range_min{0.05f};
  float range_max{10.0f};
};

// Circular obstacle or circular-shaped agent
struct WorldCircle
{
  float x, y, r;
};

// Corridor wall or line-segment obstacle
struct WorldSegment
{
  float x1, y1, x2, y2;
};

// Oriented Bounding Box — exact shape for rectangle-shaped agents
// hl = half-length (along agent's heading axis)
// hw = half-width  (perpendicular to heading axis)
struct WorldOBB
{
  float cx, cy;   // center
  float hl, hw;   // half-extents
  float phi;      // heading (radians, world frame)
};

class LidarSensor
{
public:
  // Returns a vector of ranges (size = num_beams).
  // Beams with no intersection within [range_min, range_max] are set to range_max.
  // The scan is computed in the world frame; beam angles are relative to the agent heading phi.
  std::vector<float> computeScan(
    float ox, float oy, float phi,
    const std::vector<WorldOBB> & obbs,
    const std::vector<WorldCircle> & circles,
    const std::vector<WorldSegment> & segments,
    const LidarParams & params) const;

private:
  // Returns distance along the ray to the first intersection, or INF if none.
  float rayCircle(
    float ox, float oy, float dx, float dy,
    float cx, float cy, float r,
    float range_min) const;

  float raySegment(
    float ox, float oy, float dx, float dy,
    float x1, float y1, float x2, float y2,
    float range_min) const;

  // Slab method: transform ray into OBB local frame, then AABB test.
  float rayOBB(
    float ox, float oy, float dx, float dy,
    const WorldOBB & obb,
    float range_min) const;
};

}  // namespace lidar_sim
