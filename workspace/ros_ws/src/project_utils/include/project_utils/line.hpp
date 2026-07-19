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
#include "project_utils/vector2.hpp"
using Vector2 = Eigen::Vector2f;
#include <Eigen/Dense>

#include <cmath>
struct LineCoefficient
{
	float A;
	float B;
	float C;
};
class Line
{
  public:
	// Constructor
	Line() = default;

	Line(Vector2 point1, Vector2 point2)
	{
		point = point1;

		Vector2 vec = point2 - point1;
		auto dx = vec.x();
		auto dy = vec.y();

		slope = dy / dx;

		coeff.A = dy;
		coeff.B = -dx;
		coeff.C = dx * point.y() - dy * point.x();

		heading = atan2(dy, dx);
		direction = Vector2(1.0f, slope);
		direction.normalize();
	}

	Line(Vector2 point1, float heading_)
	{
		point = point1;
		heading = heading_;

		// optional, but avoid relying on slope for geometry
		slope = std::tan(heading);

		float dx = std::cos(heading);
		float dy = std::sin(heading);

		// General line form: Ax + By + C = 0
		coeff.A = dy;
		coeff.B = -dx;
		coeff.C = dx * point.y() - dy * point.x();
		direction = Vector2(1.0f, slope);
		direction.normalize();
	}

	// Destructor
	~Line() = default;

	Line perp()
	{
		Line line2;

		// same passing point
		line2.point = point;

		// perpendicular normal
		line2.coeff.A = -coeff.B;
		line2.coeff.B = coeff.A;

		// pass through point
		line2.coeff.C = -(line2.coeff.A * line2.point.x() + line2.coeff.B * line2.point.y());

		// safer heading from direction vector
		float dx = -line2.coeff.B;
		float dy = line2.coeff.A;

		line2.heading = std::atan2(dy, dx);

		// optional: avoid slope for vertical lines
		if (std::abs(line2.coeff.B) > 1e-6) {
			line2.slope = -line2.coeff.A / line2.coeff.B;
			line2.direction = Vector2(1.0f, line2.slope);
			line2.direction.normalize();
		} else {
			line2.slope = std::numeric_limits<float>::infinity();
			line2.direction = Vector2(0.0f, 1.0f);
			line2.direction.normalize();
		}
		return line2;
	}
	float slope{-1.0};
	float heading{-1.0};
	Vector2 point;
	Vector2 direction;
	LineCoefficient coeff;
};

class LineSegment : public Line
{
  public:
	LineSegment() = default;
	~LineSegment() = default;
	Vector2 mStartPoint;
	Vector2 mEndPoint;
	LineSegment(Vector2 start_point, Vector2 end_point)
	{
		mStartPoint = start_point;
		mEndPoint = end_point;
		mSupportingLine = Line(start_point, end_point);
	}
	Line mSupportingLine;  // a segment has a supporting line
};

inline std::pair<float, Vector2> projectionOnto(const Line & line, const Vector2 & point)
{
	float norm = line.direction.norm();
	if (norm < 1e-6f) {
		return {0.0f, line.point};
	}

	Vector2 unitW = line.direction.normalized();
	Vector2 vec2 = point - line.point;
	float t = static_cast<float>((unitW).dot(vec2));
	auto closestVec = line.point + t * (unitW);
	return std::make_pair(t, closestVec);
}
