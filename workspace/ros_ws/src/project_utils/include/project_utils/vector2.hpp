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
#include <Eigen/Dense>
using Vector2 = Eigen::Vector2f;

constexpr float RVO_EPSILON = 1e-5f;

// Utility functions (only ones Eigen doesn't directly provide)

inline float normVec(const Vector2 & v)
{
	return v.norm();
}

inline float normVecSq(const Vector2 & v)
{
	return v.squaredNorm();
}

inline float det(const Vector2 & u, const Vector2 & v)
{
	/*signed area of the parallelogram formed by u and v*/
	/*positive: v is left of u, negative: v is right of u*/
	return u.x() * v.y() - u.y() * v.x();
}

inline float leftOf(const Vector2 & a, const Vector2 & b, const Vector2 & c)
{
	return det(b - a,c - a);  // return positive value if c is left of b-a, 0 if co-linear
}

inline float signedDist(const Vector2 & a, const Vector2 & b, const Vector2 & c)
{
	/*distance from point C to line AB*/
	return det( b - a,c - a) / normVec(b - a);
}

inline Vector2 normalize(const Vector2 & v)
{
	return v.normalized();
}

inline float dot(const Vector2 & u, const Vector2 & v)
{
	return u.x() * v.x() + u.y() * v.y();
}

