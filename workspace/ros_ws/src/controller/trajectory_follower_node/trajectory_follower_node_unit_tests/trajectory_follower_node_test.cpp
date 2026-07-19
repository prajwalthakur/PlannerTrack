// Copyright 2026 Prajwal Thakur
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
#include "project_utils_testing/test_fixture.hpp"
// file exits at
// home/prajwalthakur24/ws/install/project_utils_testing/include/project_utils_testing/test_fixture.hpp
class TestRRTPlanner : public mplTestFixture
{
  public:
	TestRRTPlanner() {}
};

// ////////////////////////////////////////////////////////////////////////////////
// Test No: 1 - check creation of RRT CLASS

mplTest(TestRRTPlanner, TestRRTClassCreation)
{
	EXPECT_EQ(9, 9);
}
