#pragma once
#include "mppi_controller/models/constraints.hpp"
#include "mppi_controller/parameters.hpp"
namespace controller::mppi_controller::models
{
/// \brief \ref ControlConstraints for \ref AckermannModel -- velocity/acceleration bounds plus the minimum turning radius (used by \ref ConstraintCritic).
struct AckermannControlConstraints : public ControlConstraints
{
	~AckermannControlConstraints() override = default;
	/// \brief Load all bounds below from ROS params under \p name.
	void onConfigure(Parameters & parameters, const std::string & name) final;

	float vxMax;
	float vxMin;

	float wzMax;
	float wzMin;

	float axMax;
	float axMin;

	float azMax;
	float azMin;
	float minTurningRadius;
	float getMinTurningRadius() const { return minTurningRadius; }
	//////////////////////////////////////////////////////////////////////////
};

/// \brief \ref SamplingStd for \ref AckermannModel -- per-dimension Gaussian noise standard deviation used by \ref AckermannNoiseGenerator.
struct AckermannSamplingStd : public SamplingStd
{
	~AckermannSamplingStd() override = default;
	/// \brief Load \ref vx / \ref wz noise standard deviations from ROS params under \p name.
	void onConfigure(Parameters & parameters, const std::string & name) final;

	float vx;
	float wz;
};

}  // namespace controller::mppi_controller::models

namespace models = controller::mppi_controller::models;