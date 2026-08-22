/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include "motion_model_base/collision_model/collision_footprint.hpp"

#include <Eigen/Dense>

#include <utility>

/**
 * \brief \ref CollisionFootPrint plugin representing an agent's
 * safety-margin footprint as an ellipse (major/minor axis lengths).
 */
class EllipseCollisionFootPrint : public CollisionFootPrint
{
  public:
	/// \brief Default constructor, required so pluginlib can instantiate this plugin.
	EllipseCollisionFootPrint() = default;
	EllipseCollisionFootPrint(const double majorAxisLength, const double minorAxisLength);
	~EllipseCollisionFootPrint() = default;
	/**
	 * \brief Configure a default-constructed (pluginlib-created) instance from YAML params.
	 * \param params Expects `params["major_axis_length"]` and `params["minor_axis_length"]`.
	 */
	void initialize(const YAML::Node & params) override;
	/// \brief Get the 2x2 matrix form of the ellipse footprint (for quadratic-form containment/collision checks).
	Eigen::Matrix2f getEllipseMatrix() const;
	/// \brief Get the center of the ellipse.
	Eigen::Vector2f getCenter();
	/// \brief Detect collision between two ellipse-shaped collision footprints.
	/// \return `{penetration/distance metric, colliding}`.
	std::pair<double, bool> detectCollision(const std::shared_ptr<CollisionFootPrint> object1,
	    const std::shared_ptr<CollisionFootPrint> object2);
	/// \brief Check whether \p pose lies within the ellipse.
	bool contains(const std::shared_ptr<stPose> & pose) const;
	void step(const stPose & pose) override;
	ShapeDescriptor describe() const override;

  private:
	std::shared_ptr<stPose> mPose{nullptr};
	double mMajorAxisLength{0.0};
	double mMinorAxisLength{0.0};
	double mSemiMajorAxisLength{0.0};
	double mSemiMinorAxisLength{0.0};
};
