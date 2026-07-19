// Author Prajwal Thakur
#include "project_utils/integrator.hpp"

//////////////////////////////////////////////////////////////////////////

IntegratorClass::IntegratorClass(
    std::function<mt::StateVector(const mt::StateVector &, const mt::InputVector &)> dynamics,
    std::function<const mt::StateVector &()> getState,
    std::function<void(const mt::StateVector &)> setState,
    std::function<void(const mt::InputVector &)> setInput, double integrationTimeStep)
    : mDynamicsFunc(dynamics),
      mSetStateFunc(setState),
      mSetInputFunc(setInput),
      mGetStateFunc(getState),
      mIntegrationStepSize(integrationTimeStep)
{
}

//////////////////////////////////////////////////////////////////////////

IntegratorClass::IntegratorClass(
    std::function<mt::StateVector(const mt::StateVector &, const mt::InputVector &)> dynamics,
    std::function<const mt::StateVector &()> getState,
    std::function<void(const mt::StateVector &)> setState,
    std::function<void(const mt::InputVector &)> setInput, double integrationTimeStep,
    double simTimeStep)
    : mDynamicsFunc(dynamics),
      mSetStateFunc(setState),
      mSetInputFunc(setInput),
      mGetStateFunc(getState),
      mIntegrationStepSize(integrationTimeStep),
      mSimStepSize(simTimeStep)
{
}

//////////////////////////////////////////////////////////////////////////

mt::StateVector IntegratorClass::rk4Integrator(
    const mt::StateVector & x, const mt::InputVector & u, double ts) const
{
	mt::StateVector k1 = mDynamicsFunc(x, u);
	mt::StateVector k2 = mDynamicsFunc(x + (ts / 2.) * k1, u);
	mt::StateVector k3 = mDynamicsFunc(x + (ts / 2.) * k2, u);
	mt::StateVector k4 = mDynamicsFunc(x + (ts / 2.) * k3, u);
	mt::StateVector x_next = x + ts * (k1 / 6. + k2 / 3. + k3 / 3. + k4 / 6.);
	// std::cerr << " x_next " << x_next << std::endl;
	return x_next;
}

//////////////////////////////////////////////////////////////////////////

mt::StateVector IntegratorClass::efIntegrator(
    const mt::StateVector & x, const mt::InputVector & u, double ts) const
{
	mt::StateVector k1 = mDynamicsFunc(x, u);
	mt::StateVector x_next = x + ts * (k1);
	return x_next;
}

//////////////////////////////////////////////////////////////////////////

void IntegratorClass::simNextState(const mt::InputVector & u, double ts) const
{
	mt::StateVector x_next = this->mGetStateFunc();
	const int integration_steps = (int)(ts / this->mIntegrationStepSize);
	for (int i = 0; i < integration_steps; i++) {
		x_next = this->rk4Integrator(x_next, u, mIntegrationStepSize);
	}
	// RCLCPP_INFO(this->get_logger(), "rk4 called");
	this->mSetStateFunc(x_next);
	this->mSetInputFunc(u);
}

//////////////////////////////////////////////////////////////////////////

void IntegratorClass::simNextState(const mt::InputVector & u) const
{
	mt::StateVector x_next = this->mGetStateFunc();
	const int integration_steps = (int)(mSimStepSize / this->mIntegrationStepSize);
	for (int i = 0; i < integration_steps; i++) {
		x_next = this->rk4Integrator(x_next, u, mIntegrationStepSize);
	}
	// RCLCPP_INFO(this->get_logger(), "rk4 called");
	// std::cerr << " rk4 integrator called "  << " ";
	this->mSetStateFunc(x_next);
	// std::cerr << " lo "  << " ";
	this->mSetInputFunc(u);
}

////////////////////////////////////////////////////////////////////////////////
