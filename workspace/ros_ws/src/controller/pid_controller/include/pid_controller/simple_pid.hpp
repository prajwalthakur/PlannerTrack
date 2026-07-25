/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#pragma once
#include <functional>

template <class T>
class PIDController
{
public:
  PIDController(double p, double i, double d, std::function<T()> pidSource, std::function<void(T output)> pidOutput);
  T tick();
  // Same as tick(), but integral/derivative use the caller-supplied dt (in
  // seconds) directly instead of registerTimeFunction()'s long/unsigned-long
  // clock. That API can only carry whole-unit resolution (e.g. whole
  // milliseconds), so ki/kd end up scaled by an awkward, hard-to-reason-about
  // factor of the clock's tick size; passing dt explicitly keeps ki/kd in
  // true per-second units and independent of how often tick() is called.
  T tick(double dt);
  void setTarget(T t);
  T getTarget();
  T getOutput();
  T getFeedback();
  T getError();
  void setEnabled(bool e);
  bool isEnabled();
  T getProportionalComponent();
  T getIntegralComponent();
  T getDerivativeComponent();
  void setMaxIntegralCumulation(T max);
  T getMaxIntegralCumulation();
  T getIntegralCumulation();

  void setInputBounded(bool bounded);
  bool isInputBounded();
  void setInputBounds(T lower, T upper);
  T getInputLowerBound();
  T getInputUpperBound();
  void setOutputBounded(bool bounded);
  bool isOutputBounded();
  void setOutputBounds(T lower, T upper);
  T getOutputLowerBound();
  T getOutputUpperBound();
  void setFeedbackWrapped(bool wrap);
  bool isFeedbackWrapped();
  void setFeedbackWrapBounds(T lower, T upper);
  T getFeedbackWrapLowerBound();
  T getFeedbackWrapUpperBound();

  void setPID(double p, double i, double d);
  void setP(double p);
  void setI(double i);
  void setD(double d);
  double getP();
  double getI();
  double getD();
  void setPIDSource(T (*pidSource)());
  void setPIDOutput(void (*pidOutput)(T output));
  void registerTimeFunction(unsigned long (*getSystemTime)());
private:
  double _p;
  double _i;
  double _d;
  T target;
  T output;
  bool enabled;
  T currentFeedback;
  T lastFeedback;
  T error;
  T lastError;
  long currentTime;
  long lastTime;
  T integralCumulation;
  T maxCumulation;
  T cycleDerivative;

  bool inputBounded;
  T inputLowerBound;
  T inputUpperBound;
  bool outputBounded;
  T outputLowerBound;
  T outputUpperBound;
  bool feedbackWrapped;
  T feedbackWrapLowerBound;
  T feedbackWrapUpperBound;

  bool timeFunctionRegistered;
  std::function<T()> _pidSource;
  std::function<void(T output)> _pidOutput;
  unsigned long (*_getSystemTime)();
};