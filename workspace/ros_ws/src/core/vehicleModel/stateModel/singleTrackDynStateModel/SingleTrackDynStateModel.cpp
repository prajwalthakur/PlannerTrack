#include "SingleTrackDynStateModel.h"

////////////////////////////////////////////////////////////////////////////////

SingleTrackDynStateModel::SingleTrackDynStateModel(YAML::Node& simConfig, YAML::Node& vehConfig, const UniqueId& id)
    : BaseType(simConfig, vehConfig, id)
{

    NX = mVehConfig["numStates"].as<int>();
    NU = mVehConfig["numControlInputs"].as<int>();
    mInitXPose = mVehConfig["initXPose"].as<double>();
    mInitYPose = mVehConfig["initYPose"].as<double>();
    mInitYaw = mVehConfig["initYaw"].as<double>();

    mInitVx = mVehConfig["initVx"].as<double>();
    mInitSf = mVehConfig["initSf"].as<double>();
    mInitAcc = mVehConfig["initAcc"].as<double>();
    mInitSv = mVehConfig["initSv"].as<double>();
    mVehWheelBase = mVehConfig["vehWheelBase"].as<double>();
}

////////////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::createIntegrator()
{
    createIntegrator(mSimConfig, mVehConfig);
    reset();
    updateCommandedControl();
}

////////////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::createIntegrator(YAML::Node& simConfig, YAML::Node& vehYamlConfig)
{
    double intTimeStep = vehYamlConfig["integration"]["integrationStepSize"].as<double>();
    double simTimeStep = simConfig["simTimeStep"].as<double>();
    auto selfPtr = shared_from_this();
    mIntegrator = std::make_shared<IntegratorClass>
                (
                    [selfPtr](const StateVector& state, const InputVector& input)->StateVector
                    {
                        return selfPtr->xdot(state,input);
                    },
                    [selfPtr]()-> const StateVector&
                    {
                        return selfPtr->getState();
                    },
                    [selfPtr](const StateVector& state) -> void
                    {
                        return selfPtr->setState(state);
                    },
                    [selfPtr](const InputVector& input)-> void
                    {
                        return selfPtr->setInput(input);
                    },
                    intTimeStep,
                    simTimeStep
                );
}

////////////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::reset(){

    mStateStruct.x = mInitXPose;
    mStateStruct.y = mInitYPose;
    mStateStruct.yaw = mInitYaw;
    mStateStruct.vx = mInitVx;
    mStateStruct.sf = mInitSf;
    mInputStruct.acc = mInitAcc;
    mInputStruct.sv = mInitSv;

    mStateVector.resize(NX);
    mInputVector.resize(NU);

    mStateVector(0) = mStateStruct.x;
    mStateVector(1) = mStateStruct.y;
    mStateVector(2) = mStateStruct.yaw;
    mStateVector(3) = mStateStruct.vx;
    mStateVector(4) = mStateStruct.sf;

    mInputVector(0) = mInitAcc;
    mInputVector(1) = mInitSv;

}

////////////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::setState(const StateVector & statevector){
    mStateVector = statevector; 
    mStateStruct.x = mStateVector(0);
    mStateStruct.y = mStateVector(1);
    mStateStruct.yaw = mStateVector(2);
    mStateStruct.vx = mStateVector(3);
    mStateStruct.sf = mStateVector(4);
    std::cerr << " updated states " <<  mStateStruct.x << " " << mStateStruct.y << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::setInput(const InputVector & input_vector){
    mInputVector = input_vector;
    mInputStruct.acc = mInputVector(0);
    mInputStruct.sv = mInputVector(1);
}

////////////////////////////////////////////////////////////////////////////////

const StateVector& SingleTrackDynStateModel::getState() const {
    return mStateVector;
}

////////////////////////////////////////////////////////////////////////////////

const StateVector& SingleTrackDynStateModel::getInput() const {
    return mInputVector;
}


////////////////////////////////////////////////////////////////////////////////

StateVector SingleTrackDynStateModel::xdot(const StateVector& statevector, const InputVector& inputvector) const
{

    StateVector statevector_dot;
    statevector_dot.resize(NX);

    auto xk = this->VectorToState(statevector);
    auto uk = this->VectorToInput(inputvector);
    //xdot,ydot,yawdot,vfodt,sfdot
    statevector_dot(0) = xk.vx*std::cos(xk.yaw);
    statevector_dot(1) = xk.vx*std::sin(xk.yaw);
    statevector_dot(2) = xk.vx*std::tan(xk.sf)/mVehWheelBase;
    statevector_dot(3) = uk.acc;
    statevector_dot(4) = uk.sv;
    return statevector_dot;
}

////////////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::updateCommandedControl()
{
    mCommandedControl = mInputVector;
}

////////////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::updateCommandedControl(const InputVector& u )
{
    mCommandedControl = u;
}

////////////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::step()
{
   mIntegrator->simNextState(mCommandedControl); // state space step
    
}

////////////////////////////////////////////////////////////////////////////////

StateVector SingleTrackDynStateModel::StateToVector(const StateStruct & state_struct) const{
    StateVector state_vector;
    state_vector(0) = state_struct.x;
    state_vector(1) = state_struct.y;
    state_vector(2) = state_struct.yaw;
    state_vector(3) = state_struct.vx;
    state_vector(4) = state_struct.sf;
    return state_vector;
}

////////////////////////////////////////////////////////////////////////////////

InputVector SingleTrackDynStateModel::InputToVector(const InputStruct& input_struct) const{
    InputVector input_vector;
    input_vector(0) = input_struct.sv;
    input_vector(1) = input_struct.acc;
    return input_vector;

}

////////////////////////////////////////////////////////////////////////////////

StateStruct SingleTrackDynStateModel::VectorToState(const StateVector& statevector) const{
    StateStruct st;
    st.x = statevector(0);
    st.y = statevector(1);
    st.yaw = statevector(2);
    st.vx = statevector(3);
    st.sf = statevector(4);
    return st;
}

////////////////////////////////////////////////////////////////////////////////

stPose SingleTrackDynStateModel::getStatePose() const
{
    
    stPose st = stPose( mStateStruct.x, mStateStruct.y, mStateStruct.z, mStateStruct.yaw, mStateStruct.sf ) ;
    return st;
}

////////////////////////////////////////////////////////////////////////////////

InputStruct SingleTrackDynStateModel::VectorToInput(const InputVector& inputvector) const{
    InputStruct input;
    input.acc = inputvector(0);
    input.sv = inputvector(1);
    return input;
}

////////////////////////////////////////////////////////////////////////////////
