#include "SingleTrackDynModel.h"

////////////////////////////////////////////////////////////////////////////////

SingleTrackDynModel::SingleTrackDynModel(YAML::Node& simConfig, YAML::Node& vehConfig, long int value)
:BaseType(simConfig, vehConfig)
{
    mId = UniqueId("vehicle",value);
    if(isStringEqual(mVehConfig["geomModelType"].as<std::string>(),"rectangular"))
    {
        mGeomModel  = std::make_shared<RectangularGeomClass>(mVehConfig["vehLength"].as<double>(),mVehConfig["vehWidth"].as<double>());

    }
    if(isStringEqual(mVehConfig["collFootPrintType"].as<std::string>(),"ellipse"))
    {
        double majorAxis = mVehConfig["collisionFootPrint"]["majorAxis"].as<double>();
        double minorAxis = mVehConfig["collisionFootPrint"]["minorAxis"].as<double>();
        ptSharedPtr<CollisionFootPrint> collFootPrint = std::make_shared<EllipseCollisionFootPrint>(majorAxis,minorAxis);
        mGeomModel->setCollisionFootPrint(collFootPrint);
    }
    std::cerr  << " trying to add mStateModel"<<std::endl;
    mStateModel = std::make_shared<SingleTrackDynStateModel>(mSimConfig, mVehConfig, mId);
    mStateModel->createIntegrator();
    std::cerr  << " trying to add numStates"<<std::endl;
    mNumStates = vehConfig["numStates"].as<int>();
    std::cerr  << " trying to add numControlInputs";
    mNumControlInputs = vehConfig["numControlInputs"].as<int>();
    
}

////////////////////////////////////////////////////////////////////////////////

void SingleTrackDynModel::step()
{
    mStateModel->step();
    StateVector st  = mStateModel->getState();
    double x = st(0);
    double y = st(1);
    double yaw = st(2);
    // double vx = st(3);
    //double sf = st(4);
    ptSharedPtr<stPose> pose  = std::make_shared<stPose>(x,y,0.0,yaw);
    mGeomModel->step(pose);
}

////////////////////////////////////////////////////////////////////////////////

StateVector SingleTrackDynModel::getState() const
{
    return mStateModel->getState();
}

////////////////////////////////////////////////////////////////////////////////

stPose SingleTrackDynModel::getStatePose() const
{
    return mStateModel->getStatePose();
}

////////////////////////////////////////////////////////////////////////////////

void SingleTrackDynModel::setState(const StateVector & stateVec) 
{
    mStateModel->setState(stateVec);
}

////////////////////////////////////////////////////////////////////////////////

bool SingleTrackDynModel::checkNumStatesInputs(const StateVector& stateVec, const InputVector& inputVec)
{
    if(stateVec.size()!=mNumStates)
        std::cerr<<"statevector size is not same as defined num of states" <<std::endl;
    if(inputVec.size()!=mNumControlInputs)
        std::cerr<<"input vector size is not same as defined num of inputs" <<std::endl;
    if(stateVec.size()!=mNumStates || inputVec.size()!=mNumControlInputs)
        return false;
    return true;
}

////////////////////////////////////////////////////////////////////////////////

std::pair<int,int> SingleTrackDynModel::getNumStatesInputs()
{
    return std::make_pair(mNumStates,mNumControlInputs);
}