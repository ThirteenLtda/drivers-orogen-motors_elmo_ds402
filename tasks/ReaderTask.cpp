/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ReaderTask.hpp"
#include <canopen_master/SDO.hpp>
#include <base-logging/Logging.hpp>

using namespace motors_elmo_ds402;

ReaderTask::ReaderTask(std::string const& name)
    : ReaderTaskBase(name)
{
    mJoints.elements.resize(1);
}

ReaderTask::ReaderTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ReaderTaskBase(name, engine)
{
    mJoints.elements.resize(1);
}

ReaderTask::~ReaderTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ReaderTask.hpp for more detailed
// documentation about them.

bool ReaderTask::configureHook()
{
    if (! ReaderTaskBase::configureHook())
        return false;

    mController = Controller(_can_id.get());
    mExpectedJointState = 0;
    auto expected_joint_state = _required_joint_states.get();
    if (expected_joint_state.position)
        mExpectedJointState |= UPDATE_JOINT_POSITION;
    if (expected_joint_state.velocity)
        mExpectedJointState |= UPDATE_JOINT_VELOCITY;
    if (expected_joint_state.current_and_torque)
        mExpectedJointState |= UPDATE_JOINT_CURRENT;

    auto queryFactors = mController.queryFactors();
    readSDOs(queryFactors, UPDATE_FACTORS, base::Time::fromSeconds(2));
    mController.setMotorParameters(_motor_parameters.get());
    mController.setZeroPosition(_zero_position.get());

    toNMTState(canopen_master::NODE_PRE_OPERATIONAL,
        canopen_master::NODE_ENTER_PRE_OPERATIONAL,
        base::Time::fromSeconds(1));

    canopen_master::PDOCommunicationParameters parameters;
    auto pdoSetup = mController.configureJointStateUpdatePDOs(
        2, _tpdo_configuration.get(), mExpectedJointState);
    writeSDOs(pdoSetup, base::Time::fromSeconds(2));
    toNMTState(canopen_master::NODE_OPERATIONAL,
        canopen_master::NODE_START,
        base::Time::fromSeconds(1));
    return true;
}

bool ReaderTask::startHook()
{
    if (! ReaderTaskBase::startHook())
        return false;
    mUpdate = Update();
    mJoints.time = base::Time();
    mJoints.elements[0] = base::JointState();
    return true;
}
void ReaderTask::updateHook()
{
    canbus::Message msg;
    while (_can_in.read(msg, false) == RTT::NewData)
    {
        mUpdate.merge(mController.process(msg));
        if (mUpdate.isUpdated(mExpectedJointState)) {
            base::JointState state = mController.getJointState(mExpectedJointState);
            mJoints.time = msg.time;
            mJoints.elements[0] = state;
            _joints_state_samples.write(mJoints);

            if (mExpectedJointState & UPDATE_JOINT_POSITION) {
                RawEncoders sample;
                sample.time = msg.time;
                sample.value = mController.getRawPosition();
                _raw_encoder_samples.write(sample);
            }

            mUpdate = Update();
        }
    }

    ReaderTaskBase::updateHook();
}
void ReaderTask::errorHook()
{
    ReaderTaskBase::errorHook();
}
void ReaderTask::stopHook()
{
    ReaderTaskBase::stopHook();
}
void ReaderTask::cleanupHook()
{
    ReaderTaskBase::cleanupHook();
}
