/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ReaderTask.hpp"
#include <canopen_master/SDO.hpp>

using namespace motors_elmo_ds402;

ReaderTask::ReaderTask(std::string const& name)
    : ReaderTaskBase(name)
    , mController(0)
{
    mJoints.elements.resize(1);
}

ReaderTask::ReaderTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ReaderTaskBase(name, engine)
    , mController(0)
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
    if (!readSDOs(queryFactors, UPDATE_FACTORS, base::Time::fromSeconds(2)))
        return false;
    mController.setMotorParameters(_motor_parameters.get());

    _can_out.write(mController.queryNodeStateTransition(
        canopen_master::NODE_ENTER_PRE_OPERATIONAL));
    canopen_master::PDOCommunicationParameters parameters;
    auto pdoSetup = mController.queryPeriodicJointStateUpdate(
            0, _rpdo_configuration.get(), mExpectedJointState);
    if (!writeSDOs(pdoSetup, base::Time::fromSeconds(2)))
        return false;
    return true;
}

bool ReaderTask::startHook()
{
    if (! ReaderTaskBase::startHook())
        return false;
    _can_out.write(mController.queryNodeStateTransition(
        canopen_master::NODE_START));
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

bool ReaderTask::readSDOs(std::vector<canbus::Message> const& queries,
    int expectedUpdate, base::Time timeout)
{
    for (auto const& query : queries) {
        if (!readSDO(query, expectedUpdate, timeout))
            return false;
    }
    return true;
}
bool ReaderTask::readSDO(canbus::Message const& query,
    int expectedUpdate, base::Time timeout)
{
    _can_out.write(query);

    base::Time deadline = base::Time::now() + timeout;
    while(true)
    {
        canbus::Message msg;
        if (_can_in.read(msg, false) == RTT::NewData) {
            if (mController.process(msg).isUpdated(expectedUpdate)) {
                return true;
            }
        }
        if (base::Time::now() > deadline) {
            exception(SDO_TIMED_OUT);
            return false;
        }
        usleep(10);
    }
    // Never reached
    return false;
}

bool ReaderTask::writeSDOs(std::vector<canbus::Message> const& queries,
    base::Time timeout)
{
    for (auto const& query : queries) {
        if (!writeSDO(query, timeout))
            return false;
    }
    return true;
}
bool ReaderTask::writeSDO(canbus::Message const& query, base::Time timeout)
{
    _can_out.write(query);

    uint16_t objectId = canopen_master::getSDOObjectID(query);
    uint16_t objectSubId = canopen_master::getSDOObjectSubID(query);

    base::Time deadline = base::Time::now() + timeout;
    while(true)
    {
        canbus::Message msg;
        if (_can_in.read(msg, false) == RTT::NewData) {
            if (mController.process(msg).isAcked(objectId, objectSubId)) {
                return true;
            }
        }
        if (base::Time::now() > deadline) {
            exception(SDO_TIMED_OUT);
            return false;
        }
        usleep(10);
    }
    // Never reached
    return false;
}
