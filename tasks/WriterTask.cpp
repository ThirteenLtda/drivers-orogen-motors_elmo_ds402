/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WriterTask.hpp"
#include <canopen_master/SDO.hpp>

using namespace motors_elmo_ds402;

WriterTask::WriterTask(std::string const& name)
    : WriterTaskBase(name)
    , mController(0)
{
}

WriterTask::WriterTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WriterTaskBase(name, engine)
    , mController(0)
{
}

WriterTask::~WriterTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See WriterTask.hpp for more detailed
// documentation about them.

bool WriterTask::configureHook()
{
    if (! WriterTaskBase::configureHook())
        return false;

    mController = Controller(_can_id.get());
    auto queryFactors = mController.queryFactors();
    if (!readSDOs(queryFactors, UPDATE_FACTORS, base::Time::fromSeconds(2)))
        return false;
    mController.setMotorParameters(_motor_parameters.get());

    mJoints.resize(1);

    _can_out.write(mController.queryNodeStateTransition(
        canopen_master::NODE_ENTER_PRE_OPERATIONAL));
    writeSDO(mController.send(ControlWord(ControlWord::SHUTDOWN, true)));

    switch(_control_mode.get())
    {
        case base::JointState::POSITION:
            writeSDO(mController.setOperationMode(OPERATION_MODE_CYCLIC_SYNCHRONOUS_POSITION));
            break;
        case base::JointState::SPEED:
            writeSDO(mController.setOperationMode(OPERATION_MODE_CYCLIC_SYNCHRONOUS_VELOCITY));
            break;
        case base::JointState::EFFORT:
            writeSDO(mController.setOperationMode(OPERATION_MODE_PROFILED_TORQUE));
            break;
        default:
            throw std::invalid_argument("invalid control mode");
    }
    auto pdoSetup = mController.configureControlPDO(
        0, _control_mode.get(), _rpdo_configuration.get());
    if (!writeSDOs(pdoSetup, base::Time::fromSeconds(2)))
        return false;

    _can_out.write(mController.queryNodeStateTransition(
        canopen_master::NODE_START));
    return true;
}
bool WriterTask::startHook()
{
    if (! WriterTaskBase::startHook())
        return false;

    writeSDO(mController.send(ControlWord(ControlWord::SWITCH_ON, true)));

    resetCurrentCommand();
    return true;
}

void WriterTask::resetCurrentCommand()
{
    mJoints.elements[0].speed = 0;
    mJoints.elements[0].effort = 0;
    mController.setControlTargets(mJoints.elements[0]);
    _can_out.write(mController.getRPDOMessage(0));
    writeSDO(mController.send(ControlWord(ControlWord::ENABLE_OPERATION, false)));
}

void WriterTask::updateHook()
{
    WriterTaskBase::updateHook();

    while (_joint_command.read(mJoints, false) == RTT::NewData)
    {
        if (mJoints.size() != 1)
            return exception(INVALID_COMMAND);
        base::JointState const& state = mJoints.elements[0];
        if (base::isUnset(state.getField(_control_mode.get())))
            return exception(INVALID_COMMAND);

        mController.setControlTargets(mJoints.elements[0]);
        _can_out.write(mController.getRPDOMessage(0));
    }
}
void WriterTask::errorHook()
{
    WriterTaskBase::errorHook();
}
void WriterTask::stopHook()
{
    writeSDO(mController.send(ControlWord(ControlWord::SHUTDOWN, true)));
    resetCurrentCommand();
    WriterTaskBase::stopHook();
}
void WriterTask::cleanupHook()
{
    WriterTaskBase::cleanupHook();
}

bool WriterTask::readSDOs(std::vector<canbus::Message> const& queries,
    int expectedUpdate, base::Time timeout)
{
    for (auto const& query : queries) {
        if (!readSDO(query, expectedUpdate, timeout))
            return false;
    }
    return true;
}
bool WriterTask::readSDO(canbus::Message const& query,
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

bool WriterTask::writeSDOs(std::vector<canbus::Message> const& queries,
    base::Time timeout)
{
    for (auto const& query : queries) {
        if (!writeSDO(query, timeout))
            return false;
    }
    return true;
}
bool WriterTask::writeSDO(canbus::Message const& query, base::Time timeout)
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
