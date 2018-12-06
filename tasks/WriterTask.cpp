/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WriterTask.hpp"
#include <canopen_master/SDO.hpp>

using namespace motors_elmo_ds402;

WriterTask::WriterTask(std::string const& name)
    : WriterTaskBase(name)
{
}

WriterTask::WriterTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WriterTaskBase(name, engine)
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
    readSDOs(queryFactors, UPDATE_FACTORS, base::Time::fromSeconds(2));
    mController.setMotorParameters(_motor_parameters.get());

    mJoints.resize(1);

    toNMTState(canopen_master::NODE_PRE_OPERATIONAL,
        canopen_master::NODE_ENTER_PRE_OPERATIONAL,
        base::Time::fromSeconds(1));

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
    writeSDOs(pdoSetup, base::Time::fromSeconds(2));

    toNMTState(canopen_master::NODE_OPERATIONAL,
        canopen_master::NODE_START,
        base::Time::fromSeconds(1));

    return true;
}
bool WriterTask::startHook()
{
    if (! WriterTaskBase::startHook())
        return false;

    try {
        resetCurrentCommand();
        writeSDO(mController.send(ControlWord(ControlWord::ENABLE_OPERATION, false)));
    }
    catch(...) {
        writeSDO(mController.send(ControlWord(ControlWord::SHUTDOWN, true)));
        writeSDO(mController.send(ControlWord(ControlWord::SWITCH_ON, true)));
        throw;
    }
    return true;
}

void WriterTask::resetCurrentCommand()
{
    mJoints.elements[0].speed = 0;
    mJoints.elements[0].effort = 0;
    mController.setControlTargets(mJoints.elements[0]);
    _can_out.write(mController.getRPDOMessage(0));
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
        auto msg = mController.getRPDOMessage(0);
        _latency.write(base::Time::now() - msg.time);
        _can_out.write(msg);
    }
}
void WriterTask::errorHook()
{
    WriterTaskBase::errorHook();
}
void WriterTask::stopHook()
{
    resetCurrentCommand();
    writeSDO(mController.send(ControlWord(ControlWord::FAULT_RESET, true)));
    writeSDO(mController.send(ControlWord(ControlWord::SHUTDOWN, true)));
    writeSDO(mController.send(ControlWord(ControlWord::SWITCH_ON, true)));
    WriterTaskBase::stopHook();
}
void WriterTask::cleanupHook()
{
    WriterTaskBase::cleanupHook();
}
