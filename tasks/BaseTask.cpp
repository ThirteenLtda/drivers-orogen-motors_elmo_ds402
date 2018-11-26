/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "BaseTask.hpp"
#include <canopen_master/SDO.hpp>

using namespace motors_elmo_ds402;

BaseTask::BaseTask(std::string const& name)
    : BaseTaskBase(name)
    , mController(0)
{
}

BaseTask::BaseTask(std::string const& name, RTT::ExecutionEngine* engine)
    : BaseTaskBase(name, engine)
    , mController(0)
{
}

BaseTask::~BaseTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See BaseTask.hpp for more detailed
// documentation about them.

bool BaseTask::configureHook()
{
    if (! BaseTaskBase::configureHook())
        return false;
    return true;
}
bool BaseTask::startHook()
{
    if (! BaseTaskBase::startHook())
        return false;
    return true;
}
void BaseTask::updateHook()
{
    BaseTaskBase::updateHook();
}
void BaseTask::errorHook()
{
    BaseTaskBase::errorHook();
}
void BaseTask::stopHook()
{
    BaseTaskBase::stopHook();
}
void BaseTask::cleanupHook()
{
    BaseTaskBase::cleanupHook();
}

static const int POLL_PERIOD_US = 100;
static const int NMT_DEAD_TIME_US = 10000;

canopen_master::NODE_STATE BaseTask::getNMTState(base::Time deadline)
{
    base::Time current = mController.timestamp<CANControllerStatus>();
    _can_out.write(mController.queryCANControllerStatus());
    while (mController.timestamp<CANControllerStatus>() != current) {
        canbus::Message msg;
        if (_can_in.read(msg, false) == RTT::NewData) {
            mController.process(msg);
        }
        if (base::Time::now() > deadline) {
            exception(NMT_TIMED_OUT);
            throw std::runtime_error("NMT state timed out");
        }
        usleep(POLL_PERIOD_US);
    }

    return mController.get<CANControllerStatus>().nodeState;
}

void BaseTask::toNMTState(canopen_master::NODE_STATE desiredState,
    canopen_master::NODE_STATE_TRANSITION transition,
    base::Time timeout)
{
    base::Time deadline = base::Time::now() + timeout;
    auto currentState = getNMTState(deadline);
    if (currentState == desiredState)
        return;

    _can_out.write(mController.queryNodeStateTransition(transition));
    usleep(NMT_DEAD_TIME_US);
    while (currentState != getNMTState(deadline)) {
        _can_out.write(mController.queryNodeStateTransition(transition));
        usleep(NMT_DEAD_TIME_US);
    }
}

void BaseTask::readSDOs(std::vector<canbus::Message> const& queries,
    int expectedUpdate, base::Time timeout)
{
    for (auto const& query : queries) {
        readSDO(query, expectedUpdate, timeout);
    }
}
void BaseTask::readSDO(canbus::Message const& query,
    int expectedUpdate, base::Time timeout)
{
    _can_out.write(query);

    base::Time deadline = base::Time::now() + timeout;
    while(true)
    {
        canbus::Message msg;
        if (_can_in.read(msg, false) == RTT::NewData) {
            if (mController.process(msg).isUpdated(expectedUpdate)) {
                return;
            }
        }
        if (base::Time::now() > deadline) {
            exception(SDO_TIMED_OUT);
            throw std::runtime_error("Reading SDO Timed Out");
        }
        usleep(POLL_PERIOD_US);
    }
}

void BaseTask::writeSDOs(std::vector<canbus::Message> const& queries,
    base::Time timeout)
{
    for (auto const& query : queries) {
        writeSDO(query, timeout);
    }
}
void BaseTask::writeSDO(canbus::Message const& query, base::Time timeout)
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
                return;
            }
        }
        if (base::Time::now() > deadline) {
            exception(SDO_TIMED_OUT);
            throw std::runtime_error("Writing SDO Timed Out");
        }
        usleep(POLL_PERIOD_US);
    }
}
