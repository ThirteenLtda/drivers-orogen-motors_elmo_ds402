/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ReaderTask.hpp"

using namespace motors_elmo_ds402;

ReaderTask::ReaderTask(std::string const& name)
    : ReaderTaskBase(name)
{
}

ReaderTask::ReaderTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ReaderTaskBase(name, engine)
{
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
    return true;
}
bool ReaderTask::startHook()
{
    if (! ReaderTaskBase::startHook())
        return false;
    return true;
}
void ReaderTask::updateHook()
{
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
