#include "Models.h"
#include "Utils.h"
#include <iostream>

VTOL::VTOL( VTOLMake make ) : state( FLYING ), make( make )
{
    switch( make )
    {
        case ALPHA:
            Init( 120, 320, 0.6, 1.6, 4, 0.25 );
            break;
        case BETA:
            Init( 100, 100, 0.2, 1.5, 5, 0.10 );
            break;
        case CHARLIE:
            Init( 160, 220, 0.8, 2.2, 3, 0.05 ); 
            break;
        case DELTA:
            Init( 90, 120, 0.62, 0.8, 2, 0.22 );
            break;
        case ECHO:
            Init( 30, 150, 0.3, 5.8, 2, 0.61 );
            break;
    }
}

void VTOL::Init( int speed, int batteryCapacity, double chargeTime, double kwhPerMile, int passengerCapacity, double faultProbability )
{
    this->speed = speed;
    this->chargeTime = chargeTime;
    this->drainTime = ( batteryCapacity / kwhPerMile ) / speed;
    this->passengerCapacity = passengerCapacity;
    this->faultProbability = faultProbability;
    this->timeToStateChange = drainTime;
    this->state = FLYING;
}

/**
 *  @brief advance the simulated state of the VTOL by a given amount of time
 *  @param dTime the time in hours to advance the simulation
 */
double VTOL::updateVTOL( double dTime, double faultRoll )
{
    // check if state needs to change this tick
    double timeInStartState = ( timeToStateChange > 0 ? std::min( dTime, timeToStateChange ) : dTime );
    
    // advance the time of the VTOL by the time spent in the initial state
    timeToStateChange -= timeInStartState;
    switch( state )
    {
        case FLYING:
            timeFlying += timeInStartState;
            if( hadFault( timeInStartState, faultRoll ) )
                numFaults += 1;
            break;
        case CHARGING:
            timeCharging += timeInStartState;
            break;
        case WAITING:
            timeWaiting += timeInStartState;
            break;
    }

    // if state changes this tick handle the state change and advance time spent in that state by remaining time
    if( dTime > timeInStartState || almostEqual( timeToStateChange, 0 ) )
    {
        timeInStateThisTick = dTime - timeInStartState;
        switch( state)
        {
            case FLYING:
                setState( WAITING );
                timeToStateChange = UNLIMITED;
                timeWaiting += timeInStateThisTick;
                break;
            case CHARGING:
                setState( FLYING );
                timeToStateChange = drainTime - timeInStateThisTick;
                timeFlying += timeInStateThisTick;
                if( hadFault( timeInStateThisTick, faultRoll ) )
                    numFaults += 1;
                break;
        }
    }
    else
    {
        timeInStateThisTick = timeInStartState;
    }
    
    return timeInStateThisTick;
}

/**
 *  @brief adjust the wait time of the aircraft for when charging station becomes available
 *  @param dTime the time in hours the charging station was available for use last tick
 */
void VTOL::moveToCharger( double dTime )
{
    // determine the amount of time that both the station was available for use and the VTOL was ready to charge
    double dAdjustTime = std::min( dTime, timeInStateThisTick );
    timeWaiting -= dAdjustTime;
    setState( CHARGING );
    updateVTOL( dAdjustTime, 1.0 );
}

void VTOL::setState( VTOLStatus state )
{
    this->state = state;
    switch( state )
    {
        case FLYING:
            timeToStateChange = drainTime;
            break;
        case CHARGING:
            timeToStateChange = chargeTime;
            break;
        case WAITING:
            timeToStateChange = UNLIMITED;
            break;
    }
}

bool VTOLQueue::push( VTOL * VTOL )
{
    if( full() )
        return false;
    
    q.push_back( VTOL );
    return true;
}

VTOL * VTOLQueue::pop()
{
    VTOL * VTOL = q.front();
    q.pop_front();
    return VTOL;
}

VTOL * VTOLQueue::getNextVTOL()
{
    if( it == q.end() )
    {
        it = q.begin();
    }
    else
    {
        if( ++it == q.end() )
        {
            return nullptr;
        }
    }

    return *it;
}

bool VTOL::hadFault( double dTime, double faultRoll )
{
    return faultRoll < dTime * faultProbability;
}

int VTOLQueue::size()
{
    return q.size();
}

bool VTOLQueue::empty()
{
    return q.empty();
}

bool VTOLQueue::full()
{
    return capacity != UNLIMITED && q.size() == capacity;
}

double VTOLQueue::getFaultRoll()
{
    if( !generator )
    {
        return 1.0;
    }
    
    std::uniform_real_distribution<double> distribution( 0.0, 1.0 );
    return distribution( *generator );
}

void VTOLQueue::initGenerator()
{
    generator = new std::default_random_engine( clock() );
}