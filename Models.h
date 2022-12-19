#ifndef MODELS_H
#define MODELS_H

#include <queue>
#include <deque>
#include <vector>
#include <cmath>
#include <random>
#include <thread>

using std::vector;
using std::deque;

#define UNLIMITED -1

enum VTOLStatus
{
    FLYING = 0,
    WAITING = 1,
    CHARGING = 2
};

enum VTOLMake
{
    ALPHA = 0,
    BETA = 1,
    CHARLIE = 2,
    DELTA = 3,
    ECHO = 4
};

class VTOL
{
    public:
        VTOL( VTOLMake make );
        ~VTOL() {}

        /**
         * @brief advance the state of the VTOL by a specified amount of time
         * @param dTime number of hours to advance the state
         * @param faultRoll the randomly generated value used to determine if the VTOL experienced a fault
         * @return the amount of time the VTOL spent in the state in which it ended the tick ( FLYING, CHARGING, or WAITING )
         */
        double updateVTOL( double dTime, double faultRoll );

        /**
         * @brief simulate moving the VTOL from the waiting queue to the charging queue
         * @param dTime the amount of time that the charger the VTOL is being moved to was available
         */
        void moveToCharger( double dTime );

        double getTimeInFlight() const { return timeFlying; }
        double getTimeWaiting() const { return timeWaiting; }
        double getTimeCharging() const { return timeCharging; }
        bool hadFault( double timeFlyingThisTick, double faultRoll );
        int getNumFaults() const { return static_cast<int>( std::ceil(numFaults) ); }
        double getPassengerMiles() const { return timeFlying * speed * passengerCapacity; }
        VTOLMake getMake() const { return make; }
        VTOLStatus getStatus() const  { return state; }
        void setState( VTOLStatus state );
        bool operator<( const VTOL & a ) { return timeInStateThisTick > a.timeToStateChange; } // allow descending sorting by time since state change for adding to wait queue
        bool operator>( const VTOL & a ) { return timeInStateThisTick < a.timeToStateChange; } // allow ascending sorting by time since state change for adding to wait queue
    private:
        void Init( int speed, int batteryCapacity, double chargeTime, double kwhPerMile, int passengerCapacity, double faultProbability );
        VTOLStatus state;
        VTOLMake make;
        int speed;                          // cruise speed in mph
        double chargeTime;                  // time from empty to full charge in hours
        double drainTime;                   // time from full to empty in hours
        int passengerCapacity;              // number of passengers VTOL can carry
        double faultProbability;            // probability of a fault occuring per hour
        double timeToStateChange;           // time from full charge to empty while flying or from empty to full while charging
        double timeFlying = 0;
        double timeWaiting = 0;
        double timeCharging = 0;
        int numFaults = 0;
        double timeInStateThisTick = 0;     // time spent in the current state this tick of the simulation
};

/**
 * wrapper class for deque to implement process queues enabling capacity limit and queue specific random number generation
 */
class VTOLQueue
{
    public:
        VTOLQueue( VTOLStatus type, int randSeed, int capacity = UNLIMITED ) : generator( nullptr ), queueType(type), capacity(capacity)
        {
            if( queueType == FLYING )
            {
                initGenerator();
            }
        }

        ~VTOLQueue() 
        {
            if( queueType == FLYING )
            {
                delete generator;
            }
        }

        bool push( VTOL * );
        VTOL * pop();
        VTOL * getNextVTOL();
        int size();
        bool empty();
        bool full();
        double getFaultRoll();
    private:
        void initGenerator();
        std::default_random_engine * generator;
        deque<VTOL *> q;
        VTOLStatus queueType;
        deque<VTOL *>::iterator it;
        int capacity;
        vector<double> overchargeDurations;
};

#endif