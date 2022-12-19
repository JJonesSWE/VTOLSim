#ifndef SIMULATION_H
#define SIMULATION_H

#include <iostream>
#include <string>
#include <thread>
#include <barrier>
#include "Models.h"
#include "Utils.h"
#include <chrono>
#include <iomanip>

#ifndef NUM_CHARGERS
#define NUM_CHARGERS 3
#endif

#ifndef NUM_AIRCRAFT
#define NUM_AIRCRAFT 20
#endif

#ifndef SIM_DUR_SEC
#define SIM_DUR_SEC 180
#endif

#ifndef TICK_PER_SEC
#define TICK_PER_SEC 30
#endif

using std::thread;
using std::barrier;
using std::string;
using std::cout;
using std::endl;

class SimulationEngine
{
    public:
        SimulationEngine();
        ~SimulationEngine() 
        {
            for( VTOL * curVTOL : VTOLs )
            {
                delete curVTOL;
            }
        }
        /**
         * @brief initialize simulation to default configuration
         */
        void init();

        /**
         * @brief run the simulation
         */
        void run();

        /**
         * @brief syncronize each tick of the simulation to a given amount of time
         */
        int syncThreads();

        /**
         * @brief add a VTOL to the simulation
         * @param make make of the VTOL to create and add to the simulation
         */
        void addNewVTOL( VTOLMake make );
    private:
        /**
         * @brief function to process all the VTOLs in a queue of a given type 
         * @param queueType which queue this function should process
         */
        int processQueue( VTOLStatus queueType );
        
        /**
         * @brief prepare and display the summary of the simulation
         */
        void prepareSummary();

        /**
         * @brief process the details for a specific make of VTOL and display the summary of that make
         * @param make which make's VTOLs to prepare and display summary info
         */
        void displayMakeSummary( VTOLMake make );

        /**
         * @brief update the state of the vtols by 1 tick of the simulation
         * @param queueType which queue type this operation should be performed on
         * @param stateChangedVTOLs a collection into which to store any VTOLs that change states
         */
        void updateVTOLs( VTOLStatus queueType, vector<VTOL *> * stateChangedVTOLs = nullptr );

        /**
         * move any vtols that changed queues to their new queue
         * @param queueType which queue type this operation should be performed on
         * @param stateChangedVTOLs a collection of VTOLs that changed states
         */
        void moveVTOLs( VTOLStatus queueType, vector<VTOL *> * stateChangedVTOLs = nullptr );

        /**
         * @brief helper function to determine appropriate queue from which state the thread is processing
         * @param queueType the queue type to retrieve the relevant variable for
         */
        VTOLQueue * getQueuePointerFromType( VTOLStatus queueType );
        
        VTOLQueue flyingQueue;                      // queue of flying VTOLs to be processed
        VTOLQueue waitingQueue;                     // queue of VTOLs waiting for a charger to be processed
        VTOLQueue chargingQueue;                    // queue of charging VTOLs to be processed
        vector<VTOL *> VTOLs;
        vector<double> chargerAvailabilityTimes;    // vector to track how much time within the current tick chargers were available
        double tickLength;
        const double hoursPerTick;
};

#endif