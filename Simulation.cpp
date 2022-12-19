#include "Simulation.h"

// set up barriers for thread syncronization
barrier g_syncPoint(4);  // one for each queue thread and one for watcher thread
barrier g_tickTiming(2); // one for watcher and one for timer threads
SimulationEngine::SimulationEngine()
    : flyingQueue( FLYING, rand() ), waitingQueue( WAITING, rand() ), chargingQueue( CHARGING, rand(), NUM_CHARGERS ), tickLength( 1.0 / TICK_PER_SEC ), hoursPerTick( tickLength / 60 )
{

}

void SimulationEngine::init()
{
    std::uniform_int_distribution distribution( 0, 4 );
    std::default_random_engine generator( clock() );
    for( int i = 0; i < NUM_AIRCRAFT; ++i )
    {
        addNewVTOL( static_cast<VTOLMake>( distribution( generator ) ) );
    }
}

void SimulationEngine::addNewVTOL( VTOLMake make )
{
    VTOLs.push_back( new VTOL( make ) );
    flyingQueue.push( VTOLs[VTOLs.size() - 1] );
}

void SimulationEngine::run()
{
    // spawn threads
    vector<thread> threads;
    threads.push_back( thread( &SimulationEngine::syncThreads, this ) );
    threads.push_back( thread( &SimulationEngine::processQueue, this, FLYING ) );
    threads.push_back( thread( &SimulationEngine::processQueue, this, CHARGING ) );
    threads.push_back( thread( &SimulationEngine::processQueue, this, WAITING ) );
    
    // loop through ticks of the simulation
    for( int i = 0; i < TICK_PER_SEC * SIM_DUR_SEC; ++ i )
    {
        // process queues
        g_syncPoint.arrive_and_wait();
        // move charging and flying vtols
        g_syncPoint.arrive_and_wait();
        // move waiting vtols
        g_syncPoint.arrive_and_wait();
        g_tickTiming.arrive_and_wait();
    }    

    for( int i = 0; i < threads.size(); ++i )
    {
        threads[i].join();
    }
    prepareSummary();
}

int SimulationEngine::processQueue( VTOLStatus queueType )
{
    
    double timeElapsed = 0.0;
    for( int tickNum = 0; tickNum < TICK_PER_SEC * SIM_DUR_SEC; ++tickNum )
    {
        timeElapsed += tickLength;
        vector<VTOL *> * stateChangedVTOLs = nullptr;
        // advance time for all VTOLs in the relevant queue
        if( queueType != WAITING )
        {
            stateChangedVTOLs = new vector<VTOL *>();
        }
        updateVTOLs( queueType, stateChangedVTOLs );
        
        g_syncPoint.arrive_and_wait();
        // move vtols that are no longer flying or charging to appropriate queue
        if( queueType != WAITING )
        {
            moveVTOLs( queueType, stateChangedVTOLs );
        }
        g_syncPoint.arrive_and_wait();

        // move waiting vtols to charger if any are available
        if( queueType == WAITING )
        {
            moveVTOLs( queueType, stateChangedVTOLs );
        }
        g_syncPoint.arrive_and_wait();
        delete stateChangedVTOLs;
    }

    return 0;
}

void SimulationEngine::updateVTOLs( VTOLStatus queueType, vector<VTOL *> * stateChangedVTOLs )
{
    VTOLQueue * threadQueue = getQueuePointerFromType( queueType );
    int VTOLsInQueue = threadQueue->size();
    for( int i = 0; i < VTOLsInQueue; ++i )
    {
        // get next vtol to process, using the order preserving getNextVTOL if queue is waiting queue
        VTOL * curVTOL = threadQueue->pop();

        double timeInEndState = curVTOL->updateVTOL( hoursPerTick, threadQueue->getFaultRoll() );
        if( stateChangedVTOLs && queueType != curVTOL->getStatus() )
        {
            stateChangedVTOLs->push_back( curVTOL );
            if( queueType == CHARGING )
            {
                chargerAvailabilityTimes.push_back( timeInEndState );
            }
        }
        else
        {
            threadQueue->push( curVTOL );
        }
    }

    // if this is the charging queue sort the charger availabilities this tick in descending order
    if( queueType == CHARGING )
    {
        std::sort( chargerAvailabilityTimes.begin(), chargerAvailabilityTimes.end(), std::greater<double>() );
    }
    else if( stateChangedVTOLs && queueType == FLYING ) // if this is the flying queue sort the vtols that changed from flying to waiting by time spent waiting (will move from waiting to charging if chargers are available in a later step)
    {
        std::sort( stateChangedVTOLs->begin(), stateChangedVTOLs->end() );
    }

}

void SimulationEngine::moveVTOLs( VTOLStatus queueType, vector<VTOL *> * stateChangedVTOLs )
{
    VTOLQueue * nextQueue = getQueuePointerFromType( static_cast<VTOLStatus>(  ( queueType + 1 ) % 3 ) );
    if( queueType != WAITING )
    {
        for( int i = 0; i < stateChangedVTOLs->size(); ++i )
        {
            nextQueue->push( stateChangedVTOLs->at( i ) );
        }
    }
    else
    {
        // check for charger availability and adjust the time spent waiting if necessary
        int chargerAvailIdx = 0;
        while( !chargingQueue.full() && !waitingQueue.empty() )
        {
            VTOL * curVTOL = waitingQueue.pop();
            if( chargerAvailIdx >= chargerAvailabilityTimes.size() )
            {
                curVTOL->moveToCharger( hoursPerTick );
            }
            else
            {
                curVTOL->moveToCharger( chargerAvailabilityTimes.at( chargerAvailIdx ) );
            }
            chargingQueue.push( curVTOL );
        }

        chargerAvailabilityTimes.clear();
    }
}

VTOLQueue * SimulationEngine::getQueuePointerFromType( VTOLStatus queueType )
{
    VTOLQueue * retQueue = nullptr;
    switch( queueType )
    {
        case FLYING:
            retQueue = &flyingQueue;
            break;
        case WAITING:
            retQueue = &waitingQueue;
            break;
        case CHARGING:
            retQueue = &chargingQueue;
            break;
    }

    return retQueue;
}

int SimulationEngine::syncThreads()
{
    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
    
    double msPerTick = tickLength * 1000;
    for( int i = 0; i < TICK_PER_SEC * SIM_DUR_SEC; ++ i )
    {
        std::this_thread::sleep_until( start + std::chrono::milliseconds( (int) msPerTick * ( i+1 ) ) );
        g_tickTiming.arrive_and_wait();
    }
    return 0;
}

void SimulationEngine::prepareSummary()
{
    cout << "Make       | Avg. Flight |  Avg. Wait  | Avg. Charge |  Max Faults | Total Passenger Miles |" << endl;
    cout << "--------------------------------------------------------------------------------------------" << endl;
    for( int i = 0; i < 5; ++i )
    {
        displayMakeSummary( static_cast<VTOLMake>( i ) );
    }
}

void SimulationEngine::displayMakeSummary( VTOLMake make )
{
    int count = 0;
    double flightTime = 0.0;
    double chargeTime = 0.0;
    double waitTime = 0.0;
    int maxFaults = 0;
    double totalPassengerMiles = 0.0;
    
    for( const VTOL * curVTOL : VTOLs )
    {
        if( curVTOL->getMake() == make )
        {
            ++count;
            flightTime += curVTOL->getTimeInFlight();
            chargeTime += curVTOL->getTimeCharging();
            waitTime += curVTOL->getTimeWaiting();
            int curFaults = curVTOL->getNumFaults();
            if( curFaults > maxFaults )
                maxFaults = curFaults;
            totalPassengerMiles += curVTOL->getPassengerMiles();
        }
    }

    if( count > 1 )
    {
        flightTime /= count;
        chargeTime /= count;
        waitTime /= count;
    }

    switch( make )
    {
        case ALPHA:
            cout << "Alpha      ";
            break;
        case BETA:
            cout << "Beta       ";
            break;
        case CHARLIE:
            cout << "Charlie    ";
            break;
        case DELTA:
            cout << "Delta      ";
            break;
        case ECHO:
            cout << "Echo       ";
            break;
    }
    cout << "|" << std::fixed << std::setw(12) << std::setprecision(2) << flightTime << " |" 
                << std::setw(12) << std::setprecision(2) << waitTime << " |" 
                << std::setw(12) << std::setprecision(2) << chargeTime << " |" 
                << std::setw(12) << maxFaults << " |" 
                << std::setw(22) << std::setprecision(2) << totalPassengerMiles << " |" << endl;
}
