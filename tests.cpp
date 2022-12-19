#include "Models.h"
#include <iostream>
#include "Utils.h"

using std::cout;
using std::endl;

#define CAUSE_FAULT 0.0
#define NO_FAULT 10000.0

int main()
{
    cout << "Testing VTOL behavior" << endl;
    VTOL testCraft( ALPHA );
    assert( testCraft.getNumFaults() == 0 );
    assert( almostEqual( testCraft.getPassengerMiles(), 0 ) );
    assert( almostEqual( testCraft.getTimeInFlight(), 0 ) );
    assert( almostEqual( testCraft.getTimeWaiting(), 0 ) );
    assert( almostEqual( testCraft.getTimeCharging(), 0 ) );
    cout << "  Passed: creation" << endl;
    
    cout << "  Advancing time 2 hours" << endl;
    testCraft.updateVTOL( 1.0, CAUSE_FAULT ); // should generate a fault
    testCraft.updateVTOL( 1.0, NO_FAULT );
    // should be 1 fault, 1.66 hours flying and .33 hours waiting
    assert( testCraft.getNumFaults() == 1 );
    assert( almostEqual( testCraft.getTimeInFlight(), 5.0 / 3.0 ) );
    assert( almostEqual( testCraft.getTimeWaiting(), 1.0 / 3.0 ) );
    assert( almostEqual( testCraft.getPassengerMiles(), 20.0 / 3.0 * 120 ) );
    cout << "  Passed: Time advance" << endl;

    cout << "  Advancing time .5 hours and moving to charger" << endl;
    testCraft.updateVTOL( 0.5, CAUSE_FAULT ); // even though cause fault value us used no fault should be generated since VTOL is not flying
    testCraft.moveToCharger( 1.0 ); // even though the charger indicates it was available for 1 hour the test craft should only have 0.6 hours charge since it only spent 30 min waiting this tick
    assert( testCraft.getNumFaults() == 1 );
    assert( almostEqual( testCraft.getTimeInFlight(), 5.0 / 3.0 ) );
    assert( almostEqual( testCraft.getTimeWaiting(), 1.0 / 3.0 ) );
    assert( almostEqual( testCraft.getTimeCharging(), 0.5 ) );
    assert( almostEqual( testCraft.getPassengerMiles(), 20.0 / 3.0 * 120 ) );
    assert( testCraft.getStatus() == CHARGING );
    cout << "  Passed: .5 hour advance and charge" << endl;

    cout << "  Advancing time .15 hours" << endl;
    testCraft.updateVTOL( 0.15, CAUSE_FAULT );  // since the vehicle spent time flying this tick this should trigger a fault
    assert( testCraft.getNumFaults() == 2 );
    assert( almostEqual( testCraft.getTimeWaiting(), 1.0 / 3.0 ) );
    assert( almostEqual( testCraft.getTimeCharging(), 0.6 ) );
    assert( testCraft.getStatus() == FLYING ); // this time advance should result in the VTOL flying again
    cout << "  Passed: .15 hour advance and charge" << endl;

    VTOL testCraft2( BETA );
    cout << "Testing appropriate behavior of different VTOL type" << endl;
    testCraft2.updateVTOL( 1.0, CAUSE_FAULT ); // causes fault
    testCraft2.updateVTOL( 1.0, NO_FAULT );
    testCraft2.updateVTOL( 0.5, CAUSE_FAULT ); // no cause fault since waiting/charging
    testCraft2.moveToCharger( 1.0 );
    testCraft2.updateVTOL( 0.15, CAUSE_FAULT ); // causes fault
    assert( !almostEqual( testCraft2.getPassengerMiles(), testCraft.getPassengerMiles() ) );
    assert( !almostEqual( testCraft2.getTimeCharging(), testCraft.getTimeCharging() ) );
    assert( !almostEqual( testCraft2.getTimeWaiting(), testCraft.getTimeWaiting() ) );
    assert( !almostEqual( testCraft2.getTimeInFlight(), testCraft.getTimeInFlight() ) );
    cout << "  Passed: different behavior for creation of other VTOL make" << endl;
    assert( almostEqual( testCraft2.getTimeInFlight(), 2.0 / 3 + 0.3 + 0.15 ) );
    assert( almostEqual( testCraft2.getTimeWaiting(), 4.0 / 3 ) );
    assert( almostEqual( testCraft2.getTimeCharging(), 0.2 ) );
    assert( almostEqual( testCraft2.getPassengerMiles(), ( 2.0 / 3 + 0.3 + 0.15 ) * 500 ) );
    assert( testCraft2.getNumFaults() == 2 );
    cout << "  Passed: expected resulting values for other make" << endl;

    // additional tests ensuring the behaviors of other makes could potentially be beneficial

    // creating unit tests for the simulation could be done by adding get functions for the resulting averages and loading the simulation with specific combinations
    // of aircraft and comparing the resulting averages with the expected values from that combination of aircrafts
    // e.g. loading with 6 alpha should result in 2.1 avg flight time, 0.3 avg wait time, .6 average charge time, 6048 passenger miles, and a variable amount of max faults
    return 0;
}