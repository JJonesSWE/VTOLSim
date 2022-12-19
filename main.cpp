#include "Simulation.h"
#include <string>
#include <iostream>

int main( int argc, char ** argv )
{
    srand( 0 );

    SimulationEngine sim;
    sim.init();
    sim.run();

    return 0;
}