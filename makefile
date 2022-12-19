# variable for output file name
FILENAME = vtol_sim

# source files
OBJS = main.o Models.o Simulation.o Utils.o
SRCS = main.cpp Models.cpp Simulation.cpp Utils.cpp
HEADERS = Models.h Simulation.h Utils.h

# c++ compilation configurations
CXX = g++
CXXFLAGS = -std=c++2a
CXXFLAGS += -Wall
CXXFLAGS += -pedantic-errors

DEBUG = -g
OPTIMIZE = -03

LDFLAGS =

# leak check variables
LEAK = valgrind
FULL = --leak-check=full

# basic compilation
${FILENAME}: ${OBJS} ${HEADERS}
	${CXX} ${LDFLAGS} ${OBJS} -o ${FILENAME}

${OBJS}: ${SRCS}
	${CXX} ${CXXFLAGS} -c ${@:.o=.cpp}

# clean
clean:
	rm *.o ${FILENAME}

# run
run:
	./${FILENAME}
