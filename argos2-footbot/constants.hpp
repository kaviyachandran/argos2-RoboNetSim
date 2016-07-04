#ifndef LIB_CONSTANTS_H
#define LIB_CONSTANTS_H

#include <iostream>
#include <cstdint>
#include <vector>
#include <map>

//using namespace std;
namespace constants
{
 
 enum MessageSource
{
	MISSION_AGENTS,
	RELAY,
	BASE_STATION
} MessageSource;

// Map with id and position of neighbours.
typedef std::map<uint8_t,std::vector<double> > NeighbourMap;

}
#endif