#ifndef LIB_CONSTANTS_H
#define LIB_CONSTANTS_H

namespace constants
{
 enum MessageSource
{
	RELAY,
	BASE_STATION,
	MISSION_AGENTS
};

// Map with id and position of neighbours.
//typedef std::map<uint8_t,std::vector<double> > NeighbourMap;
}
#endif