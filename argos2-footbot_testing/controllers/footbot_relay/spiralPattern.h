#ifndef _SPIRALPATTERN_H_
#define _SPIRALPATTERN_H_

#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <termios.h>
#include <math.h>
#include <map>
#include <set>
#include <vector>
#include <queue>
#include <sstream>
#include <cstdlib>
#include <ctime>
#include <iterator>   
#include <random>
#include <deque>
#include <algorithm>


using namespace std;


class SpiralPattern
{

 public:

    struct dataWrite
    {
      ofstream data_file;
      string filename;
    };

    struct Positiontype
    {
        double x;
        double y;
    };

  typedef struct Positiontype Position;

    struct SpiralData
    {   
      //spiral line segments motion variables
      enum Direction {
        ALONG_POSX = 0,
        ALONG_NEGY = 1,
        ALONG_NEGX = 2,
        ALONG_POSY = 3

      } direction;

      

      map<uint8_t, int> rectSpiralSequence;
      map<int, uint8_t> rectSpiralSequenceRev;
    };
    
    struct SpiralMotion
    { 
      uint8_t spiralCondition; // corner or edge or centre
      uint8_t sequence;
      float spiral_count;
      bool clockwise;
      vector<bool> possibleDirections;
      vector<bool> limitedDirection;
      Position spiralPos;
      bool initialise;
      uint8_t counter;
      bool endCondition;
      vector<uint8_t> quadrantTrack; 
    };

private:
    
    SpiralData spiralData;
    SpiralMotion spiralMotion;

    Position minimum;
    Position maximum;
  
    double threshold;
    int randNum = 0;
    map<int,vector<double> > quadrant;

public:
    
   std::vector<uint8_t> q = {3,4,2,1};
    /* Class constructor. */
    SpiralPattern();

    /* Class destructor. */
    virtual ~SpiralPattern() {
    }

   /* inline double distanceCalculator(Position start, Position end)
  {
    return math.sqrt(math.pow((end.x-start.x),2) + math.pow((end.y-start.y),2));
  } */
    std::vector<double> current;
    dataWrite relayPositions;
    
    // To find whether coordinate is in centre or edge/corner
    SpiralMotion* calculatePositions(double x, double y);

    // Calculates position using direction, value to increment and current position
    vector<double>  getNormalSpiralPositions(uint8_t, float, Position*);
    
    // Check condition if the spiral has reached its end
    bool checkCondition(uint8_t, Position&);
    
    //finds distance between two positions
    double findDistance(Position*,Position*);
    
    //gives the midpoint of a quadrant based on the random quadrant chosen
    //map<int,vector<double> >&  getQuadrant();

    void getQuadrant();
    
    // get next Position for the spiral
    SpiralMotion* getPosition(SpiralMotion* spiralTemp);
    
    // Once a spiral has ended choose a random quadrant
    SpiralMotion* assignQuadrant(SpiralMotion* spiralTemp);
    
    // finds which quadrant a coordinate belongs to
    inline int quadrantFunc(double x, double y)
  {
    int i = (int)((atan2(y,x)/atan2(1.,0.))+2.0); 
    return q.at(i);
  }

   
};

#endif
