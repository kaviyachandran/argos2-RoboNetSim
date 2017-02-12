#include "spiralPattern.h"

/**** Three possible postions for spiral to start
a.centre, b.edge, c.Corner ****/

SpiralPattern::SpiralPattern(){

  spiralMotion.possibleDirections.assign(4,true); // pos X , pos Y, neg X , neg Y
  spiralMotion.initialise = true;
  //corner = 0;
 // environment coordinates
  minimum.x = -4.50;
  minimum.y = -4.50;
  maximum.x = 4.50;
  maximum.y = 4.50;

  threshold = 0.75; // agent communication range
  
 
  spiralMotion.spiral_count = threshold;
  spiralMotion.sequence = 0;
  spiralMotion.counter = 0;
  spiralMotion.clockwise = true;
  spiralMotion.endCondition = true;
  spiralMotion.quadrantTrack = q;
  
  spiralData.rectSpiralSequence.emplace(0,1); // posX -> +1
  spiralData.rectSpiralSequence.emplace(1,-2); // posy
  spiralData.rectSpiralSequence.emplace(2,-1); // negX
  spiralData.rectSpiralSequence.emplace(3,2); // negY

  spiralData.rectSpiralSequenceRev.emplace(1,0);
  spiralData.rectSpiralSequenceRev.emplace(-2,1);
  spiralData.rectSpiralSequenceRev.emplace(-1,2);
  spiralData.rectSpiralSequenceRev.emplace(2,3);

  relayPositions.filename = "positionSpiral.csv";
  relayPositions.data_file.open(relayPositions.filename, ios::out | ios::ate);
  
  getQuadrant();
}

void 
SpiralPattern::getQuadrant()
{ 
  vector<double> coordinate, x_limits, y_limits;
  map<int ,  vector<double> > tempQuadrant;

  x_limits.push_back(maximum.x);
  x_limits.push_back(minimum.x);

  y_limits.push_back(maximum.y);
  y_limits.push_back(minimum.y);
  
  

  for(int i = 0; i < x_limits.size(); i++)
  {
    for(int j = 0; j < y_limits.size(); j++)
    { 
      coordinate.clear();
     
      coordinate.push_back(x_limits[i]/2);
      coordinate.push_back(y_limits[j]/2);

      quadrant.emplace(quadrantFunc(coordinate[0],coordinate[1]),coordinate);
    
    }
    
  }
  
  //return quadrant;
}


struct SpiralPattern::SpiralMotion* 
SpiralPattern::calculatePositions(double x, double y)
{   

    spiralMotion.spiralPos.x = x;
    spiralMotion.spiralPos.y = y;

    //initialise
    spiralMotion.possibleDirections.assign(4,true); // pos X , pos Y, neg X , neg Y
    spiralMotion.initialise = true;
    spiralMotion.spiral_count = 1;
    spiralMotion.sequence = 0;
    spiralMotion.clockwise = true;
    spiralMotion.limitedDirection.assign(4,true);


    cout << "maximum.x" << maximum.x << endl;
    cout << "maximum.y" << maximum.y << endl;

    if((spiralMotion.spiralPos.x+spiralMotion.spiral_count) > maximum.x && spiralMotion.possibleDirections[0])
    {
      spiralMotion.possibleDirections[0] = false;
    }

    if(spiralMotion.spiralPos.x-spiralMotion.spiral_count < minimum.x && spiralMotion.possibleDirections[2])
    {
      spiralMotion.possibleDirections[2] = false;
    } 
    if((spiralMotion.spiralPos.y+spiralMotion.spiral_count) > maximum.y && spiralMotion.possibleDirections[3])
    {
      spiralMotion.possibleDirections[3] = false;
    }

    if(spiralMotion.spiralPos.y-spiralMotion.spiral_count < minimum.y && spiralMotion.possibleDirections[1])
    {
      spiralMotion.possibleDirections[1] = false;
    } 

    spiralMotion.spiralCondition = uint8_t(std::count(spiralMotion.possibleDirections.begin(), spiralMotion.possibleDirections.end(), 0));
    
    SpiralMotion* calculatedSpiralM = &spiralMotion;
    cout << "initialised in calculatePositions: " << "sequence:" << calculatedSpiralM->sequence << "Increment: " << calculatedSpiralM->spiral_count << endl;
    return calculatedSpiralM; 
}

vector<double> 
SpiralPattern::getNormalSpiralPositions(uint8_t switchVal, float increment, Position* pos)
{   
    printf("Increment %f\n", increment);
    printf("%f %f\n", pos->x, pos->y);
    int indicator =0;
    switch(switchVal) 
    {
      case SpiralData::ALONG_POSX: 
      {
        if((pos->x+increment) > (maximum.x-threshold))
        {
          pos->x = maximum.x - threshold;  
          spiralMotion.limitedDirection[switchVal] = false;
          indicator ++;
        }
        else
        {
          pos->x = double(pos->x+increment); // 1.0 -> range of mission agents
        }
        printf("In pox x %f %d\n",pos->x, indicator);
        break;
      }
      case SpiralData::ALONG_POSY: {
        if((pos->y+increment) > (maximum.y-threshold))
        {
          pos->y = maximum.y - threshold;  
          spiralMotion.limitedDirection[switchVal] = false;
          indicator ++;
        }
        else
        {
          pos->y = double(pos->y+increment); // 1.0 -> range of mission agents
        }
        printf("In pox y %f %d\n",pos->y, indicator);
        break;
      }
      case SpiralData::ALONG_NEGX: {
        if((pos->x-increment) < (minimum.x+threshold))
        {
          pos->x = minimum.x + threshold; 
          spiralMotion.limitedDirection[switchVal] = false;
          indicator ++; 
        }
        else
        {
          pos->x = double(pos->x-increment); // 1.0 -> range of mission agents
        }
        printf("In neg x %f %d\n",pos->x, indicator);
        break;
      }
      case SpiralData::ALONG_NEGY: {
        if((pos->y-increment) < (minimum.y+threshold))
        {
          pos->y = minimum.y + threshold;  
          spiralMotion.limitedDirection[switchVal] = false;
          indicator ++;
        }
        else
        {
          pos->y =  double(pos->y-increment); // 1.0 -> range of mission agents
        }
        printf("In neg y %f %d\n", pos->y, indicator);
        break;      
      }
      default: {
         cout << "We can't be here, there's a bug!" << std::endl;
      }
    }
    vector<double> position = {pos->x,pos->y};

    printf("%f %f \n",pos->x,pos->y);
    relayPositions.data_file << pos->x << "," << pos->y << "\n";
    return position;
}

double 
SpiralPattern::findDistance(Position* startVal, Position* endVal)
{ 
  printf("Start: %f %f  End: %f %f\n", abs(startVal->x),abs(startVal->y), abs(endVal->x), abs(endVal->y));
  return sqrt(pow(abs(endVal->x)-abs(startVal->x),2)+pow(abs(endVal->y)-abs(startVal->y),2));
}

bool
SpiralPattern::checkCondition(uint8_t temp, Position& currentPos){
  
  bool flag = false;
  Position limit;
  if(temp == 0)
  {
    limit = maximum;
  }
  else
  {
    limit.x = maximum.x/2;
    limit.y = maximum.y/2;
  }
  printf("distance to threshold %f\n", findDistance(&currentPos,&limit));
  if(findDistance(&currentPos,&limit) > threshold)
    {
        flag = true;
    } 
  return flag;
}

struct SpiralPattern::SpiralMotion* 
SpiralPattern::getPosition(SpiralMotion* spiralTemp)
{     
   Position tempP =  spiralTemp->spiralPos;
   vector<double> current_pos;

   /* if(spiralTemp->counter % 4 == 0)
    {
      spiralTemp->counter = 0;
    } */
    
    if(spiralTemp->sequence %4 == 0)
    {
      spiralTemp->sequence = 0;
      spiralTemp->limitedDirection.assign(4,true);
    }

    if(spiralTemp->spiralCondition == 0)
    { 
      if(spiralTemp->initialise)
      {
        cout << "I am here: spiralCount" << spiralTemp->spiral_count << "spiralMotion.sequence " << spiralTemp->sequence << endl;
        //spiralTemp->clockwise = true;
        //counter = 0;
        spiralTemp->spiral_count = threshold;
        spiralTemp->sequence = 0;
        spiralTemp->initialise = false;
        spiralTemp->counter = 0;
      }


    current_pos = getNormalSpiralPositions(spiralTemp->sequence, spiralTemp->spiral_count,&tempP);
    
    //spiralTemp->counter = spiralTemp->counter + 1;
    
    if(spiralTemp->sequence%2 == 0)
    {
      spiralTemp->spiral_count = spiralTemp->spiral_count + threshold;
    }

   }
    else if(spiralTemp->spiralCondition == 1 || spiralTemp->spiralCondition == 2)
    { 
      /*if(spiralTemp->initialise)
      {
        //ptrdiff_t zeroPos = std::find(spiralTemp->possibleDirections.begin(), spiralTemp->possibleDirections.end(), 0)
        //-spiralTemp->possibleDirections.begin(); 
        
        //printf("zero Pos: %d\n", zeroPos);
   

        //spiralTemp->sequence =  spiralData.rectSpiralSequenceRev[-1*spiralData.rectSpiralSequence[zeroPos]];
        
        //spiralTemp->clockwise = false;
        spiralTemp->initialise = false;
       
        //spiralTemp->spiral_count = abs(current_pos[std::abs(spiralData.rectSpiralSequence[spiralTemp->sequence])-1])-threshold;
        printf("spiral Count: %f\n",spiralTemp->spiral_count);
        spiralTemp->counter = 0;
      }*/
     current_pos = quadrant[quadrantFunc(spiralTemp->spiralPos.x,spiralTemp->spiralPos.y)];
     spiralTemp->spiralCondition = 0;
    }
    /*current_pos.clear();
    current_pos = getNormalSpiralPositions(spiralTemp->sequence, spiralTemp->spiral_count,&tempP);
    spiralTemp->spiralPos.x = current_pos[0];
    spiralTemp->spiralPos.y = current_pos[1];
    spiralTemp->counter = spiralTemp->counter + 1;
      
    spiralTemp->sequence = spiralTemp->sequence + 1.0;
    
    if(spiralTemp->counter%2 == 0)
    {
      if(spiralTemp->clockwise)
      {
        spiralTemp->spiral_count = spiralTemp->spiral_count + threshold;
       
      }
      else if(not spiralTemp->clockwise)
      {
        spiralTemp->spiral_count = spiralTemp->spiral_count - threshold;
        
      }
    }*/

    spiralTemp->spiralPos.x = current_pos[0];
    spiralTemp->spiralPos.y = current_pos[1];
    spiralTemp->sequence = spiralTemp->sequence + 1.0;

    int limitedDirection_Num =int(std::count(spiralTemp->limitedDirection.begin(), spiralTemp->limitedDirection.end(), 0));  

    printf("limited Direction: %d\n" ,limitedDirection_Num);
    if(limitedDirection_Num >= 2 || not checkCondition(spiralTemp->spiralCondition, spiralTemp->spiralPos)) // if an agetn cannot move in two directions
    {
      spiralTemp->endCondition =  false;
    }
   
   
  
  relayPositions.data_file << spiralTemp->spiralPos.x << "," << spiralTemp->spiralPos.y << "\n";
  return spiralTemp;
}


struct SpiralPattern::SpiralMotion* 
SpiralPattern::assignQuadrant(SpiralMotion* spiralTemp)
{ 
  if(spiralTemp->clockwise)
  {
    spiralTemp->quadrantTrack.erase(std::remove(spiralTemp->quadrantTrack.begin(),
      spiralTemp->quadrantTrack.end(), randNum), spiralTemp->quadrantTrack.end());
  }
  else
  {
    uint8_t currentQuad = (uint8_t)quadrantFunc(spiralTemp->spiralPos.x,spiralTemp->spiralPos.y);

    spiralTemp->quadrantTrack.erase(std::remove(spiralTemp->quadrantTrack.begin(),
      spiralTemp->quadrantTrack.end(), currentQuad), spiralTemp->quadrantTrack.end());

    
  }

  if(spiralTemp->quadrantTrack.size() == 0)
  {
    spiralTemp->quadrantTrack = q;
  }

  srand(time(NULL));
  

  

  for(int i = 0 ;i < spiralTemp->quadrantTrack.size(); i++)
  {
    printf("quadrant %d\n", spiralTemp->quadrantTrack[i] );
  }

  if(spiralTemp->quadrantTrack.size()>1)
  {
    randNum = (rand() % spiralTemp->quadrantTrack.size()) + 1;
    randNum = spiralTemp->quadrantTrack[randNum];
    printf("Random Number %d\n", randNum);
  }
  else
  { 
    randNum = spiralTemp->quadrantTrack[0];
    //spiralTemp->quadrantTrack.clear();
  }
  
  for(int i = 1; i <= quadrant.size();i++)
  {
     vector<double> temp = quadrant[i];
     printf("x: %f y: %f\n", temp[0], temp[1]);
  }

  vector<double> randPos = quadrant[randNum]; 
  spiralTemp->spiralPos.x = randPos[0];
  spiralTemp->spiralPos.y = randPos[1]; 

  relayPositions.data_file << spiralTemp->spiralPos.x << "," << spiralTemp->spiralPos.y << "\n";
  return spiralTemp;
}

