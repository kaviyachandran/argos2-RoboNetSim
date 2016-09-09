#include "footbot_mission_agents.h"



double distance(double x1, double y1, double x2, double y2)
{
  return std::sqrt(pow((y2-y1),2)+ pow((x2-x1),2));
}

vector<double> approximate_pos(vector<double> curr_pos,std::vector<uint16_t> &t, uint16_t number_timesteps, double optimal_speed, vector<double> &target_positions, uint8_t interval)
{ 
  vector<double> posi;
  double result, param;
  int rest = 0;
  
  cout << "t " << t[0] << " "<< t[1] << endl;

for(int i = 0 ; i < t.size(); i++)
  { 
    int t2;
    /// Calculating positions only for required number of timesteps

    if(t[i] < number_timesteps)
    {
      t2 = t[i];
    }
    else
    {
      t2 = (number_timesteps-t[i-1])+1;
    }
    
    cout << "time step *************" << t2 << endl;

    while(t2 > interval)
    {   
        double x;
        double y;
        uint8_t time_interval;

        param = (target_positions[((i*2)+1)]-curr_pos[1])/(target_positions[(i*2)]-curr_pos[0]); 
        result = atan (param) * 180 / PI;
       
        
        if(rest != 0)
       {  
          time_interval = rest;
          rest = 0;
       }
       else
       {
          time_interval = interval;
       }

        x = curr_pos[0] + (optimal_speed*sin(result)*time_interval);
        y = curr_pos[1] + (optimal_speed*cos(result)*time_interval);
        t2 = t2 - time_interval;

        posi.push_back(x);
        posi.push_back(y);

        cout << "x: " << x << " y:" << y << " ";
        cout << "time " << t2 << endl;

        curr_pos[0] = x;
        curr_pos[1] = y;

       
    }

    cout << "One target reached " << endl;
    if(t2 != 0)
    {
      curr_pos[0] = curr_pos[0] + (optimal_speed*sin(result)*t2);
      curr_pos[1] = curr_pos[1] + (optimal_speed*cos(result)*t2);
      rest = interval - t2;
      cout << "rest " << rest;
    }
  }
return posi;
}


/*
/ Used to predict the positions for every 10 timestep / twice per second.
/
*/
vector<double> 
FootbotMissionAgents::calculated_positions(uint16_t time_s, uint8_t interval)
{

std::vector<double> curr_pos, predicted_pos;
vector<uint16_t> timestep;
curr_pos.push_back(m_navClient->currentPosition().GetX());
curr_pos.push_back(m_navClient->currentPosition().GetY());

uint16_t prev_time = 0, curr_timestep = 0 , i = 0;
double temp = distance(curr_pos[0],curr_pos[1],target_positions[i],target_positions[i+1]);


while(curr_timestep <= time_s)
  {  
    uint16_t time_step = (temp/optimal_speed) - (min_distance_to_target/optimal_speed); /// Total time to reach target - time taken to coven 0.5m (targetMinPointDistance)
    time_step = time_step + prev_time;
    //dist.push_back(temp);
    timestep.push_back(time_step);
     
    prev_time = time_step;
    curr_timestep = curr_timestep + time_step;
    i = i + 2;

    temp = distance(target_positions[i-2],target_positions[i-1],target_positions[i],target_positions[i+1]);
   }

    predicted_pos = approximate_pos(curr_pos,timestep, time_s, optimal_speed,target_positions,interval);
    cout << "pos size " << predicted_pos.size() << endl;

    return predicted_pos;
}