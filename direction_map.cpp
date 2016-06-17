#include "direction_map.h"
#include <iostream>
#include <utility>

DirectionMap::DirectionMap( const MapLoader& ml , const vector < vector<int>* >& paths ) {
  int map_size = ml.rows * ml.cols;
  int num_of_agents = paths.size();
  dm.resize( map_size , make_pair(0,0) );
  vector<int> counters ( map_size , 0 );
  for (int agent=0; agent<num_of_agents; agent++) {
    for (int timestep=0; timestep<(int)(paths[agent])->size()-1; timestep++) {
      MapLoader::valid_actions_t action = ml.get_action( paths[agent]->at(timestep) ,  paths[agent]->at(timestep+1) );
      switch(action) {
      case MapLoader::valid_actions_t::WAIT:
	//cout << "WAIT," << endl;
	break;
      case MapLoader::valid_actions_t::NORTH:
	// update from-cell
	dm.at( paths[agent]->at(timestep) ).second++;
	counters.at ( paths[agent]->at(timestep) )++;
	// update to-cell
	dm.at( paths[agent]->at(timestep+1) ).second++;
	counters.at ( paths[agent]->at(timestep+1) )++;
	break;
      case MapLoader::valid_actions_t::EAST:
	// update from-cell
	dm.at( paths[agent]->at(timestep) ).first++;
	counters.at ( paths[agent]->at(timestep) )++;
	// update to-cell
	dm.at( paths[agent]->at(timestep+1) ).first++;
	counters.at ( paths[agent]->at(timestep+1) )++;
	break;
      case MapLoader::valid_actions_t::SOUTH:
	// update from-cell
	dm.at( paths[agent]->at(timestep) ).second--;
	counters.at ( paths[agent]->at(timestep) )++;
	// update to-cell
	dm.at( paths[agent]->at(timestep+1) ).second--;
	counters.at ( paths[agent]->at(timestep+1) )++;
	break;
      case MapLoader::valid_actions_t::WEST:
	// update from-cell
	dm.at( paths[agent]->at(timestep) ).first--;
	counters.at ( paths[agent]->at(timestep) )++;
	// update to-cell
	dm.at( paths[agent]->at(timestep+1) ).first--;
	counters.at ( paths[agent]->at(timestep+1) )++;
	break;
      default:
	break;
      }
    }
  }
  // normalize
  for (int id=0; id<map_size; id++) {
    if ( counters.at(id) != 0) {
      dm.at(id).first  /= counters.at(id);
      dm.at(id).second /= counters.at(id);
    }
  }
}

DirectionMap::~DirectionMap(){
}
