#ifndef DIRECTIONMAP_H
#define DIRECTIONMAP_H
#include "map_loader.h"
#include <vector>

using namespace std;

class DirectionMap {
 public:

  vector< pair<double,double> > dm; // direction map -- contains dv_x and dv_y for each cell in the map

  // Ctor gets a map and paths, and generates dm
  DirectionMap( const MapLoader& ml , const vector < vector<int>* >& paths );
  ~DirectionMap();
};

#endif
