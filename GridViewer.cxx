#include "GridViewer.h"
#include<iostream>  // for cout
#include <math.h>  // for sin, cos, PI
#include <cstdlib>  // for RAND_MAX
#include <stdlib.h>  // for srand, rand
#include <time.h>  // for time


#define TWEAK_G_VAL false

using namespace std;

GridViewer::GridViewer(int x,int y,int w,int h,const char *l)
  : Fl_Gl_Window(x,y,w,h,l) { // call the default Fl_Gl_Window ctor
  this->ml = NULL;
  this->al = NULL;
  this->egr = NULL;
  this->cbs_search = NULL;
  this->ecbs_search = NULL;
}

int GridViewer::handle (int e) {

  if (this->viewTabFlag == true) { // -------------------------------------------------------------
    switch ( e ) {
    case FL_PUSH:
      int pushed_row = Fl::event_y()/cell_d;
      int pushed_col = Fl::event_x()/cell_d;
      int pushed_loc = ml->linearize_coordinate(pushed_row,pushed_col);
      if (ml->my_map[pushed_loc] == false) { // only if we click on a non-blocked cell
	//	cout << "LEFT CLICK AT - [" << goal_row << "," << goal_col << "]" << endl; fflush(stdout);
      }
      this->redraw();
      return 1; // END FL_PUSH (note -- return 1 means we took care of the handle)
    }
  } // end viewTabFlag ---------------------------------------------------------------------

  if (this->editTabFlag == true) { // ------------------------------------------------------
    switch ( e ) {
    case FL_PUSH:
      int pushed_row = Fl::event_y()/cell_d;
      int pushed_col = Fl::event_x()/cell_d;
      //      cout << "*** PUSH EVENT IN EDIT TAB : [" << pushed_row << "," << pushed_col << "]" << endl;
      if (editObstacle == true) {  // ----------------------------------
        location_a = ml->linearize_coordinate(pushed_row, pushed_col);
        // add or remove the obstacle
        if (ml->my_map[location_a] == false)
          ml->my_map[location_a] = true;
        else
          ml->my_map[location_a] = false;
        // update the highway accordingly
        if (egr != NULL)
          egr->removeVertex (location_a);  // this is always safe
        location_a = -1;
        location_b = -1;
      }  // ------------------------------------------------------------
      if (editHighway == true) {  //------------------------------------
        if ( Fl::event_button() == FL_RIGHT_MOUSE ) {
          egr->removeVertex(ml->linearize_coordinate(pushed_row, pushed_col));
          location_a = -1;
        } else {
          // update location_a or _b if click wasn't on blocked cell
          if (location_a == -1) {
            location_a = ml->linearize_coordinate(pushed_row, pushed_col);
            if ( ml->my_map[location_a] == true )
              location_a = -1;
          } else {
            location_b = ml->linearize_coordinate(pushed_row, pushed_col);
            if ( ml->my_map[location_b] == true || location_b == location_a )
              location_b = -1;
          }
          if ( location_a != -1 && location_b !=-1 ) {
            double* zeroH = new double [ml->rows * ml->cols];  // deleted by SingleAgentSearch
            SingleAgentSearch eg_path(location_a,
                                      location_b,
                                      zeroH,
                                      ml->get_map(),
                                      ml->rows * ml->cols,
                                      ml->actions_offset,
                                      NULL,
                                      1.0,
                                      false );
            if ( eg_path.findPath(NULL) == true ) {
              if ( egr == NULL ) {
                egr = new EgraphReader();
                (egr->nodes).set_empty_key(0);
                (egr->nodes).set_deleted_key(-1);
              }
              egr->addVertices(eg_path.getPath());
            }
            location_a = -1;
            location_b = -1;
          }
        }
      }  // ------------------------------------------------------------
      if (editAgent == true) {  //------------------------------------
        if ( Fl::event_button() == FL_RIGHT_MOUSE ) {  // right click -- remove agent
          if (location_a == -1) {
            location_a = ml->linearize_coordinate(pushed_row, pushed_col);
            if (ml->my_map[location_a] == false) {
              pair<int, int> idxs = al->agentStartOrGoalAt(pushed_row, pushed_col);
              if (idxs.first != -1 || idxs.second != -1)
                al->clearLocationFromAgents(pushed_row, pushed_col);
            }
            location_a = -1;
          }
        } else {  // left click -- add agent
          if (location_a == -1) {
            location_a = ml->linearize_coordinate(pushed_row, pushed_col);
            if (ml->my_map[location_a] == false) {
              pair<int, int> idxs = al->agentStartOrGoalAt(pushed_row, pushed_col);
              if (idxs.first != -1)
                location_a = -1;  // if we clicked on another agent's start location, retry
            } else {  // click was on an obstacle, retry
              location_a = -1;
            }
          } else {  // location_a != -1
            location_b = ml->linearize_coordinate(pushed_row, pushed_col);
            if (ml->my_map[location_b] == false) {
              pair<int, int> idxs = al->agentStartOrGoalAt(pushed_row, pushed_col);
              if (idxs.second != -1) {
                location_b = -1;
              } else {
                al->addAgent(ml->row_coordinate(location_a),
                             ml->col_coordinate(location_a),
                             ml->row_coordinate(location_b),
                             ml->col_coordinate(location_b));
                location_a = -1;
                location_b = -1;
              }
            } else {  // click was on an obstacle
              location_b = -1;
            }
          }
        }
      }  // ------------------------------------------------------------
      if (editRandomAgents == true) {  // ------------------------------
        if (location_a == -1) {
          location_a = ml->linearize_coordinate(pushed_row, pushed_col);
        } else {
          location_b = ml->linearize_coordinate(pushed_row, pushed_col);
          if (random_start_locs.size() == 0) {
            //            cout << "Gen Starts:" << endl;
            random_start_locs = getRandomLocs(location_a, location_b, num_of_agents_to_random_generate, true);
          } else {
            //            cout << "Gen Goals:" << endl;
            random_goal_locs = getRandomLocs(location_a, location_b, num_of_agents_to_random_generate, false);
            for (int i = 0; i < num_of_agents_to_random_generate; i++)
              al->addAgent(ml->row_coordinate(random_start_locs[i]),
                           ml->col_coordinate(random_start_locs[i]),
                           ml->row_coordinate(random_goal_locs[i]),
                           ml->col_coordinate(random_goal_locs[i]));
            random_start_locs.clear();
            random_goal_locs.clear();
          }
          location_a = -1;
          location_b = -1;
        }
      }  // ------------------------------------------------------------
      this->redraw();
      return 1;  // END FL_PUSH (note -- return 1 means we took care of the handle)
    }
  }  // end editTabFlag ---------------------------------------------------------------------
  
  return Fl_Gl_Window::handle(e);  // if we didn't take care of it, return the handle as is
}

// generate num_of_rand_ag valid locations between loc_a and loc_b
vector<int> GridViewer::getRandomLocs(int loc_a, int loc_b, int num_of_rand_ag, bool forStartLoc) {
  vector<int> retVal;
  if (loc_a == loc_b)
    return retVal;
  int left_col = ml->col_coordinate(loc_a) < ml->col_coordinate(loc_b) ? ml->col_coordinate(loc_a) : ml->col_coordinate(loc_b);
  int right_col = ml->col_coordinate(loc_a) > ml->col_coordinate(loc_b) ? ml->col_coordinate(loc_a) : ml->col_coordinate(loc_b);
  int upper_row = ml->row_coordinate(loc_a) < ml->row_coordinate(loc_b) ? ml->row_coordinate(loc_a) : ml->row_coordinate(loc_b);
  int lower_row = ml->row_coordinate(loc_a) > ml->row_coordinate(loc_b) ? ml->row_coordinate(loc_a) : ml->row_coordinate(loc_b);
  //  cout << "[" << left_col << "," << upper_row << "] to [" << right_col << "," << lower_row << "]" << endl;
  vector<int> possible_locs;
  for (int row = upper_row; row <= lower_row; row++) {
    for (int col = left_col; col <= right_col; col++) {
      if (ml->my_map[ ml->linearize_coordinate(row, col) ] == false) {  // if not a blocked cell
        pair<int, int> idx = al->agentStartOrGoalAt(row, col);
        if ( (forStartLoc == true && idx.first == -1) ||
             (forStartLoc == false && idx.second == -1) )
          possible_locs.push_back(ml->linearize_coordinate(row, col));
      }
    }
  }
  //  cout << "Generating " << num_of_rand_ag << " start/goal from " << possible_locs.size() << " cells" << endl;
  if ((int)possible_locs.size() >= num_of_rand_ag) {
    for (int i = 0; i < num_of_rand_ag; i++) {
      srand(time(NULL));
      int idx = rand() % possible_locs.size();
      //      cout << "[IDX:" << idx << "] =  " << possible_locs.at(idx) << endl;
      retVal.push_back(possible_locs.at(idx));
      possible_locs.erase(possible_locs.begin()+idx);
    }
  }
  return retVal;
}


// draw the direction vector [vx,vy] going out from the center of cell [row,col]
void GridViewer::drawDirectionVector(int row, int col, double vx, double vy) {
  double x1 = cell_d*col+robot_d;
  double y1 = cell_d*row+robot_d;
  double x2 = x1+vx*cell_d;
  double y2 = y1-vy*cell_d; //(note -- dy is positive for NORTH, but opengl is opposite. Hence the minus.)
  // draw a line [x1,y1] -- [x2,y2]
  glBegin(GL_LINES);
  glVertex2f(x1,y1);
  glVertex2f(x2,y2);
  glEnd();
  glBegin( GL_TRIANGLE_FAN );
  // draw a circle at [x2,y2]
  glVertex2f( x2 , y2 );
  for( float i = 0; i <= 2 * 3.1415 + 1; i += 1 )
    glVertex2f( x2 + sin( i ) * 4, y2 + cos( i ) * 4 );
  glEnd();
} // -----------------------------------------------------------------------

void GridViewer::drawDirectionMap() {
  glColor3f(1, 0.5, 0);  // orange
  for (int id = 0; id < ml->rows*ml->cols; id++)
    if ( (ml->my_map)[id] == false )  // if cell (id) is not blocked, print its DV
      drawDirectionVector(ml->row_coordinate(id),
                          ml->col_coordinate(id),
                          dir_map->dm[id].first,
                          dir_map->dm[id].second);
}

// draw an arrow from [i1,j1] to [i2,j2]
void GridViewer::drawArrow(int i1, int j1, int i2, int j2) {
  int x1 = cell_d*j1+robot_d;
  int y1 = cell_d*i1+robot_d;
  int x2 = cell_d*j2+robot_d;
  int y2 = cell_d*i2+robot_d;
  int arr_i1=0, arr_j1=0, arr_i2=0, arr_j2 = 0;
  if (i1 < i2) { // [x1,y1] ^ [x2,y2]
    arr_i1 = x2-arrow_size;
    arr_j1 = y2-arrow_size;
    arr_i2 = x2+arrow_size;
    arr_j2 = y2-arrow_size;
  }
  if (i1 > i2) { //  [x1,y1] _ [x2,y2]
    arr_i1 = x2-arrow_size;
    arr_j1 = y2+arrow_size;
    arr_i2 = x2+arrow_size;
    arr_j2 = y2+arrow_size;
  }
  if (j1 > j2) { //  [x1,y1] <-- [x2,y2]
    arr_i1 = x2+arrow_size;
    arr_j1 = y2+arrow_size;
    arr_i2 = x2+arrow_size;
    arr_j2 = y2-arrow_size;
  }
  if (j2 > j1) { //  [x1,y1] --> [x2,y2]
    arr_i1 = x2-arrow_size;
    arr_j1 = y2+arrow_size;
    arr_i2 = x2-arrow_size;
    arr_j2 = y2-arrow_size;      
  }
  glBegin(GL_LINES); // draw arrow-line
  glVertex2f(x1,y1);
  glVertex2f(x2,y2);
  glVertex2f(arr_i1,arr_j1);
  glVertex2f(x2,y2);
  glVertex2f(arr_i2,arr_j2);
  glVertex2f(x2,y2);
  glEnd();
} // --------------------------------------------------------------------------------

// draw a circle centered at [i1,j1] with radius r
void GridViewer::drawCircle(int i1, int j1, int r) {
  int y = cell_d*i1+robot_d;
  int x = cell_d*j1+robot_d;
  glBegin( GL_TRIANGLE_FAN );
  glVertex2f( x, y );
  for( float i = 0; i <= 2 * 3.1415 + 1; i += 1 )
    glVertex2f( x + sin( i ) * r, y + cos( i ) * r );
  glEnd();
} // --------------------------------------------------------------------------------


// draw the path found for agentNum (assumes it exists in AgentsLoader)
void GridViewer::drawPath(int agentNum) {
  const vector<int>* path;
  if (runCBS)
    path = (cbs_search->paths)[agentNum];
  else
    path = (ecbs_search->paths)[agentNum];
  glColor3f(0, 0, 1.0); //blue
  //  cout << "DRAW PATH. Length=" << path->size() << endl; fflush(stdout);
  for (size_t i=1; i<path->size(); i++) {
    //    int lin_coord = (this->ml)->linearize_coordinate( (this->ml)->row_coordinate(path->at(i)) , (this->ml)->col_coordinate(path->at(i)) );
    //    cout << "[" << (this->ml)->row_coordinate(path->at(i)) << "," << (this->ml)->col_coordinate(path->at(i)) << "] -->  " << lin_coord << " ; ";
    if ( path->at(i-1) == path->at(i) )
      drawCircle ( ml->row_coordinate(path->at(i)), ml->col_coordinate(path->at(i)) , arrow_size-1 ); // a wait action
    else
      drawArrow ( ml->row_coordinate(path->at(i-1)), ml->col_coordinate(path->at(i-1)), // a move action
		  ml->row_coordinate(path->at(i)), ml->col_coordinate(path->at(i)) );
  }
} // end drawPath -------------------------------------------------------


// draw the number of collisions in every grid
void GridViewer::drawCollisions() {
  vector < vector<int>* > paths;
  vector< tuple<int, int, int, int, int> >* collisions;
  if (runCBS)
    collisions = cbs_search->extractCollisions(al->num_of_agents);
  else
    collisions = ecbs_search->extractCollisions();

  double* collisionsMap = new double[ml->rows * ml->cols]();
  for ( size_t i = 0; i < collisions->size(); i++ ) {
    int agent1_id, agent2_id, location1, location2, timestep;
    tie(agent1_id, agent2_id, location1, location2, timestep) = collisions->at(i);
    if (location2 == -1) {  // plus one at location1 for vertex collision
      collisionsMap[location1]++;
    } else {  // plus half for each of the vertices for edge collision
      collisionsMap[location1] += 0.5;
      collisionsMap[location2] += 0.5;
    }
  }
  
  gl_font(1, 10);
  glColor3f(1.0, 1.0, 0);  // yellow
  for (int i = 1; i < ml->rows - 1; i++) {
    for (int j = 1; j < ml->cols - 1; j++) {
      int lin_coor = ml->linearize_coordinate(i, j);
      char* num_str = new char[4];
      sprintf(num_str, "%f", 4, collisionsMap[lin_coor]);
      glRasterPos2d(j*cell_d+1, (i+1)*cell_d-1);
      gl_draw(num_str, strlen(num_str));
    }
  }
  delete(collisionsMap);
}  // end drawCollisions ------------------------------------------------------

// draw the path found for agentNum (assumes it exists in AgentsLoader)
void GridViewer::drawHeuristic (int agentNum) {
  const SingleAgentSearch* sas = (cbs_search->search_engines)[agentNum];
  const double* h_vals = sas->my_heuristic;
  gl_font(1, 10);
  glColor3f(1.0, 1.0, 0); //yellow
  for (int i=0; i<ml->rows; i++)
    for (int j=0; j<ml->cols; j++) {
      int lin_coor = ml->linearize_coordinate(i,j);
      if (h_vals[lin_coor] != DBL_MAX) {
	char* num_str = new char[5];
	sprintf (num_str, "%f", 5, h_vals[lin_coor]);
	glRasterPos2d(j*cell_d+1,(i+1)*cell_d-1);
	gl_draw( num_str, strlen(num_str) );
      }
    }
} // end drawHeuristic ----------------------------------------------------

// draw the highway
void GridViewer::drawHighway ( vector< pair<int,int> >* edges) {
  glColor3f(1.0, 0, 0); //red
  for (vector< pair<int,int> >::const_iterator it = edges->begin(); it != edges->end(); ++it) {
    /*    cout << it->first << " --> " << it->second;
    cout << " : [" << ml->row_coordinate(it->first) << "," << ml->col_coordinate(it->first) << "] --> ";
    cout << "[" << ml->row_coordinate(it->second) << "," << ml->col_coordinate(it->second) << "]" << endl;
    */
    drawArrow ( ml->row_coordinate(it->first), ml->col_coordinate(it->first),
		ml->row_coordinate(it->second), ml->col_coordinate(it->second) );
  }
  delete(edges);
} // end drawHighway -------------------------------------------------------

void GridViewer::drawGrid ( int rows, int cols, bool* my_map ) {
  // draw grid's horizontal lines (note that the 0 (and rows) row is always blocked)
  glColor3f(1.0, 1.0, 1.0); //white
  for (int i=1; i<rows; i++) {
    glBegin(GL_LINE_STRIP);
    glVertex2f(cell_d,i*cell_d);
    glVertex2f(cell_d*(cols-1),i*cell_d);
    glEnd();
  }
  // draw grid's vertical lines
  for (int i=1; i<cols; i++) {
    glBegin(GL_LINE_STRIP);
    glVertex2f(i*cell_d,cell_d);
    glVertex2f(i*cell_d,cell_d*(rows-1));
    glEnd();
  }
  // draw obsticales (filled rectangles). Again, frame is always blocked...
  for (int i=1; i<rows-1; i++)
    for (int j=1; j<cols-1; j++)
      if ( my_map[i * cols + j] ) {
	glBegin(GL_QUADS);
	glVertex2f(j*cell_d, i*cell_d);
	glVertex2f(j*cell_d, (i+1)*cell_d);
	glVertex2f((j+1)*cell_d, (i+1)*cell_d);
	glVertex2f((j+1)*cell_d, i*cell_d);
	glEnd();
      }
}//drawGrid

// draw string (s) at given grid (i,j) with pixel offset (p_offset) and color (r,g,b) and font type (font_f) and size (font_s)
inline void GridViewer::drawString (string s, int i, int j, int p_offset, GLubyte r, GLubyte g, GLubyte b, int font_f, int font_s) {
  gl_font(font_f, font_s);
  glColor3ub(r, g, b);
  glRasterPos2d(j * cell_d + p_offset, i * cell_d + p_offset);
  gl_draw(s.c_str(), strlen(s.c_str()));
} // end drawString --------------------------------------------------------------


// draw the start and goal locations of agentNum (assumes it exists in AgentsLoader)
void GridViewer::drawStartGoal (int agentNum) {
  int start_row = (al->initial_locations[agentNum]).first;
  int start_col = (al->initial_locations[agentNum]).second;
  int goal_row = (al->goal_locations[agentNum]).first;
  int goal_col = (al->goal_locations[agentNum]).second;
  drawString ( "S" + std::to_string(agentNum), start_row, start_col, robot_d, 0, 255, 0, 1, 12);
  drawString ( "G" + std::to_string(agentNum), goal_row, goal_col, robot_d, 0, 255, 0, 1, 12);
} // end drawStartGoal --------------------------------------------------------------


// draw agentNum location for the given timestep (note -- if agentNum=num_of_agents it simulate all of them)
void GridViewer::drawLocationTimestep (int agentNum, int curr_timestep) {
  int from_agent, to_agent;
  if (agentNum < al->num_of_agents) { // print paths just for one agent
    from_agent = agentNum;
    to_agent = agentNum+1;
  } else { // print paths for all agents
    from_agent = 0;
    to_agent = al->num_of_agents;
  }
  //  cout << "(draw): #AGENTS=" << al->num_of_agents << " ; FROM:" << from_agent << " ; TO:" << to_agent << endl;
  for (int agent=from_agent; agent<to_agent; agent++) {
    vector<int>* agent_plan;
    if (runCBS)
      agent_plan = (cbs_search->paths)[agent];
    else
      agent_plan = (ecbs_search->paths)[agent];
    int row, col;
    if ( curr_timestep < (int)agent_plan->size() ) { // print current location
      row = ml->row_coordinate ( agent_plan->at(curr_timestep) );
      col = ml->col_coordinate ( agent_plan->at(curr_timestep) );
    } else { // plan ended (print the "stay" in goal location)
      row = ml->row_coordinate ( agent_plan->at(agent_plan->size()-1) );
      col = ml->col_coordinate ( agent_plan->at(agent_plan->size()-1) );
    }
    drawString ( "A" + std::to_string(agent), row, col, robot_d, 255, 255, 0, 1, 12);
    //    cout << "   A=" << agent << " ; T=" << curr_timestep << " ; [" << row << "," << col << "]" << endl;
  }
} // end simulatePathTimestep --------------------------------------------------------

void GridViewer::draw() {

    if (!valid()) {
      valid(1);
      glMatrixMode (GL_PROJECTION);
      glLoadIdentity ();
      glOrtho (0, w(), h(), 0, 0, 1);
      glMatrixMode (GL_MODELVIEW);
      glDisable(GL_DEPTH_TEST);
    }
    
    glClearColor(0.0, 0.0, 0.0, 0.0); //black
    glClear(GL_COLOR_BUFFER_BIT);

    //    glColor3f(1.0, 1.0, 1.0); //white

    // always draw the grid
    if (this->ml != NULL)
      drawGrid( (this->ml)->rows, (this->ml)->cols, (this->ml)->my_map );

    if (this->viewTabFlag == true) {
      if (this->displaySG == true && this->ml != NULL && this->al != NULL && agentNum != -1) {
        if (agentNum == al->num_of_agents)
          for (int ag=0; ag<al->num_of_agents; ag++)
            drawStartGoal(ag);
        else
          drawStartGoal(agentNum);
      }
      if ( this->displayHighway == true && this->egr != NULL )
        drawHighway ( (this->egr)->getAllEdges() );
      if ( this->displayHeuristic == true && this->cbs_search != NULL && agentNum < al->num_of_agents && agentNum > -1 ) {
        drawHeuristic ( agentNum );
      }
      if ( this->displayPath == true && this->cbs_search != NULL ) {
        if (agentNum == al->num_of_agents)
          for (int ag=0; ag<al->num_of_agents; ag++)
            drawPath(ag);
        else
          drawPath(agentNum);
        //      drawPath ( (this->search_engine)->getPath() );
        //      gl_font(1, 12);
        //      glColor3f(1.0, 1.0, 1.0);
        //      glRasterPos2d(10 , 500);
        //      string s = "cost=" + to_string(search_engine->path_cost) +
        //	" ; expanded=" + to_string(search_engine->num_expanded) +
        //	" ; generated=" + to_string(search_engine->num_generated);
        //      gl_draw(s.c_str(), strlen(s.c_str()));
      }
      // simPaths    
      if ( curr_simTimestep >= 0 && curr_simTimestep < max_simTimestep )
        this->drawLocationTimestep( agentNum , curr_simTimestep );
      if ( this->displayDirectionMap == true && this->dir_map != NULL )
        drawDirectionMap();
      if ( this->displayCollisions == true && this->cbs_search != NULL )
        drawCollisions();
    } // end viewTabFlag ------------------------------------------------

    if (this->editTabFlag == true) {
      if ( this->egr != NULL )
        drawHighway((this->egr)->getAllEdges());
      if ( this->al != NULL ) {
        for (int ag = 0; ag < al->num_of_agents; ag++)
          drawStartGoal(ag);
      }
    }  // end editTabFlag ------------------------------------------------

    this->redraw();
}

// computes heuristic and generate search engine objects if possible
void GridViewer::recomputeHeuristic() {
  if (this->cbs_search != NULL) {
    delete (this->cbs_search);
    this->cbs_search = NULL;
  }
  if (this->ecbs_search != NULL) {
    delete (this->ecbs_search);
    this->ecbs_search = NULL;
  }

  if ( this->ml != NULL && this->al != NULL && this->egr != NULL && this->hwy_weight >= 1 ) {
    this->cbs_search = new CBSSearch (*(this->ml),
                                      *(this->al),
                                      *(this->egr),
                                      this->hwy_weight,
                                      TWEAK_G_VAL);
  }
  if ( this->ml != NULL && this->al != NULL && this->egr != NULL && this->hwy_weight >= 1 && this->ecbs_weight >= 1 ) {
    this->ecbs_search = new ECBSSearch (*(this->ml),
                                        *(this->al),
                                        *(this->egr),
                                        this->hwy_weight,
                                        this->ecbs_weight,
                                        TWEAK_G_VAL);
  }
}  // end recomputeHeuristic()
