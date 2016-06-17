#ifndef GRIDVIEWER_H
#define GRIDVIEWER_H
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/gl.h>
#include <FL/fl_draw.H>
#include "map_loader.h"
#include "egraph_reader.h"
#include "compute_heuristic.h"
#include "single_agent_search.h"
#include "agents_loader.h"
#include "cbs_search.h"
#include "ecbs_search.h"
#include "direction_map.h"
#include <vector>

class GridViewer : public Fl_Gl_Window {
 public:
  const int cell_d = 30;
  const int robot_d = cell_d/2;
  const int arrow_size = 5;
  bool displayHighway = false;
  bool displayHeuristic = false;
  bool displayPath = false;
  bool displaySG = false;
  bool displayDirectionMap = false;
  int agentNum = -1;
  int HLNode = -1;
  bool displayCollisions = false;
  bool editObstacle = false;
  bool editHighway = false;
  bool editAgent = false;
  bool editRandomAgents = false;
  bool viewTabFlag = false;
  bool editTabFlag = false;
  int simSpeed = 1;
  int curr_simTimestep = -1;
  int max_simTimestep = -1;
  int num_of_agents_to_random_generate = -1;
  vector<int> random_start_locs, random_goal_locs;

  // used to construct eg edges
  int location_a = -1;
  int location_b = -1;

  // used to determine if we run CBS or ECBS
  bool runCBS = false;

  double hwy_weight = -1.0;
  double ecbs_weight = -1.0;
  MapLoader* ml = NULL;
  AgentsLoader* al = NULL;
  EgraphReader* egr = NULL;
  CBSSearch* cbs_search = NULL;
  DirectionMap* dir_map = NULL;
  ECBSSearch* ecbs_search = NULL;
  
  GridViewer(int x,int y,int w,int h,const char *l=0);
  /*The widget class draw() override.
   *
   *The draw() function initialize Gl for another round of drawing
   * then calls specialized functions for drawing each of the
   * entities displayed in the cube view.
   *
   */
  void draw();

  int handle (int e);

  void drawGrid ( int rows, int cols, bool* my_map );
  void drawStartGoal (int agentNum);
  void drawHighway ( vector< pair<int,int> >* );
  void drawHeuristic (int agentNum);
  void drawPath(int agentNum);
  void drawArrow(int i1, int j1, int i2, int j2);
  void drawDirectionVector(int row, int col, double vx, double vy);
  void drawDirectionMap();
  void drawCircle(int i, int j, int r);
  void recomputeHeuristic();
  void drawString (string s, int i, int j, int p_offset, GLubyte r, GLubyte g, GLubyte b, int font_f, int font_s);
  void drawLocationTimestep (int agentNum, int curr_timestep);
  void drawCollisions();
  vector<int> getRandomLocs(int loc_a, int loc_b, int num_of_rand_ag, bool forStartLoc);

  static void Timer_CB(void *userdata) {
    GridViewer *gv = (GridViewer*)userdata;
    //    cout << "(Timer) : A=" << gv->agentNum << " ; T=" << gv->curr_simTimestep << " ; MAX_T=" << gv->max_simTimestep << endl;
    if ( gv->curr_simTimestep >= 0 && gv->curr_simTimestep < gv->max_simTimestep ) {
      gv->curr_simTimestep++;
      gv->redraw();
      Fl::repeat_timeout(0.5, Timer_CB, userdata);
    } else {
      Fl::remove_timeout(Timer_CB);
      gv->curr_simTimestep = -1;
      gv->redraw();
    }
  }
  
  
};
#endif

