#ifndef GUI_UI_h
#define GUI_UI_h

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Tabs.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_File_Input.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Check_Button.H>
#include <FL/Fl_Choice.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Native_File_Chooser.H>
#include <FL/Fl_Float_Input.H>
#include <FL/Fl_Int_Input.H>
#include <FL/fl_ask.H>

#include "GridViewer.h"
#include "map_loader.h"
#include "egraph_reader.h"
#include "compute_heuristic.h"
#include "single_agent_search.h"
#include "agents_loader.h"
#include "cbs_search.h"
#include "ecbs_search.h"


class GUI_UI {
public:
  static void Button_CB(Fl_Widget* flw, void* userdata);
  void real_callback(Fl_Widget *flw, void *userdata);
  ~GUI_UI();
  GUI_UI();
  Fl_Double_Window *mainWindow;
  Fl_Tabs *mainTab;
  Fl_Group *loadSaveTab;
  Fl_File_Input *mapFileName;
  Fl_File_Input *highwayFileName;
  Fl_Float_Input *highwayWeight;
  Fl_Group *viewGroup;
  Fl_Check_Button *viewSG;
  Fl_Check_Button *viewHighway;
  Fl_Check_Button *viewHeuristic;
  Fl_Choice *viewAgentChoice;
  Fl_Check_Button *viewPath;
  Fl_Check_Button *viewDirectionMap;
  Fl_Button *simulatePlans;
  Fl_Check_Button *viewCollisions;
  Fl_Button *solveCBS;
  Fl_Button *solveECBS;
  Fl_Button *loadButton;
  Fl_Button *saveButton;
  Fl_Button *newMapFileButton;
  Fl_Button *browseMapFile;
  Fl_Button *browseHighwayFile;
  Fl_Choice *viewHLNode;
  Fl_Button *browseAgentsFile;
  Fl_Int_Input *viewSimSpeed;
  Fl_File_Input *agentsFileName;
  Fl_Float_Input *weightECBS;
  Fl_Group *editTab;
  Fl_Check_Button *editObstacle;
  Fl_Check_Button *editHighway;
  Fl_Check_Button *editAgent;
  Fl_Check_Button *editRandomAgents;
  Fl_Check_Button *editCrissCrossHWY;
  Fl_Group *gridViewGroup;
  Fl_Box *gridViewBox;
  GridViewer *gridView;
  void show(int argc, char **argv);
  void clear_data_structures();
};
#endif
