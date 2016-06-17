#include <FL/Fl.H>
#include "GUI_UI.h"
#include "map_loader.h"
#include "compute_heuristic.h"
#include "egraph_reader.h"
#include "single_agent_search.h"
#include "agents_loader.h"
#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cstdlib>

int
main(int argc, char **argv) {

  GUI_UI *myGUI = new GUI_UI();

  if (argc > 1)
    (myGUI->mapFileName)->value(argv[1]);
  
  if (argc > 2)
    (myGUI->highwayFileName)->value(argv[2]);

  if (argc > 3)
    (myGUI->highwayWeight)->value(argv[3]);

  if (argc > 4)
    (myGUI->agentsFileName)->value(argv[4]);

  if (argc > 5)
    (myGUI->weightECBS)->value(argv[5]);
    
  //Initial global objects.
  
  Fl::visual(FL_DOUBLE|FL_INDEX);
  
  myGUI->show(0,NULL); // avoid the menu for show(argc, argv);
  
  return Fl::run();
}
