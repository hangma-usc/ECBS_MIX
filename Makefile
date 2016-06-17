CXX = g++ # the compiler 
CXXFLAGS = -Wall -DNDEBUG -L /home/lirchi/gurobi604/linux64/lib -lgurobi_c++ -lgurobi60 -I /home/lirchi/gurobi604/linux64/include -I /usr/include/boost/ -I /usr/local/lib -std=c++11 -lfltk -lfltk_gl -lglut -lGL -O3 # the compiler flags 
#add -DNDEBUG for better Boost performance (when done debugging)
#add -O3 for optimization (when done debugging)
#remove -g (when done debugging)

map_reader.o: map_loader.cpp map_loader.h
	$(CXX) $(CXXFLAGS) -c map_loader.cpp

agents_loader.o: agents_loader.cpp agents_loader.h
	$(CXX) $(CXXFLAGS) -c agents_loader.cpp

compute_heuristic.o: compute_heuristic.cpp compute_heuristic.h map_reader.o agents_loader.o egraph_reader.o
	$(CXX) $(CXXFLAGS) -c compute_heuristic.cpp

node.o: node.cpp node.h
	$(CXX) $(CXXFLAGS) -c node.cpp

egraph_reader.o: egraph_reader.cpp egraph_reader.h
	$(CXX) $(CXXFLAGS) -c egraph_reader.cpp

single_agent_search.o: single_agent_search.cpp single_agent_search.h compute_heuristic.o
	$(CXX) $(CXXFLAGS) -c single_agent_search.cpp

cbs_node.o: cbs_node.cpp cbs_node.h
	$(CXX) $(CXXFLAGS) -c cbs_node.cpp

cbs_search.o: cbs_search.cpp cbs_search.h cbs_node.o single_agent_search.o
	$(CXX) $(CXXFLAGS) -c cbs_search.cpp

direction_map.o: map_reader.o direction_map.h direction_map.cpp
	$(CXX) $(CXXFLAGS) -c direction_map.cpp

single_agent_ecbs.o: single_agent_ecbs.cpp single_agent_ecbs.h compute_heuristic.o
	$(CXX) $(CXXFLAGS) -c single_agent_ecbs.cpp

ecbs_node.o: ecbs_node.cpp ecbs_node.h
	$(CXX) $(CXXFLAGS) -c ecbs_node.cpp

ecbs_search.o: ecbs_search.cpp ecbs_search.h cbs_node.o single_agent_ecbs.o
	$(CXX) $(CXXFLAGS) -c ecbs_search.cpp

create_stp.o: create_stp.h create_stp.cpp
	$(CXX) $(CXXFLAGS) -c create_stp.cpp

clean:
	rm -rf *o *~ driver driver.log core

backup:
	tar czvf backup.tar.gz *h *cpp *cxx Makefile

GridViewer.o: GridViewer.h GridViewer.cxx
	$(CXX) $(CXXFLAGS) -c GridViewer.cxx

GUI_UI.o: GUI_UI.h GUI_UI.cxx GridViewer.o
	$(CXX) $(CXXFLAGS) -c GUI_UI.cxx

GUIMain.o: GUI_UI.o GUIMain.cxx
	$(CXX) $(CXXFLAGS) -c GUIMain.cxx

gui: GUIMain.o node.o map_loader.o agents_loader.o egraph_reader.o compute_heuristic.o node.o egraph_reader.o single_agent_search.o cbs_node.o cbs_search.o direction_map.o single_agent_ecbs.o ecbs_node.o ecbs_search.o create_stp.o
	$(CXX) GridViewer.o GUI_UI.o GUIMain.o map_loader.o agents_loader.o egraph_reader.o compute_heuristic.o single_agent_search.o node.o cbs_node.o cbs_search.o direction_map.o single_agent_ecbs.o ecbs_node.o ecbs_search.o create_stp.o $(CXXFLAGS) -o mapf_gui

driver: driver.o
	$(CXX) $(CXXFLAGS) -c driver.cpp

executable: driver.o map_loader.o agents_loader.o egraph_reader.o compute_heuristic.o single_agent_search.o node.o cbs_node.o cbs_search.o direction_map.o single_agent_ecbs.o ecbs_node.o ecbs_search.o create_stp.o
	$(CXX) driver.o map_loader.o agents_loader.o egraph_reader.o compute_heuristic.o single_agent_search.o node.o cbs_node.o cbs_search.o direction_map.o single_agent_ecbs.o ecbs_node.o ecbs_search.o create_stp.o $(CXXFLAGS) -o driver

profiler:
	echo "zzz" >> callgrind.out
	rm callgrind.out*
	valgrind --tool=callgrind ./driver corridor1.map corridor1.agents corridor1.hwy 2.0 1.1
	kcachegrind callgrind.out*
