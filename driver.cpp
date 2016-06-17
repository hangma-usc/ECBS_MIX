#include "map_loader.h"
#include "agents_loader.h"
#include "compute_heuristic.h"
#include "egraph_reader.h"
#include "single_type_search.h"
#include "cbs_search.h"
#include "ecbs_search.h"
#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include "cbs_node.h"
#include "ecbs_node.h"
#include "create_stp.h"
#include <cstdlib>
#include <fstream>

using namespace std;

int main(int argc, char** argv) {

	cout << "driver map.txt agents.txt highway.txt hwy_weight focal_weight" << endl;
	cout << "note -- if highway.txt is CRISSCROSS (case sensitive) then a criss-cross highway is generated and used with the given weight." << endl;

	ofstream output;
	output.open("C:\\ECBS_MIX\\Release\\Instances\\30X30\\cbs_mix_output.txt", ios::app);

	// read the map file and construct its two-dim array
	MapLoader ml = MapLoader(argv[1]);
	// read agents' start and goal locations
	AgentsLoader al = AgentsLoader(argv[2]);
	EgraphReader egr;
	CBSSearch cbs = CBSSearch(ml, al, egr, 0, 0);
	bool res = cbs.runCBSSearch();
	if (res) {
		cout << "From Driver: Path found" << endl;
		cbs.printPaths();
		output << argv[1] << " " << cbs.solution_cost << " " << cbs.running_time << endl;
	}
	else {
		cout << "From Driver: NO Path found" << endl;
		output << argv[1] << " " << -1 << " " << cbs.running_time << endl;
	}

	output.close();
}


/*
int main(int argc, char** argv) {

	cout << "driver map.txt agents.txt highway.txt hwy_weight focal_weight" << endl;
	cout << "note -- if highway.txt is CRISSCROSS (case sensitive) then a criss-cross highway is generated and used with the given weight." << endl;

	ofstream output;
	output.open("C:\\ECBS_MIX\\Release\\Instances\\cbs_mix_output.txt", ios::app);
	bool continue_last_run = false;
	int agt;
	int instance;
	ifstream current_instance("C:\\ECBS_MIX\\Release\\Instances\\current_instance");
	if (current_instance.good()) {
		string line;
		getline(current_instance, line);
		stringstream linestream(line);
		linestream >> agt >> instance;
		continue_last_run = true;
	}
	current_instance.close();

	string map_name;
	string agent_name;
	for (int a = 10; a <= 50; a+=5) {
		for (int i = 1; i <= 50; i++) {
			if (continue_last_run) {
				a = agt;
				i = instance;
				continue_last_run = false;
				continue;
			}
			
			// read the map file and construct its two-dim array
			MapLoader ml = MapLoader("C:\\ECBS_MIX\\Release\\Instances\\map-30X30-10-" + to_string(a) + "-" + to_string(i));
			// read agents' start and goal locations
			AgentsLoader al = AgentsLoader("C:\\ECBS_MIX\\Release\\Instances\\agents-30X30-10-" + to_string(a) + "-" + to_string(i));
			EgraphReader egr;
			cout << "Computing solution for: 30X30-10-" + to_string(a) + "-" + to_string(i) << endl;
			CBSSearch cbs = CBSSearch(ml, al, egr, 0, 0);
			bool res = cbs.runCBSSearch();	
			if (res) {
				cout << "From Driver: Path found" << endl;
				output << cbs.solution_cost << " " << cbs.running_time << endl;
			}
			else {
				cout << "From Driver: NO Path found" << endl;
				output << -1 << " " << -1 << endl;
			}
			ofstream output_current;
			output_current.open("current_instance", ios::trunc);
			output_current << a << " " << i ;
			output_current.close();
		}
	}
	output.close();
}


int main(int argc, char** argv) {

	cout << "driver map.txt agents.txt highway.txt hwy_weight focal_weight" << endl;
	cout << "note -- if highway.txt is CRISSCROSS (case sensitive) then a criss-cross highway is generated and used with the given weight." << endl;

	ofstream output;
	output.open("C:\\ECBS_MIX\\Release\\Instances\\100\\cbs_mix_output.txt", ios::app);
	bool continue_last_run = false;
	int agt;
	int instance;
	ifstream current_instance("C:\\ECBS_MIX\\Release\\Instances\\100\\current_instance");
	if (current_instance.good()) {
		string line;
		getline(current_instance, line);
		stringstream linestream(line);
		linestream >> agt >> instance;
		continue_last_run = true;
	}
	current_instance.close();

	string map_name;
	string agent_name;
	int agents[] = { 50,25,20,10,5,4,2 };
	for (int a = 4; a < 5; a++) {
		for (int i = 1; i <= 50; i++) {
			if (continue_last_run) {
				a = agt;
				i = instance;
				continue_last_run = false;
				continue;
			}

			// read the map file and construct its two-dim array
			MapLoader ml = MapLoader("C:\\ECBS_MIX\\Release\\Instances\\100\\map-30X30-10-" + to_string(agents[a]) + "-" + to_string(i));
			// read agents' start and goal locations
			AgentsLoader al = AgentsLoader("C:\\ECBS_MIX\\Release\\Instances\\100\\agents-30X30-10-" + to_string(agents[a]) + "-" + to_string(i));
			EgraphReader egr;
			cout << "Computing solution for: 30X30-10-" + to_string(agents[a]) + "-" + to_string(i) << endl;
			CBSSearch cbs = CBSSearch(ml, al, egr, 0, 0);
			bool res = cbs.runCBSSearch();
			if (res) {
				cout << "From Driver: Path found" << endl;
				output << cbs.solution_cost << " " << cbs.running_time << endl;
			}
			else {
				cout << "From Driver: NO Path found" << endl;
				output << -1 << " " << cbs.running_time << endl;
			}
			ofstream output_current;
			output_current.open("current_instance", ios::trunc);
			output_current << a << " " << i;
			output_current.close();
		}
	}
	output.close();
}

*/
