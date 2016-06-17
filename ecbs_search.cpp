#include "ecbs_search.h"
#include <exception>
#include <iostream>
#include <utility>
#include <list>
#include <vector>
#include <tuple>
#include <cmath>

#include <boost/array.hpp>
#include <boost/graph/grid_graph.hpp>

using boost::heap::fibonacci_heap;
using boost::heap::compare;
using std::cout;


void ECBSSearch::printPaths() {
	for (size_t t = 0; t < paths.size(); t++) {
		for (size_t i = 0; i < paths[t]->size(); i++) {
			cout << "TYPE " << t << "AGENT " << i << " Path: ";
			for (vector<int>::const_iterator it = paths[t]->at(i).begin(); it != paths[t]->at(i).end(); ++it)
				std::cout << *it << ' ';
			cout << endl;
		}
	}
}

// computes g_val based on current paths
inline double ECBSSearch::compute_g_val(int num_of_types) {
	double retVal = 0;
	for (int i = 0; i < num_of_types; i++) {
		if (paths[i]->at(0).size() - 1 > retVal) {
			retVal = paths[i]->at(0).size() - 1;
		}
	}
	return retVal;
}


// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
inline void ECBSSearch::updatePaths(ECBSNode* curr, ECBSNode* root_node) {
	paths = paths_found_initially;
	paths_costs = paths_costs_found_initially;
	vector<bool> updated(paths.size());
	/* used for backtracking -- only update paths[i] if it wasn't updated before (that is, by a younger node)
	* because younger nodes take into account ancesstors' nodes constraints. */
	for (size_t i = 0; i < paths.size(); i++)
		updated.push_back(false);
	while (curr != root_node) {
		if (updated[curr->type_id] == false) {
			paths[curr->type_id] = &(curr->path);
			paths_costs[curr->type_id] = curr->path_cost;
			updated[curr->type_id] = true;
		}
		curr = curr->parent;
	}
}

// Used in the GUI
void ECBSSearch::updatePathsForExpTime(int t_exp) {
	if (t_exp > (int) HL_num_expanded || t_exp < 0)
		return;  // do nothing if there's no high-level node for the specified time_expanded

	ECBSNode* t_exp_node = NULL;
	for (list < ECBSNode* >::iterator it = popped_nodes.begin(); it != popped_nodes.end() && t_exp_node == NULL; it++)
		if ((*it)->time_expanded == t_exp)
			t_exp_node = *it;

	updatePaths(t_exp_node, dummy_start);
	printPaths();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// find all constraints on this agent (recursing to the root) and compute (and store) a path satisfying them.
// returns true only if such a path exists (otherwise false and path remain empty).
inline bool ECBSSearch::updateECBSNode(ECBSNode* leaf_node, ECBSNode* root_node) {
	// extract all constraints on leaf_node->agent_id
	list < tuple<int, int, int> > constraints;
	int type_id = leaf_node->type_id;
	ECBSNode* curr = leaf_node;
	while (curr != root_node) {
		if (curr->type_id == type_id)
			constraints.push_front(curr->constraint);
		curr = curr->parent;
	}

	// calc max_timestep
	int max_timestep = -1;
	for (list< tuple<int, int, int> >::iterator it = constraints.begin(); it != constraints.end(); it++)
		if (get<2>(*it) > max_timestep)
			max_timestep = get<2>(*it);

	// initialize a constraint vector of length max_timestep+1. Each entry is an empty list< pair<int,int> > (loc1,loc2)
	vector < list< pair<int, int> > >* cons_vec = new vector < list< pair<int, int> > >(max_timestep + 1, list< pair<int, int> >());
	for (list< tuple<int, int, int> >::iterator it = constraints.begin(); it != constraints.end(); it++)
		cons_vec->at(get<2>(*it)).push_back(make_pair(get<0>(*it), get<1>(*it)));

	// find a path w.r.t cons_vec
	if (search_engines[type_id]->findPath(focal_w, cons_vec, getReservationTable(type_id), leaf_node->parent->lowerbound) == true) {
		// update leaf's path to the one found
		leaf_node->path = vector<vector<int>>(*(search_engines[type_id]->getPath()));
		leaf_node->lowerbound = search_engines[type_id]->cost_lowerbound;
		current_makespan = (current_makespan < search_engines[type_id]->cost_lowerbound) ? search_engines[type_id]->cost_lowerbound : current_makespan;
		delete (cons_vec);
		vector<vector<int>>* temp_old_path = paths[type_id];
		paths[type_id] = &(leaf_node->path);
		leaf_node->g_val = (leaf_node->parent->g_val < paths_found_initially[type_id]->size()) ? paths_found_initially[type_id]->size() : leaf_node->parent->g_val;
		leaf_node->path_cost = search_engines[type_id]->path_cost;
		paths[type_id] = temp_old_path;
		return true;
	}

	return false;
}
////////////////////////////////////////////////////////////////////////////////

/*
return agent_id's location for the given timestep
Note -- if timestep is longer than its plan length,
then the location remains the same as its last cell)
*/
inline int ECBSSearch::getAgentLocation(int type_id, int agent_id, size_t timestep) {
	// if last timestep > plan length, agent remains in its last location
	if (timestep >= paths[type_id]->at(agent_id).size())
		return paths[type_id]->at(agent_id)[paths[type_id]->at(agent_id).size() - 1];
	// otherwise, return its location for that timestep
	return paths[type_id]->at(agent_id)[timestep];
}

/*
return true iff agent1 and agent2 switched locations at timestep [t,t+1]
*/
inline bool ECBSSearch::switchedLocations(int type1_id, int agent1_id, int type2_id, int agent2_id, size_t timestep) {
	// if both agents at their goal, they are done moving (cannot switch places)
	if (timestep >= paths[type1_id]->at(agent1_id).size() && timestep >= paths[type2_id]->at(agent2_id).size())
		return false;
	if (getAgentLocation(type1_id, agent1_id, timestep) == getAgentLocation(type2_id, agent2_id, timestep + 1) &&
		getAgentLocation(type1_id, agent1_id, timestep + 1) == getAgentLocation(type2_id, agent2_id, timestep))
		return true;
	return false;
}

/*
Emulate agents' paths and returns a vector of collisions
Note - a collision is a tuple of <int agent1_id, agent2_id, int location1, int location2, int timestep>).
Note - the tuple's location_2=-1 for vertex collision.
*/
vector< tuple<int, int, int, int, int> >* ECBSSearch::extractCollisions(int num_of_types) {
	vector< tuple<int, int, int, int, int> >* cons_found = new vector< tuple<int, int, int, int, int> >();
	earliest_conflict = make_tuple(-1, -1, -1, -1, INT_MAX);
	// check for vertex and edge collisions
	for (int t1 = 0; t1 < num_of_types; t1++) {
		for (int t2 = t1 + 1; t2 < num_of_types; t2++) {
			size_t max_path_length = paths[t1]->at(0).size() > paths[t2]->at(0).size() ? paths[t1]->at(0).size() : paths[t2]->at(0).size();
			for (size_t timestep = 0; timestep < max_path_length; timestep++) {
				for (int a1 = 0; a1 < paths[t1]->size(); a1++) {
					for (int a2 = 0; a2 < paths[t2]->size(); a2++) {
						if (getAgentLocation(t1, a1, timestep) == getAgentLocation(t2, a2, timestep)) {
							cons_found->push_back(make_tuple(t1, t2, getAgentLocation(t1, a1, timestep), -1, timestep));
							if ((int)timestep < std::get<4>(earliest_conflict)) {
								earliest_conflict = make_tuple(t1, t2, getAgentLocation(t1, a1, timestep), -1, timestep);
							}
						}
						if (switchedLocations(t1, a1, t2, a2, timestep)) {
							cons_found->push_back(make_tuple(t1, t2, getAgentLocation(t1, a1, timestep), getAgentLocation(t2, a2, timestep), timestep));
							if ((int)timestep < std::get<4>(earliest_conflict)) {
								earliest_conflict = make_tuple(t1, t2, getAgentLocation(t1, a1, timestep), getAgentLocation(t2, a2, timestep), timestep);
							}
						}
					}
				}
			}
		}
	}
	return cons_found;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ECBSSearch::ECBSSearch(MapLoader& ml, const AgentsLoader& al, const EgraphReader& egr, double e_w, double e_f, bool tweak_g_val) {
	focal_w = e_f;
	HL_num_expanded = 0;
	HL_num_generated = 0;
	num_of_types = al.num_of_types;
	nums_of_agents = al.nums_of_agents;
	map_size = ml.rows*ml.cols;
	solution_found = false;
	solution_cost = -1;
	current_makespan = 0;
	max_makespan = 4 * num_of_types * map_size;
	start_locations = vector<vector<int>>(num_of_types);
	goal_locations = vector<vector<int>>(num_of_types);
	search_engines = vector < SingleTypeSearchECBS* >(num_of_types);

	ll_min_f_vals = vector <double>(num_of_types);
	paths_costs = vector <double>(num_of_types);
	ll_min_f_vals_found_initially = vector <double>(num_of_types);
	paths_costs_found_initially = vector <double>(num_of_types);

	for (int t = 0; t < num_of_types; t++) {
		vector<int> init_locs_list(nums_of_agents[t]);
		vector<int> goal_locs_list(nums_of_agents[t]);
		for (int i = 0; i < nums_of_agents[t]; i++) {
			int init_loc = ml.linearize_coordinate(al.initial_locations[t][i].first, al.initial_locations[t][i].second);
			int goal_loc = ml.linearize_coordinate(al.goal_locations[t][i].first, al.goal_locations[t][i].second);
			init_locs_list[i] = init_loc;
			goal_locs_list[i] = goal_loc;
		}
		start_locations[t] = init_locs_list;
		goal_locations[t] = goal_locs_list;

		//determine starting makespan
		for (int i = 0; i < nums_of_agents[t]; i++) {
			int min_distance_to_a_goal = map_size;
			for (int j = 0; j < nums_of_agents[t]; j++) {
				int manhattan_distance = abs(ml.row_coordinate(init_locs_list[i]) - ml.row_coordinate(goal_locs_list[j])) + abs(ml.col_coordinate(init_locs_list[i]) - ml.col_coordinate(goal_locs_list[j]));
				min_distance_to_a_goal = (manhattan_distance < min_distance_to_a_goal) ? manhattan_distance : min_distance_to_a_goal;
			}
			current_makespan = (current_makespan < min_distance_to_a_goal) ? min_distance_to_a_goal : current_makespan;
		}
	}

	// initialize allNodes_table (hash table)
	empty_node = new ECBSNode();
	empty_node->time_generated = -2; empty_node->type_id = -2;
	deleted_node = new ECBSNode();
	deleted_node->time_generated = -3; deleted_node->type_id = -3;
	allNodes_table.set_empty_key(empty_node);
	allNodes_table.set_deleted_key(deleted_node);

	// initialize paths_found_initially (contain all individual optimal policies)
	paths_found_initially.resize(num_of_types);

	for (int t = 0; t < num_of_types; t++) {
		paths = paths_found_initially;

		search_engines[t] = new SingleTypeSearchECBS(start_locations[t], goal_locations[t],
			NULL,
			ml.get_map(), ml.rows, ml.cols, ml.actions_offset,
			&egr,
			e_w,
			tweak_g_val);

		if (search_engines[t]->findPath(focal_w, NULL, getReservationTable(t), current_makespan) == true) {
			paths_found_initially[t] = new vector<vector<int> >(*search_engines[t]->getPath());
			ll_min_f_vals_found_initially[t] = search_engines[t]->cost_lowerbound;
			current_makespan = (current_makespan < search_engines[t]->cost_lowerbound) ? search_engines[t]->cost_lowerbound : current_makespan;
			paths_costs_found_initially[t] = search_engines[t]->path_cost;
		}
	}

	paths = paths_found_initially;

	dummy_start = new ECBSNode();
	dummy_start->type_id = -1;
	dummy_start->g_val = current_makespan;//compute_g_val(num_of_types);
	dummy_start->lowerbound = current_makespan;
	dummy_start->open_handle = open_list.push(dummy_start);
	dummy_start->focal_handle = focal_list.push(dummy_start);

	focal_list_threshold = focal_w * current_makespan;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ECBSSearch::runECBSSearch() {

	std::clock_t start;
	double duration;
	start = std::clock();

	// start is already in the heap
	while (!focal_list.empty() && !solution_found) {
		ECBSNode* curr = focal_list.top();
		focal_list.pop();
		open_list.erase(curr->open_handle);
		HL_num_expanded++;
		curr->time_expanded = HL_num_expanded;

		updatePaths(curr, dummy_start);  // takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start
		vector< tuple<int, int, int, int, int> >* collision_vec = extractCollisions(num_of_types);  // check for collisions on updated paths
																									/*    for (vector< tuple<int,int,int,int,int> >::const_iterator it = collision_vec->begin(); it != collision_vec->end(); it++)																									cout << "A1:" << get<0>(*it) << " ; A2:" << get<1>(*it) << " ; L1:" << get<2>(*it) << " ; L2:" << get<3>(*it) << " ; T:" << get<4>(*it) << endl;    */
		if (collision_vec->size() == 0) {
			solution_found = true;
			solution_cost = curr->g_val;
		}
		else {
			int type1_id, type2_id, location1, location2, timestep;
			tie(type1_id, type2_id, location1, location2, timestep) = earliest_conflict;
			ECBSNode* n1 = new ECBSNode();
			ECBSNode* n2 = new ECBSNode();
			n1->type_id = type1_id;
			n2->type_id = type2_id;
			if (location2 == -1) {  // generate vertex constraint
				n1->constraint = make_tuple(location1, -1, timestep);
				n2->constraint = make_tuple(location1, -1, timestep);
			}
			else {  // generate edge constraint
				n1->constraint = make_tuple(location1, location2, timestep);
				n2->constraint = make_tuple(location2, location1, timestep);
			}
			n1->parent = curr;
			n2->parent = curr;

			// find all constraints on this agent (recursing to the root) and compute (and store) a path satisfying them. Also updates n1's g_val
			if (updateECBSNode(n1, dummy_start) == true) {
				n1->h_val = curr->h_val - paths_costs[n1->type_id] + n1->path_cost;

				n1->open_handle = open_list.push(n1);
				if (n1->sum_min_f_vals <= focal_list_threshold)
			}
			else {
				delete (n1);
			}
			// same for n2
			if (updateECBSNode(n2, dummy_start) == true) {
				n2->open_handle = heap.push(n2);
			}
			else {
				delete (n2);
			}

			//      cout << "It has found the following paths:" << endl;
			//      printPaths();
			//      cout << "First node generated: " << *n1 << "Second node generated: " << *n2 << endl;
		}
		delete (collision_vec);
	}
	if (solution_found) {
		cout << "FOUND COLLISION FREE PATH! Path cost:" << solution_cost << " ; High-level Expanded:" << num_expanded << endl;
		printPaths();
	}
	else {
		cout << "FAILED TO FIND A COLLISION FREE PATH :(" << " ; High-level Expanded:" << num_expanded << endl;
	}
	return solution_found;
}


// Generates a boolean reservation table for paths (cube of map_size*max_timestep).
// This is used by the low-level EECBS to count possible collisions efficiently
// Note -- we do not include the agent for which we are about to plan for
vector<vector<int>> ECBSSearch::getReservationTable(int exclude_type) {
	int relaxed_makespan = (int)(focal_w * current_makespan);
	vector<vector<int>> res_table(relaxed_makespan + 1, vector<int>(map_size, 0));
	for (size_t timestep = 0; timestep <= relaxed_makespan; timestep++) {
		for (int type = 0; type < num_of_types; type++) {
			if (type != exclude_type && paths[type] != NULL) {
				for (int agent = 0; agent < nums_of_agents[type]; agent++) {
					int loc = getAgentLocation(type, agent, timestep);
					res_table[timestep][loc] += 1;
				}
			}
		}
	}
	return res_table;
}

int ECBSSearch::computeNumOfCollidingTypes() {
	int retVal = 0;
	for (int t1 = 0; t1 < num_of_types; t1++) {
		for (int t2 = t1 + 1; t2 < num_of_types; t2++) {
			for (size_t timestep = 0; timestep < current_makespan; timestep++) {
				for (int a1 = 0; a1 < paths[t1]->size(); a1++) {
					for (int a2 = 0; a2 < paths[t2]->size(); a2++) {
						if (getAgentLocation(t1, a1, timestep) == getAgentLocation(t2, a2, timestep) ||
							switchedLocations(t1, a1, t2, a2, timestep)) {
							retVal++;
							timestep = current_makespan;
							a1 = paths[t1]->size();
							a2 = paths[t2]->size();
						}
					}
				}
			}
		}
	}
	return retVal;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ECBSSearch::~ECBSSearch() {
	for (size_t i = 0; i < search_engines.size(); i++)
		delete (search_engines[i]);
	for (size_t i = 0; i < paths_found_initially.size(); i++)
		delete (paths_found_initially[i]);
	//  for (size_t i=0; i<paths.size(); i++)
	//    delete (paths[i]);
	// clean heap memory and empty heap (if needed, that is if heap wasn't empty when solution found)
	for (heap_open_t::iterator it = heap.begin(); it != heap.end(); it++)
		delete (*it);
	heap.clear();
	// clean up other nodes (the ones that were popped out of the heap)
	//  cout << "Number of ECBS nodes expanded: " << popped_nodes.size() << endl;
	while (!popped_nodes.empty()) {
		delete popped_nodes.front();
		popped_nodes.pop_front();
	}
}
