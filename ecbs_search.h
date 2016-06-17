// ECBS Search (High-level)
#ifndef ECBSSEARCH_H
#define ECBSSEARCH_H

#include <boost/heap/fibonacci_heap.hpp>
#include <cstring>
#include <climits>
#include <tuple>
#include <string>
#include <vector>
#include <list>
#include "map_loader.h"
#include "agents_loader.h"
#include "compute_heuristic.h"
#include "egraph_reader.h"
#include "single_type_search_ecbs.h"
#include "ecbs_node.h"

using boost::heap::fibonacci_heap;
using boost::heap::compare;
using std::cout;

class ECBSSearch {
public:
	double focal_w = 1.0;
	double focal_list_threshold;
	double min_sum_f_vals;

	typedef boost::heap::fibonacci_heap< ECBSNode*, boost::heap::compare<ECBSNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< ECBSNode*, boost::heap::compare<ECBSNode::secondary_compare_node> > heap_focal_t;
	typedef dense_hash_map<ECBSNode*, ECBSNode*, ECBSNode::ECBSNodeHasher, ECBSNode::ECBS_eqnode> hashtable_t;

	vector<vector < vector<int> >* > paths;  // agents paths (each entry [t][i] is a vector<int> which specify the locations on the path of agent i)
	vector<vector < vector<int> >* > paths_found_initially;  // contain initial paths found (that is, each with optimal policy)
	bool solution_found;
	double solution_cost;

	list < ECBSNode* > popped_nodes;  // used to clean the memory at the end
	ECBSNode* dummy_start;
	vector<vector <int> > start_locations;
	vector<vector <int> > goal_locations;

	const vector<bool> my_map;
	int map_size;
	int num_of_types;
	int current_makespan;
	int max_makespan;
	vector<int> nums_of_agents;
	const int* actions_offset;


	uint64_t HL_num_expanded = 0;
	uint64_t HL_num_generated = 0;
	uint64_t LL_num_expanded = 0;
	uint64_t LL_num_generated = 0;

	heap_open_t open_list;
	heap_focal_t focal_list;
	hashtable_t allNodes_table;

	// used in hash table and would be deleted from the d'tor
	ECBSNode* empty_node;
	ECBSNode* deleted_node;

	vector < SingleTypeSearchECBS* > search_engines;  // used to find (single) agents' paths

	vector <double> ll_min_f_vals_found_initially;  // contains initial ll_min_f_vals found
	vector <double> ll_min_f_vals;  // each entry [i] represent the lower bound found for agent[i]
	vector <double> paths_costs_found_initially;
	vector <double> paths_costs;

	tuple<int, int, int, int, int> earliest_conflict;

	ECBSSearch(MapLoader& ml, const AgentsLoader& al, const EgraphReader& egr, double e_w, double e_f, bool tweak_g_val = false);
	inline double compute_g_val(int num_of_types);
	inline void updatePaths(ECBSNode* curr, ECBSNode* root_node);
	inline bool updateECBSNode(ECBSNode* leaf_node, ECBSNode* root_node);
	bool runECBSSearch();
	inline bool ECBSSearch::switchedLocations(int type1_id, int agent1_id, int type2_id, int agent2_id, size_t timestep);
	inline int ECBSSearch::getAgentLocation(int type_id, int agent_id, size_t timestep);
	vector< tuple<int, int, int, int, int> >* extractCollisions(int nums_of_types);
	void printPaths();
	void updatePathsForExpTime(int t_exp);

	vector<vector<int>> getReservationTable(int exclude_type);

	void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);

	int computeNumOfCollidingTypes();

	inline void releaseClosedListNodes();

	~ECBSSearch();
};

#endif
