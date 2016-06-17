#include "single_type_search.h"
#include <cstring>
#include <climits>
#include <vector>
#include <list>
#include <utility>

#include <assert.h>
#include <boost/graph/copy.hpp>
#include <boost/graph/cycle_canceling.hpp>
#include <boost/graph/edmonds_karp_max_flow.hpp>
#include <boost/graph/find_flow_cost.hpp>

#include <boost/graph/push_relabel_max_flow.hpp>

#include <boost/graph/successive_shortest_path_nonnegative_weights.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/iteration_macros.hpp>

#include <boost/heap/fibonacci_heap.hpp>
#include <sparsehash/dense_hash_map>

#include <boost/graph/graphviz.hpp>

#include "node.h"

using google::dense_hash_map;      // namespace where class lives by default
using std::cout;
using std::endl;
using boost::heap::fibonacci_heap;


SingleTypeSearch::SingleTypeSearch(vector<int> s_locs, vector<int> g_locs, const double* heuristic,
	const vector<bool> & my_m, int rows, int cols, const int* actions,
	const EgraphReader* egr, double e_w, bool tweak) :
	my_heuristic(heuristic), my_map(my_m), actions_offset(actions), egr(egr) {
	this->start_locations = s_locs;
	this->goal_locations = g_locs;
	this->rows = rows;
	this->cols = cols;
	this->map_size = rows * cols;
	this->e_weight = e_w;
	this->tweak_g_val = tweak;
	this->path_cost = 0;

	this->num_agents = start_locations.size();
	this->path = new vector<vector<int>>(num_agents);
	this->current_timestep = 0;

	capacities = boost::get(boost::edge_capacity, time_expanded_graph);
	rev_edges = boost::get(boost::edge_reverse, time_expanded_graph);
	residual_capacities = boost::get(boost::edge_residual_capacity, time_expanded_graph);
	weights = boost::get(boost::edge_weight, time_expanded_graph);

	VertexProperties* empty_node = new VertexProperties{ -1, -1, false, false, false, -1 };
	VertexProperties* deleted_node = new VertexProperties{ -2, -2, false, false, false, -2 };
	vertex_index_map.set_empty_key(empty_node);
	vertex_index_map.set_deleted_key(deleted_node);

	EdgeAdder edge_adder(time_expanded_graph, weights, capacities, rev_edges, residual_capacities);

	for each (int location in start_locations) {
		VertexProperties* in_node_pt = new VertexProperties{ location, -1, true, false, false, 0 };//create out_node for time 0
		VertexProperties* out_node_pt = new VertexProperties{ location, -1, false, true, false, 0 };//create out_node for time 0
		Graph_Vertex_t in_node = addVertex(time_expanded_graph, in_node_pt);
		Graph_Vertex_t out_node = getVertex(time_expanded_graph, out_node_pt);
		edge_adder.addEdge(in_node, out_node, 0, 1); 

	}

}

long SingleTypeSearch::getVertexIndex(VertexProperties* v_pt) {
	return (long)((((v_pt->loc1 * map_size + v_pt->loc2) * 10000 + v_pt->timestep) * 2 + v_pt->is_in_node) * 2 + v_pt->is_out_node) * 2 + v_pt->is_u_node;
}

inline bool SingleTypeSearch::existVertex(Graph_t & g, VertexProperties* v_pt) {
	if (vertex_index_map.find(v_pt) == vertex_index_map.end()) {
		return false;
	}
	return true;
}

SingleTypeSearch::Graph_Vertex_t SingleTypeSearch::getVertex(Graph_t & g, VertexProperties* v_pt) {
	if (!existVertex(g, v_pt)) {
		Graph_Vertex_t v = boost::add_vertex(g);
		g[v] = *v_pt;
		vertex_index_map[v_pt] = v;
		return v;
	}
	else {
		return vertex_index_map[v_pt];
	}
}

SingleTypeSearch::Graph_Vertex_t SingleTypeSearch::addVertex(Graph_t & g, VertexProperties* v_pt) {
	Graph_Vertex_t v = boost::add_vertex(g);
	g[v] = *v_pt;
	vertex_index_map[v_pt] = v;
	return v;
}

inline bool SingleTypeSearch::isDirectionAllowed(int location, int direction) {
	int row_coordinate = location / cols;
	int col_coordinate = location % cols;
	if (row_coordinate == 0 && direction == 1) { //north
		return false;
	}
	if (col_coordinate == cols - 1 && direction == 2) { //east
		return false;
	}
	if (row_coordinate == rows - 1 && direction == 3) { //south
		return false;
	}
	if (col_coordinate == 0 && direction == 4) { //west
		return false;
	}
	return true;
}

void SingleTypeSearch::setUntillLayer(vector<vector<int>> & res_table, int layer_time_step) {
	//updateReachableSets(layer_time_step);
	EdgeAdder edge_adder(time_expanded_graph, weights, capacities, rev_edges, residual_capacities);
	for (current_timestep; current_timestep < layer_time_step; current_timestep++) {
		//setHashtable(current_timestep);
		for (int current_location = 0; current_location < map_size; current_location++) {
			VertexProperties* current_out_node_pt = new VertexProperties{ current_location, -1, false, true, false, current_timestep };
			if (existVertex(time_expanded_graph, current_out_node_pt)) { // exist current_out_node
				Graph_Vertex_t current_out_node = vertex_index_map[current_out_node_pt];
				for (int direction = 0; direction < 5; direction++) {
					int location = current_location + actions_offset[direction];
					if (isDirectionAllowed(current_location, direction) && !my_map[location] /*&& reachable_locations[current_timestep + 1][location]*/) { // location is valid as a target from current_location
						VertexProperties* in_node_pt = new VertexProperties{ location, -1, true, false, false, current_timestep + 1 };
						Graph_Vertex_t in_node;//get in_node for current_time + 1
						if (!existVertex(time_expanded_graph, in_node_pt)) {
							in_node = addVertex(time_expanded_graph, in_node_pt);
							VertexProperties* out_node_pt = new VertexProperties{ location, -1, false, true, false, current_timestep + 1 };
							Graph_Vertex_t out_node = getVertex(time_expanded_graph, out_node_pt);
							edge_adder.addEdge(in_node, out_node, res_table[current_timestep + 1][location], 1); //add edge from in to out, with cost being number of conflicts to existing paths
						}
						else {
							in_node = vertex_index_map[in_node_pt];
						}
						if (direction > 0) {
							VertexProperties* u_node_pt;
							VertexProperties* v_node_pt;
							// create gadget
							if (current_location <= location) {
								u_node_pt = new VertexProperties{ current_location, location, false, false, true, current_timestep };
								v_node_pt = new VertexProperties{ current_location, location, false, false, false, current_timestep };
							}
							else {
								u_node_pt = new VertexProperties{ location, current_location, false, false, true, current_timestep };
								v_node_pt = new VertexProperties{ location, current_location, false, false, false, current_timestep };
							}
							Graph_Vertex_t u_node, v_node;

							if (!existVertex(time_expanded_graph, u_node_pt)) {
								u_node = addVertex(time_expanded_graph, u_node_pt);//create u_node for current_time 
								v_node = addVertex(time_expanded_graph, v_node_pt);//create v_node for current_time
								edge_adder.addEdge(u_node, v_node, 0, 1);
							}
							else {								
								assert(existVertex(v_node_pt));
								u_node = vertex_index_map[u_node_pt];
								v_node = vertex_index_map[v_node_pt];
							}
							edge_adder.addEdge(current_out_node, u_node, 0, 1);
							edge_adder.addEdge(v_node, in_node, 0, 1);
						}
						else {
							// direction = 4 (stay)
							edge_adder.addEdge(current_out_node, in_node, 0, 1);// no gadget, out_node to next in_node
						}
					}
				}
			}
		}
	}
}

/*
	void SingleTypeSearch::setHashtable(int layer_time_step) {
		hashtable_t hash;
		VertexProperties* empty_node = new VertexProperties{ -1, -1, false, false, false, layer_time_step };
		VertexProperties* deleted_node = new VertexProperties{ -2, -2, false, false, false, layer_time_step };
		hash.set_empty_key(empty_node);
		hash.set_deleted_key(deleted_node);
		vertex_index_maps.push_back(hash);
	}


	SingleTypeSearch::Graph_Vertex_t SingleTypeSearch::addNode(VertexProperties* vp, int layer_time_tep) {
		Graph_Vertex_t vertex = boost::add_vertex(*vp, time_expanded_graph);
		time_expanded_graph[*vp] = *vp;
		nodes_hashtables[layer_time_tep][vp] = vertex;
		return vertex;
	}


	void SingleTypeSearch::updatePath(Node* goal) {
		path.clear();
		Node* curr = goal;
		// cout << "   UPDATING Path for one agent to: ";
		while (curr->timestep != 0) {
			path.push_back(curr->id);
			// cout << curr->id << ",";
			curr = curr->parent;
		}
		path.push_back(start_location);
		// cout << start_location << endl;
		reverse(path.begin(), path.end());
		path_cost = goal->g_val;
	}

	inline void SingleTypeSearch::releaseClosedListNodes(hashtable_t* allNodes_table) {
		hashtable_t::iterator it;
		for (it = allNodes_table->begin(); it != allNodes_table->end(); it++) {
			delete ((*it).second);  // Node* s = (*it).first; delete (s); (note -- it.first is the key and it.second is value)
		}
	}


	// iterate over the constraints ( cons[t] is a list of all constraints for timestep t) and return the latest
	// timestep which has a constraint involving the goal location
	int SingleTypeSearch::extractLastGoalTimestep(int goal_location, const vector< list< pair<int, int> > >* cons) {
		if (cons != NULL) {
			for (int t = static_cast<int>(cons->size()) - 1; t > 0; t--) {
				for (list< pair<int, int> >::const_iterator it = cons->at(t).begin(); it != cons->at(t).end(); ++it) {
					// $$$: in the following if, do we need to check second (maybe cannot happen in edge constraints?)
					if ((*it).first == goal_location || (*it).second == goal_location) {
						return (t);
					}
				}
			}
		}
		return -1;
	}
	*/

	/*
	  Deprecated -- extractLastGoalTimestep used instead and is more efficient.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Return true if there are constraints that involve the goal_location after the curr_timestep
	inline bool SingleTypeSearch::checkFutureConstraints(int goal_location, int curr_timestep, const vector< list< pair<int, int> > >* cons) {
	  // cout << "CURR_T:" << curr_timestep << " ; CONS_SIZE:" << cons->size() << endl; fflush(stdout);
	  if (cons != NULL) {
		if ( curr_timestep < static_cast<int>(cons->size()) ) {
		  for (int t = curr_timestep; t < static_cast<int>(cons->size()); t++) {
			for (list< pair<int, int> >::const_iterator it = cons->at(t).begin(); it != cons->at(t).end(); ++it) {
			  // $$$: in the following if, do we need to check second (maybe cannot happen in edge constraints?)
			  if ((*it).first == goal_location || (*it).second == goal_location) {
				return true;
			  }
			}
		  }
		}
	  }
	  return false;
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	*/

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// input: curr_id (location at time next_timestep-1) ; next_id (location at time next_timestep); next_timestep
	//        cons[timestep] is a list of <loc1,loc2> of (vertex/edge) constraints for that timestep.
inline bool SingleTypeSearch::isConstrained(int curr_id, int next_id, int next_timestep, const vector< list< pair<int, int> > >* cons) {
	//  cout << "check if ID="<<id<<" is occupied at TIMESTEP="<<timestep<<endl;
	if (cons == NULL)
		return false;

	// check vertex constraints (being in next_id at next_timestep is disallowed)
	if (next_timestep < static_cast<int>(cons->size())) {
		for (list< pair<int, int> >::const_iterator it = cons->at(next_timestep).begin(); it != cons->at(next_timestep).end(); ++it) {
			if ((*it).second == -1) {
				if ((*it).first == next_id) {
					return true;
				}
			}
		}
	}

	// check edge constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
	if (next_timestep > 0 && next_timestep - 1 < static_cast<int>(cons->size())) {
		for (list< pair<int, int> >::const_iterator it = cons->at(next_timestep - 1).begin(); it != cons->at(next_timestep - 1).end(); ++it) {
			if ((*it).first == curr_id && (*it).second == next_id) {
				return true;
			}
		}
	}

	return false;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// return true if a path found (and updates vector<int> path) or false if no path exists
bool SingleTypeSearch::findPath(const vector < list< pair<int, int> > >* constraints, vector<vector<int>> & res_table, int cost_upperbound) {

	assert(current_timestep <= cost_upperbound);
	setUntillLayer(res_table, cost_upperbound);
	assert(current_timestep == cost_upperbound);
	Graph_t modified_time_expanded_graph;
	boost::copy_graph(time_expanded_graph, modified_time_expanded_graph);

	// current constraints
	for (int t = 0; t < constraints->size(); t++) {
		for (list< pair<int, int> >::const_iterator it = constraints->at(t).begin(); it != constraints->at(t).end(); ++it) {
			if (it->second == -1) {// vertex conflict, remove the edge from in_node to out_node					
				VertexProperties* in_node_pt = new VertexProperties{ it->first, -1, true, false, false, t };
				VertexProperties* out_node_pt = new VertexProperties{ it->first, -1, false, true, false, t };
				Graph_Vertex_t in_node = vertex_index_map[in_node_pt];
				Graph_Vertex_t out_node = vertex_index_map[out_node_pt];
				boost::remove_edge(in_node, out_node, modified_time_expanded_graph);
				boost::remove_edge(out_node, in_node, modified_time_expanded_graph);
			}
			else {// edge conflict, remove the edges from t th out_node to u_node and v_node to t+1 th in_node
				VertexProperties* out_node_pt = new VertexProperties{ it->first, -1, false, true, false, t };

				VertexProperties* u_node_pt;
				VertexProperties* v_node_pt;
				if (it->first <= it->second) {
					u_node_pt = new VertexProperties{ it->first, it->second, false, false, true, t };
					v_node_pt = new VertexProperties{ it->first, it->second, false, false, false, t };
				}
				else {
					u_node_pt = new VertexProperties{ it->second, it->first, false, false, true, t };
					v_node_pt = new VertexProperties{ it->second, it->first, false, false, false, t };
				}
				VertexProperties* in_node_pt = new VertexProperties{ it->first, -1, true, false, false, t + 1 };
				Graph_Vertex_t out_node = vertex_index_map[out_node_pt];
				Graph_Vertex_t u_node = vertex_index_map[u_node_pt];
				Graph_Vertex_t v_node = vertex_index_map[v_node_pt];
				Graph_Vertex_t in_node = vertex_index_map[in_node_pt];
				boost::remove_edge(out_node, u_node, modified_time_expanded_graph);
				boost::remove_edge(u_node, out_node, modified_time_expanded_graph);
				boost::remove_edge(v_node, in_node, modified_time_expanded_graph);
				boost::remove_edge(in_node, v_node, modified_time_expanded_graph);
			}
		}
	}

	EdgeAdder edge_adder(
		modified_time_expanded_graph,
		boost::get(boost::edge_weight, modified_time_expanded_graph),
		boost::get(boost::edge_capacity, modified_time_expanded_graph),
		boost::get(boost::edge_reverse, modified_time_expanded_graph),
		boost::get(boost::edge_residual_capacity, modified_time_expanded_graph)
		);

	// source and target
	VertexProperties* source_node_pt = new VertexProperties{ -3, -3, false, false, false, -3 };
	VertexProperties* terminal_node_pt = new VertexProperties{ -4, -4, false, false, false, -4 };
	Graph_Vertex_t source_node = boost::add_vertex(*source_node_pt, modified_time_expanded_graph);
	Graph_Vertex_t terminal_node = boost::add_vertex(*terminal_node_pt, modified_time_expanded_graph);
	for (int agt = 0; agt < num_agents; agt++) {
		// add edges from source node to time 0 out node
		VertexProperties* out_node_0_pt = new VertexProperties{ start_locations[agt], -1, false, true, false, 0 };
		edge_adder.addEdge(source_node, vertex_index_map[out_node_0_pt], 0, map_size);

		// add edges from time current_timestep out node to terminal node
		VertexProperties* out_node_t_pt = new VertexProperties{ goal_locations[agt], -1, false, true, false, current_timestep };
		if (!existVertex(modified_time_expanded_graph, out_node_t_pt)) { //can't even reach goal location
			path->clear();
			modified_time_expanded_graph.clear();
			return false;
		}
		Graph_Vertex_t out_node_t = vertex_index_map[out_node_t_pt];
		edge_adder.addEdge(out_node_t, terminal_node, 0, map_size);
	}

	boost::successive_shortest_path_nonnegative_weights(modified_time_expanded_graph, source_node, terminal_node);

	Capacity cap = boost::get(boost::edge_capacity, modified_time_expanded_graph);
	ResidualCapacity re_cap = boost::get(boost::edge_residual_capacity, modified_time_expanded_graph);
	//path_cost = boost::find_flow_cost(modified_time_expanded_graph);

	int maxflow_value = 0;
	for each(int location in goal_locations) {
		VertexProperties* out_node_t_pt = new VertexProperties{ location, -1, false, true, false, current_timestep };
		Graph_Vertex_t out_node_t = vertex_index_map[out_node_t_pt];
		Graph_Edge_t edge = boost::edge(out_node_t, terminal_node, modified_time_expanded_graph).first;
		if (cap[edge] > 0 && cap[edge] - re_cap[edge] > 0) {
			maxflow_value++;
		}
	}


	if (maxflow_value < num_agents) {
		// no path found
		path->clear();
		modified_time_expanded_graph.clear();
		return false;
	}

	Graph_t::out_edge_iterator e_iter, e_end;

	path = new vector<vector<int>>(num_agents);

	for (int agt = 0; agt < num_agents; agt++) {
		int location = start_locations[agt];
		path->at(agt).push_back(location);
		VertexProperties* out_node_pt = new VertexProperties{ location, -1, false, true, false, 0 };
		for (int t = 0; t <= current_timestep; t++) {
			for (boost::tie(e_iter, e_end) = boost::out_edges(vertex_index_map[out_node_pt], modified_time_expanded_graph); e_iter != e_end; ++e_iter) {
				if (cap[*e_iter] > 0 && cap[*e_iter] - re_cap[*e_iter] > 0) {
					std::cout << "Source: " << vertex_index_map[out_node_pt] << " destination: " << boost::target(*e_iter, modified_time_expanded_graph) << " capacity: " << cap[*e_iter] << "residual cap: " << re_cap[*e_iter] << " used capacity: "
						<< (cap[*e_iter] - re_cap[*e_iter]) << std::endl;

					Graph_Vertex_t target_node = boost::target(*e_iter, modified_time_expanded_graph);
					VertexProperties target_node_property = modified_time_expanded_graph[target_node];
					if (target_node_property.is_in_node) { // same location for next t
						path->at(agt).push_back(location);
						out_node_pt = new VertexProperties{ location, -1, false, true, false, t + 1 };
						break;
					}
					else if (target_node_property.is_u_node) {
						location = (target_node_property.loc1 == location) ? target_node_property.loc2 : location;
						path->at(agt).push_back(location);
						out_node_pt = new VertexProperties{ location, -1, false, true, false, t + 1 };
						break;
					}
				}
			}
		}
	}

	//printPaths();
	modified_time_expanded_graph.clear();
	return true;
}

bool SingleTypeSearch::findPath(vector<vector<int>> & res_table, int cost_upperbound) {

	assert(current_timestep <= cost_upperbound);
	setUntillLayer(res_table, cost_upperbound);
	assert(current_timestep == cost_upperbound);
	Graph_t modified_time_expanded_graph;
	boost::copy_graph(time_expanded_graph, modified_time_expanded_graph);

	EdgeAdder edge_adder(
		modified_time_expanded_graph,
		boost::get(boost::edge_weight, modified_time_expanded_graph),
		boost::get(boost::edge_capacity, modified_time_expanded_graph),
		boost::get(boost::edge_reverse, modified_time_expanded_graph),
		boost::get(boost::edge_residual_capacity, modified_time_expanded_graph)
		);

	// source and target
	VertexProperties* source_node_pt = new VertexProperties{ -3, -3, false, false, false, -3 };
	VertexProperties* terminal_node_pt = new VertexProperties{ -4, -4, false, false, false, -4 };
	Graph_Vertex_t source_node = boost::add_vertex(*source_node_pt, modified_time_expanded_graph);
	Graph_Vertex_t terminal_node = boost::add_vertex(*terminal_node_pt, modified_time_expanded_graph);
	for (int agt = 0; agt < num_agents; agt++) {
		// add edges from source node to time 0 out node
		VertexProperties* in_node_0_pt = new VertexProperties{ start_locations[agt], -1, true, false, false, 0 };
		edge_adder.addEdge(source_node, vertex_index_map[in_node_0_pt], 0, map_size);

		// add edges from time current_timestep out node to terminal node
		VertexProperties* out_node_t_pt = new VertexProperties{ goal_locations[agt], -1, false, true, false, current_timestep };
		if (!existVertex(modified_time_expanded_graph, out_node_t_pt)) { //can't even reach goal location
			modified_time_expanded_graph.clear();
			path->clear();
			return false;
		}
		Graph_Vertex_t out_node_t = vertex_index_map[out_node_t_pt];
		edge_adder.addEdge(out_node_t, terminal_node, 0, map_size);
	}

	//myVertexWriter<Graph_t> vw(modified_time_expanded_graph);
	//boost::write_graphviz(std::cout, modified_time_expanded_graph, vw, boost::make_label_writer(boost::get(boost::edge_capacity, modified_time_expanded_graph)));	
	
	
	boost::successive_shortest_path_nonnegative_weights(modified_time_expanded_graph, source_node, terminal_node);

	Capacity cap = boost::get(boost::edge_capacity, modified_time_expanded_graph);
	ResidualCapacity re_cap = boost::get(boost::edge_residual_capacity, modified_time_expanded_graph);
	path_cost = boost::find_flow_cost(modified_time_expanded_graph);

	
	int maxflow_value = 0;
	for each(int location in goal_locations) {
		VertexProperties* out_node_t_pt = new VertexProperties{ location, -1, false, true, false, current_timestep };
		Graph_Vertex_t out_node_t = vertex_index_map[out_node_t_pt];
		Graph_Edge_t edge = boost::edge(out_node_t, terminal_node, modified_time_expanded_graph).first;
		if (cap[edge] > 0 && cap[edge] - re_cap[edge] > 0) {
			maxflow_value++;
		}
	}

	/*long maxflow_value = boost::edmonds_karp_max_flow(modified_time_expanded_graph, source_node, terminal_node);

	if (maxflow_value < num_agents) {
		// no path found
		path->clear();
		modified_time_expanded_graph.clear();
		return false;
	}

	boost::cycle_canceling(modified_time_expanded_graph);
	*/
		
	Graph_t::out_edge_iterator e_iter, e_end;

	path = new vector<vector<int>>(num_agents);

	for (int agt = 0; agt < num_agents; agt++) {
		int location = start_locations[agt];
		path->at(agt).push_back(location);
		VertexProperties* out_node_pt = new VertexProperties{ location, -1, false, true, false, 0 };
		for (int t = 0; t < current_timestep; t++) {
			for (boost::tie(e_iter, e_end) = boost::out_edges(vertex_index_map[out_node_pt], modified_time_expanded_graph); e_iter != e_end; ++e_iter) {
				if (/*cap[*e_iter] > 0 &&*/ cap[*e_iter] - re_cap[*e_iter] > 0) {

					std::cout << "Source: " << vertex_index_map[out_node_pt] << " destination: " << boost::target(*e_iter, modified_time_expanded_graph) << " capacity: " << cap[*e_iter] << "residual cap: " << re_cap[*e_iter] << " used capacity: "
						<< (cap[*e_iter] - re_cap[*e_iter]) << std::endl;

					Graph_Vertex_t target_node = boost::target(*e_iter, modified_time_expanded_graph);
					VertexProperties target_node_property = modified_time_expanded_graph[target_node];
					if (target_node_property.is_in_node) { // same location for next t
						path->at(agt).push_back(location);
						out_node_pt = new VertexProperties{ location, -1, false, true, false, t + 1 };
						break;
					}
					else if (target_node_property.is_u_node) {
						location = (target_node_property.loc1 == location) ? target_node_property.loc2 : location;
						path->at(agt).push_back(location);
						out_node_pt = new VertexProperties{ location, -1, false, true, false, t + 1 };
						break;
					}
				}
			}
		}
	}

	//printPaths();
	modified_time_expanded_graph.clear();
	return true;
}

vector<int> SingleTypeSearch::getDistancesToAllLocations(int start) {
	std::unordered_set<int> visited_locations;
	std::queue<int> queue;
	queue.push(start);
	vector<int> distances(my_map.size());
	distances[start] = 0;
	while (!queue.empty()) {
		int current_location = queue.front();
		queue.pop();
		for (int direction = 0; direction < 5; direction++) {
			int location = current_location + actions_offset[direction];
			if (isDirectionAllowed(current_location, direction) && !my_map[location]) {
				if (visited_locations.find(location) == visited_locations.end()) {
					visited_locations.insert(location);
					queue.push(location);
					// This vertex has not been visited, set initial distance
					if (location != start) {
						distances[location] = distances[current_location] + 1;
					}
				}
				else {
					// Update distance if shorter
					if (distances[location] > distances[current_location] + 1) {
						distances[location] = distances[current_location] + 1;
					}
				}
			}
		}
	}
	return distances;
}

void SingleTypeSearch::printPaths() {
	for (size_t i = 0; i < path->size(); i++) {
		cout << "AGENT " << i << " Path: ";
		for (vector<int>::const_iterator it = path->at(i).begin(); it != path->at(i).end(); ++it)
			std::cout << *it << '(' << *it % cols << ',' << *it / cols << ')' << ' ';
		cout << endl;
	}
}

/*
void SingleTypeSearch::updateReachableSets(int timesteps) {
	for (int t = current_timestep + 1; t <= timesteps; t++) {
		vector<bool> reachables(map_size, false);
		for (int a1 = 0; a1 < num_agents; a1++) {
			for (int a2 = 0; a2 < num_agents; a2++) {
				vector<int> distFromStart = getDistancesToAllLocations(start_locations[a1]);
				vector<int> distFromGoal = getDistancesToAllLocations(goal_locations[a1]);
				// Assign reachability values
				for (int loc = 0; loc < map_size; loc++) {
					if (distFromStart[loc] <= t	&& distFromGoal[loc] <= timesteps - t) {
						reachables[loc] = true;
					}
				}

			}
		}
		reachable_locations.push_back(reachables);
	}

}
*/

int SingleTypeSearch::my_max_flow(Graph_t & g, int s, int t, Weight & w, Capacity & c, Reversed & rev, ResidualCapacity & residual_capacity) {
	
	boost::filtered_graph<const Graph_t, boost::is_residual_edge<ResidualCapacity> >  gres(g, residual_capacity);	
	std::vector<Graph_Vertex_t> parents(boost::num_vertices(g));
	std::vector<int> distances(boost::num_vertices(g));

	while (true) {
		boost::dijkstra_shortest_paths(gres, s,
			boost::predecessor_map(boost::make_iterator_property_map(parents.begin(), get(boost::vertex_index, g))).
			distance_map(boost::make_iterator_property_map(distances.begin(), get(boost::vertex_index, g))));

	}
}

SingleTypeSearch::~SingleTypeSearch() {
	//  delete[] this->my_heuristic; (created once and should be deleted from outside)
	time_expanded_graph.clear();
	delete[] my_heuristic;
}
