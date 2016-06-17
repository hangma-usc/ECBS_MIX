//          Copyright W.P. McNeill 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)


// This program uses the A-star search algorithm in the Boost Graph Library to
// solve a maze.  It is an example of how to apply Boost Graph Library
// algorithms to implicit graphs.
//
// This program generates a random maze and then tries to find the shortest
// path from the lower left-hand corner to the upper right-hand corner.  Mazes
// are represented by two-dimensional grids where a cell in the grid may
// contain a barrier.  You may move up, down, right, or left to any adjacent
// cell that does not contain a barrier.
//
// Once a maze solution has been attempted, the maze is printed.  If a
// solution was found it will be shown in the maze printout and its length
// will be returned.  Note that not all mazes have solutions.
//
// The default maze size is 20x10, though different dimensions may be
// specified on the command line.


#include <boost/array.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/labeled_graph.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <ctime>
#include <iostream>

#include "map_loader.h"


#define GRID_RANK 2
typedef boost::labeled_graph<boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, std::pair<int,int> >, int> grid;
typedef boost::graph_traits<grid>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<grid>::vertices_size_type vertices_size_type;

// A searchable maze
//
// The maze is grid of locations which can either be empty or contain a
// barrier.  You can move to an adjacent location in the grid by going up,
// down, left and right.  Moving onto a barrier is not allowed.  The maze can
// be solved by finding a path from the lower-left-hand corner to the
// upper-right-hand corner.  If no open path exists between these two
// locations, the maze is unsolvable.
//
// The maze is implemented as a filtered grid graph where locations are
// vertices.  Barrier vertices are filtered out of the graph.
//
// A-star search is used to find a path through the maze. Each edge has a
// weight of one, so the total path length is equal to the number of edges
// traversed.
class maze {
public:
	friend std::ostream& operator<<(std::ostream&, const maze&);

	// The grid underlying the maze
	grid m_grid;
	int start_location;
	int goal_location;

	maze(const MapLoader & ml, int x, int y) {
		start_location = x;
		goal_location = y;

		grid my_grid;

		for (int i = 0; i < ml.rows; i++) {
			for (int j = 0; j < ml.cols; j++) {
				if (!ml.my_map[ml.cols*i + j]) {
					vertex_descriptor u = boost::add_vertex(ml.cols*i + j, my_grid);
					my_grid[u] = pair<int,int>(i, j);
					if (i > 0 && !ml.my_map[ml.cols*(i - 1) + j]) {
						boost::add_edge(ml.cols*(i - 1) + j, ml.cols*i + j, my_grid);
					}
					if (j > 0 && !ml.my_map[ml.cols*i + j - 1]) {
						boost::add_edge(ml.cols*i + j - 1, ml.cols*i + j, my_grid);
					}
				}
			}
		}
	};

	bool solve();
	bool solved() const { return !(m_solution_length == 0); }

	// The length of the solution path
	int m_solution_length;
};


// Euclidean heuristic for a grid
//
// This calculates the Euclidean distance between a vertex and a goal
// vertex.
class euclidean_heuristic :
	public boost::astar_heuristic<grid, double>
{
public:
	euclidean_heuristic(vertex_descriptor goal) :m_goal(goal) {};

	double operator()(vertex_descriptor v) {
		return sqrt(pow(double(m_goal[0] - v[0]), 2) + pow(double(m_goal[1] - v[1]), 2));
	}

private:
	vertex_descriptor m_goal;
};

// Exception thrown when the goal vertex is found
struct found_goal {};

// Visitor that terminates when we find the goal vertex
struct astar_goal_visitor :public boost::default_astar_visitor {
	astar_goal_visitor(vertex_descriptor goal) :m_goal(goal) {};

	void examine_vertex(vertex_descriptor u, const grid&) {
		if (u == m_goal)
			throw found_goal();
	}

private:
	vertex_descriptor m_goal;
};

// Solve the maze using A-star search.  Return true if a solution was found.
bool maze::solve() {
	boost::static_property_map<int> weight(1);
	// The predecessor map is a vertex-to-vertex mapping.
	typedef boost::unordered_map<vertex_descriptor,
		vertex_descriptor,
		vertex_hash> pred_map;
	pred_map predecessor;
	boost::associative_property_map<pred_map> pred_pmap(predecessor);
	// The distance map is a vertex-to-distance mapping.
	typedef boost::unordered_map<vertex_descriptor,
		int,
		vertex_hash> dist_map;
	dist_map distance;
	boost::associative_property_map<dist_map> dist_pmap(distance);

	vertex_descriptor s = source();
	vertex_descriptor g = boost::get(goal_location, m_grid.graph);
	euclidean_heuristic heuristic(g, );
	astar_goal_visitor visitor(g);

	try {
		boost::astar_search(m_grid, s, heuristic,
			boost::weight_map(weight).
			predecessor_map(pred_pmap).
			distance_map(dist_pmap).
			visitor(visitor));
	}
	catch (found_goal fg) {
		// Walk backwards from the goal through the predecessor chain adding
		// vertices to the solution path.
		/*
		for (vertex_descriptor u = g; u != s; u = predecessor[u])
			m_solution.insert(u);
		m_solution.insert(s);
		*/
		m_solution_length = distance[g];
		return true;
	}

	return false;
}


// Generate a maze with barriers.
maze generate_grid(const MapLoader & ml, int start, int goal) {
	maze m(ml.rows, ml.cols);
	m.start_location = start;
	m.goal_location = goal;

	for (int i = 0; i < ml.rows; i++) {
		for (int j = 0; j < ml.cols; j++) {
			if (ml.my_map[ml.cols*i + j]) {
				vertex_descriptor u = vertex(ml.cols*i + j, m.m_grid);
				boost::remove_vertex(u, m.m_grid.);
			}
		}
	}

	return m;
}
