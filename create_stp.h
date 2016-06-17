// Generate partial order graph

#ifndef CREATESTP_H
#define CREATESTP_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/property_map/property_map.hpp>
// #include <boost/function.hpp>
// #include <boost/make_shared.hpp>
#include <sparsehash/dense_hash_map>
#include <iostream>
#include <vector>
#include <utility>
#include <map>
#include <cstdint>
#include <memory>
#include <functional>
#include <string>

#define EPS 0.000000001

using std::map;
using std::vector;
using std::pair;
using std::shared_ptr;
using std::function;
using google::dense_hash_map;
using std::tr1::hash;

class CreateSTP {
public:
	// Create a struct to hold properties for each vertex
	struct VertexProperties {
		// represent agent_id's:
		// loc1 (if loc2=-1 and markers are false)
		// if isMarker==true, the vertex represents delta(loc1,loc2)
		int agent_id;
		int loc1;
		int loc2;
		bool isMarker;
		int timestep;
	};

	// Create a struct to hold properties for each edge
	struct EdgeProperties {
		double lb = 0;  // lower bound
		double ub = 0;  // upper bound
	};


	struct STPNodeHasher {
		std::size_t operator()(const VertexProperties* n) const {
			size_t agent_id_hash = std::hash<int>()(n->agent_id);
			size_t time_generated_hash = std::hash<int>()(n->timestep);
			return (agent_id_hash ^ (time_generated_hash << 1));
		}
	};

	struct stp_eqnode {
		bool operator()(const VertexProperties* s1, const VertexProperties* s2) const {
			return (s1 == s2) || (s1 && s2 &&
				s1->agent_id == s2->agent_id &&
				s1->loc1 == s2->loc1 &&
				s1->loc2 == s2->loc2 &&
				s1->isMarker == s2->isMarker &&
				s1->timestep == s2->timestep);
		}
	};


	typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, VertexProperties, EdgeProperties> Graph_t;
	typedef boost::graph_traits<Graph_t>::vertex_descriptor Graph_Vertex_t;
	typedef boost::graph_traits<Graph_t>::edge_descriptor Graph_Edge_t;
	typedef boost::graph_traits<Graph_t>::vertex_iterator vertex_iter_t;
	typedef boost::graph_traits<Graph_t>::edge_iterator edge_iter_t;
	//  typedef boost::adjacency_list<boost::vecS, boost::hash_setS, boost::directedS, VertexDesc> my_digraph;
	typedef dense_hash_map<VertexProperties*, Graph_Vertex_t, STPNodeHasher, stp_eqnode> hashtable_t;


	size_t max_path_length;
	size_t num_of_agents;

	Graph_t stp_graph;
	Graph_Vertex_t X0, XF;

	hashtable_t stpNodes;  // map VertexProperties to their Vertex_Descriptor

	double lambda;
	double delta;
	double maxV;

	explicit CreateSTP(const vector< vector<int>* >& paths, double delta = 0, double maxV = 1);
	void addTemporalConstraints(const vector< vector<int>* >& paths);
	inline void setVertexProperties(VertexProperties* v, int agent_id, int loc1, int loc2, bool isMarker, int timestep);
	inline std::string getVarName(VertexProperties* var);
	inline std::string getEdgeName(Graph_Edge_t e_desc);
	void exampleLP();
	void solveLP();
	~CreateSTP();
	//  void exportSTP(string filename);

	// inner class that is used to "pretty-print" to DOT file (VERTEX) ///////////////////////////////////
	template <class GraphType> class myVertexWriter {
	public:
		explicit myVertexWriter(GraphType _graphType) : graphType(_graphType) {}  // used to print a Graph_t to dot format
		template <class VertexOrEdge> void operator()(std::ostream &out, const VertexOrEdge &v) const {
			out << "[agent_id=\"" << graphType[v].agent_id << "\", " <<
				"loc1=\"" << graphType[v].loc1 << "\", " <<
				"loc2=\"" << graphType[v].loc2 << "\", " <<
				"isMarker=\"" << graphType[v].isMarker << "\", " <<
				"label=\"AG:" << graphType[v].agent_id << ", L1:" << graphType[v].loc1 <<
				", L2:" << graphType[v].loc2 << ", T:" << graphType[v].timestep <<
				"\"]";  // for dotty
		}
	private:
		GraphType graphType;
	};  // end of inner class ///////////////////////////////////////////////////////////////////////
	// inner class that is used to "pretty-print" to DOT file (EDGE) ///////////////////////////////////
	template <class GraphType> class myEdgeWriter {
	public:
		explicit myEdgeWriter(GraphType _graphType) : graphType(_graphType) {}  // used to print a Graph_t to dot format
		template <class VertexOrEdge> void operator()(std::ostream &out, const VertexOrEdge &e) const {
			out << "[label=\"[" << graphType[e].lb << "," << graphType[e].ub << "]\"]";
		}
	private:
		GraphType graphType;
	};  // end of inner class ///////////////////////////////////////////////////////////////////////

};

#endif
