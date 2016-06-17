#include "create_stp.h"
#include <vector>
#include <utility>
#include <cfloat>
#include <string>
#include "gurobi_c++.h"

using std::make_pair;
using std::cout;
using std::endl;
using std::to_string;
using std::pair;


size_t getPathsMaxLength(const vector< vector<int>* >& paths) {
  size_t retVal = 0;
  int num_of_agents = paths.size();
  for (int ag = 0; ag < num_of_agents; ag++)
    if ( paths[ag] != NULL && paths[ag]->size() > retVal )
      retVal = paths[ag]->size();
  return retVal;
}  ////////////////////////////////////////////////////////////////////////////////////////////////


inline int getAgentLocation(int agent_id, size_t timestep, const vector < vector<int>* >& paths) {
  // if last timestep > plan length, agent remains in its last location
  if (timestep >= paths[agent_id]->size())
    return paths[agent_id]->at(paths[agent_id]->size()-1);
  // otherwise, return its location for that timestep
  return paths[agent_id]->at(timestep);
}  ////////////////////////////////////////////////////////////////////////////////////////////////


// void CreateSTP::addSTPEdges() {
  /*
  //  action_sets_graphs.resize(action_sets.size());
  for (vector< vector< pair<int, int> > >::const_iterator action_set_it = action_sets.begin();
       action_set_it != action_sets.end();
       ++action_set_it) {
    Graph_t as_graph;
    // create action_set graph's nodes (for timestep given by the iterator above).
    for (vector< pair<int, int> >::const_iterator ag_move_it = action_set_it->begin();
         ag_move_it != action_set_it->end();
         ++ag_move_it) {
      Graph_Vertex_t v = boost::add_vertex(as_graph);
      as_graph[v].agent_id = std::distance(action_set_it->begin(), ag_move_it);  // (note -- constant time for RandomAccessIterator)
      as_graph[v].loc1 = ag_move_it->first;
      as_graph[v].loc2 = ag_move_it->second;
    }
    // create edges (connect two nodes if their loc1/loc2 intersect).
    boost::graph_traits<Graph_t>::vertex_iterator vi, vj, v_end;
    for (boost::tie(vi, v_end) = boost::vertices(as_graph); vi != v_end; ++vi) {
      vj = vi+1;
      if (vj != v_end)
        if (as_graph[*vi].loc1 == as_graph[*vj].loc1 ||
            as_graph[*vi].loc1 == as_graph[*vj].loc2 ||
            as_graph[*vi].loc2 == as_graph[*vj].loc1 ||
            as_graph[*vi].loc2 == as_graph[*vj].loc2)
          boost::add_edge(*vi, *vj, as_graph);
    }
    action_sets_graphs.push_back(as_graph);
  }
  */
//}  ////////////////////////////////////////////////////////////////////////////////////////////////

void CreateSTP::addTemporalConstraints(const vector< vector<int>* >& paths) {
  for (size_t ag1 = 0; ag1 < num_of_agents; ag1++) {
    for (size_t t1 = 0; t1 < paths[ag1]->size()-1; t1++) {
      if ( getAgentLocation(ag1, t1, paths) != getAgentLocation(ag1, t1+1, paths) ) {  // skip wait
        for (size_t ag2 = 0; ag2 < num_of_agents; ag2++) {
          if ( ag1 != ag2 ) {
            for (size_t t2 = t1; t2 < paths[ag2]->size(); t2++) {
              if ( getAgentLocation(ag2, t2, paths) != getAgentLocation(ag2, t2+1, paths) ||  // skip wait
                   t2 == paths[ag2]->size()-1 ) {  // unless its the goal location
                if ( getAgentLocation(ag1, t1, paths) == getAgentLocation(ag2, t2, paths) ) {
                  // find marker right after <ag1,t1> and right before <ag2,t2>
                  // cout << "A1:" << ag1 << ",LOC1:" << paths[ag1]->at(t1) << ",LOC2:" << paths[ag1]->at(t1+1) <<
                  //    "T:" << t1 << " ; A2:" << ag2 << ",LOC1:" << paths[ag2]->at(t2) <<
                  //    ",LOC2:" << paths[ag2]->at(t2-1) << "T:" << t2-1 << endl;
                  VertexProperties* ag1_v = new VertexProperties();
                  setVertexProperties(ag1_v, ag1, paths[ag1]->at(t1), paths[ag1]->at(t1+1), true, t1);
                  VertexProperties* ag2_v = new VertexProperties();
                  setVertexProperties(ag2_v, ag2, paths[ag2]->at(t2), paths[ag2]->at(t2-1), true, t2-1);
                  Graph_Vertex_t ag1_v_desc = stpNodes[ag1_v];
                  Graph_Vertex_t ag2_v_desc = stpNodes[ag2_v];
                  Graph_Edge_t e_desc = (boost::add_edge(ag1_v_desc, ag2_v_desc, stp_graph)).first;
                  stp_graph[e_desc].lb = EPS;
                  stp_graph[e_desc].ub = DBL_MAX;
                }
              }
            }
          }
        }
      }
    }
  }
}  // ----------------------------------------------------------------------------------------------------

inline void CreateSTP::setVertexProperties(VertexProperties* v, int agent_id, int loc1, int loc2, bool isMarker, int timestep) {
  v->agent_id = agent_id;
  v->loc1 = loc1;
  v->loc2 = loc2;
  v->isMarker = isMarker;
  v->timestep = timestep;
}


CreateSTP::CreateSTP(const vector < vector<int>* >& paths, double delta, double maxV) :
  delta(delta), maxV(maxV) {
  max_path_length = getPathsMaxLength(paths);
  num_of_agents = paths.size();

  // init hash table
  VertexProperties* empty_node = new VertexProperties();
  setVertexProperties(empty_node, -1, -1, -1, false, -1);
  VertexProperties* deleted_node = new VertexProperties();
  setVertexProperties(deleted_node, -2, -2, -2, false, -2);
  stpNodes.set_empty_key(empty_node);
  stpNodes.set_deleted_key(deleted_node);

  // generate X_0 and X_F
  VertexProperties* X0_p = new VertexProperties();
  setVertexProperties(X0_p, -3, -3, -3, false, -3);
  X0 = boost::add_vertex(stp_graph);
  stp_graph[X0] = *X0_p;
  stpNodes[X0_p] = X0;
  VertexProperties* XF_p = new VertexProperties();
  setVertexProperties(XF_p, -4, -4, -4, false, -4);
  XF = boost::add_vertex(stp_graph);  // generate vertex descriptor in graph
  stp_graph[XF] = *XF_p;  // update bundeled properties
  stpNodes[XF_p] = XF;  // update hash table (properties to vertex descriptor)

  Graph_Edge_t e_desc;
  /*********************** DO NOT UNCOMMENT (will add wrong edge to the LP)
  // add edge X0-XF annotated with [0,lambda]
  e_desc = (boost::add_edge(X0, XF, stp_graph)).first;
  stp_graph[e_desc].lb = 0;
  stp_graph[e_desc].ub = -13;
  ************************/
  // generate (basic location and marker) nodes
  for (size_t ag = 0; ag < num_of_agents; ag++) {
    Graph_Vertex_t loc1, marker12, marker21;
    VertexProperties* vp;
    int loc1_id = -1;
    int loc2_id = -1;
    bool firstNode = true;  // used to add an edge from X0 to the first node (cannot simply use t==0 because of waits)
    for (size_t t = 0; t < paths[ag]->size()-1; t++) {
      loc1_id = getAgentLocation(ag, t, paths);
      loc2_id = getAgentLocation(ag, t+1, paths);
      if (loc1_id != loc2_id) {  // skip waits

        loc1 = boost::add_vertex(stp_graph);
        vp = new VertexProperties();
        setVertexProperties(vp, ag, loc1_id, -1, false, t);
        stp_graph[loc1] = *vp;
        stpNodes[vp] = loc1;

        if (firstNode == true) {  // add X0-loc edge
          e_desc = (boost::add_edge(X0, loc1, stp_graph)).first;
          stp_graph[e_desc].lb = 0;
          stp_graph[e_desc].ub = 0;
          firstNode = false;
        } else {  // add marker21->loc1 for all agents that are not the first
          e_desc = (boost::add_edge(marker21, loc1, stp_graph)).first;
          stp_graph[e_desc].lb = delta / maxV;
          stp_graph[e_desc].ub = DBL_MAX;
        }

        marker12 = boost::add_vertex(stp_graph);
        vp = new VertexProperties();
        setVertexProperties(vp, ag, loc1_id, loc2_id, true, t);
        stp_graph[marker12] = *vp;
        stpNodes[vp] = marker12;
        // add edge between location and safety marker
        e_desc = (boost::add_edge(loc1, marker12, stp_graph)).first;
        stp_graph[e_desc].lb = delta / maxV;
        stp_graph[e_desc].ub = DBL_MAX;
	
        marker21 = boost::add_vertex(stp_graph);
        vp = new VertexProperties();
        setVertexProperties(vp, ag, loc2_id, loc1_id, true, t);
        stp_graph[marker21] = *vp;
        stpNodes[vp] = marker21;
        // add edge between the two safety markers (we assume that the cost of every edge is 1).
        e_desc = (boost::add_edge(marker12, marker21, stp_graph)).first;
        stp_graph[e_desc].lb = (1-2*delta) / maxV;
        stp_graph[e_desc].ub = DBL_MAX;
      }
    }
    // add goal location for the agent
    loc1 = boost::add_vertex(stp_graph);
    vp = new VertexProperties();
    setVertexProperties(vp, ag, loc2_id, -1, false, paths[ag]->size()-1);
    stp_graph[loc1] = *vp;
    stpNodes[vp] = loc1;
    e_desc = (boost::add_edge(marker21, loc1, stp_graph)).first;
    stp_graph[e_desc].lb = delta / maxV;
    stp_graph[e_desc].ub = DBL_MAX;
    // add edge to XF
    e_desc = (boost::add_edge(loc1, XF, stp_graph)).first;
    stp_graph[e_desc].lb = 0;
    stp_graph[e_desc].ub = DBL_MAX;
  }
  addTemporalConstraints(paths);
  // Generate DOT file for debug
  myVertexWriter<Graph_t> vw(stp_graph);  // instantiate the writer class
  myEdgeWriter<Graph_t> ew(stp_graph);  // instantiate the writer class
  boost::write_graphviz(std::cout, stp_graph, vw, ew);
  cout << endl;
  solveLP();
  
}  ////////////////////////////////////////////////////////////////////////////////////////////////

inline string CreateSTP::getVarName(VertexProperties* var) {
  return ("AG_" + to_string(var->agent_id) 
    + "_L1_" + to_string(var->loc1)
    + "_L2_" + to_string(var->loc2)
	  + "_T_"  + to_string(var->timestep));
}

inline string CreateSTP::getEdgeName(Graph_Edge_t e_desc) {
  Graph_Vertex_t v_source = boost::source(e_desc, stp_graph);
  Graph_Vertex_t v_target = boost::target(e_desc, stp_graph);
  return getVarName(&stp_graph[v_source]) + "--" + getVarName(&stp_graph[v_target]);
  //    + "_[" + to_string(stp_graph[e_desc].lb) + "," + to_string(stp_graph[e_desc].ub) + "]";
}


void CreateSTP::solveLP() {
  try {
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);
    // create variables
    hashtable_t::iterator it;  // it.first is VertexProperties*, it.second is Graph_Vertex_t
    // iterate over all nodes in the hash table (that is, all nodes in the graph)
    for (it=stpNodes.begin(); it != stpNodes.end(); it++) {
      // (lb, ub, 0.0 (added later through setObjective), GRB_CONTINUOUS
      //      string v_name = "AG_" + to_string(((*it).first)->agent_id) 
      model.addVar (0.0, DBL_MAX, 0.0, GRB_CONTINUOUS, getVarName((*it).first));
    }
    // Integrate new variables
    model.update();
    // Iterate through the edges in the graph
    edge_iter_t ei, ei_end;
    tie(ei, ei_end) = boost::edges(stp_graph);
    for (; ei != ei_end; ++ei) {
      Graph_Edge_t e_desc = *ei;
      Graph_Vertex_t v_source = boost::source(e_desc,stp_graph);
      Graph_Vertex_t v_target = boost::target(e_desc,stp_graph);
      GRBVar grb_v_source = model.getVarByName(getVarName(&stp_graph[v_source]));
      GRBVar grb_v_target = model.getVarByName(getVarName(&stp_graph[v_target]));
      model.addConstr(grb_v_target - grb_v_source >= stp_graph[e_desc].lb, getEdgeName(e_desc));
      //      cout << "EDGE between: " << getVarName(&stp_graph[v_source]) << " -- " << getVarName(&stp_graph[v_target])
      //	   << " ; WITH CONSTRAINT: [" << stp_graph[e_desc].lb << "," << stp_graph[e_desc].ub << "]" << endl;
    }
    // set objective
    GRBVar grb_X0 = model.getVarByName(getVarName(&stp_graph[X0]));
    GRBVar grb_XF = model.getVarByName(getVarName(&stp_graph[XF]));
    model.setObjective(grb_XF - grb_X0, GRB_MINIMIZE);
    // Optimize model
    model.optimize();
    // Print results
    //cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
    /* print the value of every node */
    /*
    for (it=stpNodes.begin(); it != stpNodes.end(); it++) {
      string v_name = getVarName((*it).first);
      GRBVar v_grb = model.getVarByName(v_name);
      double v_value = v_grb.get(GRB_DoubleAttr_X);
      cout << v_name << " = " << v_value << endl;
    }
    */
    
  } catch(GRBException e) {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  } catch(...) {
    cout << "Exception during optimization" << endl;
  }

}


void CreateSTP::exampleLP() {
try {
    GRBEnv env = GRBEnv();

    GRBModel model = GRBModel(env);

    // Create variables
    // (lb, ub, 0.0 (added later through setObjective), GRB_CONTINUOUS
    GRBVar x = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x");
    GRBVar y = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y");
    GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z");

    // Integrate new variables

    model.update();

    // Set objective: maximize x + y + 2 z

    model.setObjective(x + y + 2 * z, GRB_MAXIMIZE);

    // Add constraint: x + 2 y + 3 z <= 4

    model.addConstr(x + 2 * y + 3 * z <= 4, "c0");

    // Add constraint: x + y >= 1

    model.addConstr(x + y >= 1, "c1");

    // Optimize model

    model.optimize();

    cout << x.get(GRB_StringAttr_VarName) << " "
         << x.get(GRB_DoubleAttr_X) << endl;
    cout << y.get(GRB_StringAttr_VarName) << " "
         << y.get(GRB_DoubleAttr_X) << endl;
    cout << z.get(GRB_StringAttr_VarName) << " "
         << z.get(GRB_DoubleAttr_X) << endl;

    cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

  } catch(GRBException e) {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  } catch(...) {
    cout << "Exception during optimization" << endl;
  }
}

CreateSTP::~CreateSTP() {
}
