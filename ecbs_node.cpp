#include "ecbs_node.h"
#include <vector>


ECBSNode::ECBSNode() {
	type_id = -1;
	constraint = make_tuple(-1, -1, -1);
	path = vector<vector<int>>();
	g_val = 0;
	h_val = 0;
	time_expanded = -1;
	path_cost = -1;
}

ECBSNode::ECBSNode(int type_id, ECBSNode* parent, double g_val, double h_val, int time_expanded, double path_cost) :
	type_id(type_id), parent(parent), g_val(g_val), h_val(h_val), time_expanded(time_expanded), path_cost(path_cost) {
	constraint = make_tuple(-1, -1, -1);
	path = vector<vector<int>>();
}


bool ECBSNode::isEqual(const ECBSNode* n1, const ECBSNode* n2) {
	return (n1->parent == n2->parent &&
		n1->type_id == n2->type_id &&
		n1->constraint == n2->constraint);
}


ECBSNode::~ECBSNode() {
}


std::ostream& operator<<(std::ostream& os, const ECBSNode& n) {
	os << "THIS NODE HAS: g_val=" << n.g_val << " and h_val=" << n.h_val << ". It constrains type " << n.type_id <<
		" on loc1[" << get<0>(n.constraint) << "] loc2[" << get<1>(n.constraint) << "] at time[" << get<2>(n.constraint) << "]" << endl;
	return os;
}
