// Load's a 2D map.
// First line: ROWS COLS
// Second line and onward, "1" represent blocked cell (otherwise, open)

#ifndef MAPLOADER_H
#define MAPLOADER_H

#include <string>
#include <vector>

using namespace std;

class MapLoader {
public:
	vector<bool> my_map;
	int rows;
	int cols;

	int start_loc;
	int goal_loc;

	enum valid_actions_t { WAIT, NORTH, EAST, WEST, SOUTH, ACTIONS_COUNT }; // ACTIONS_COUNT is the enum's size
	int* actions_offset;

	MapLoader(std::string fname); // load map from file
	MapLoader(int rows, int cols); // initialize new [rows x cols] empty map
	inline bool is_blocked(int row, int col) { return my_map[row * this->cols + col]; }
	void printMap();
	void printMap(char* mapChar);
	void printHeuristic(const double* mapH, const int agent_id);
	char* mapToChar();
	vector<bool> get_map() const; // return a deep-copy of my_map
	inline int linearize_coordinate(int row, int col) const { return (this->cols * row + col); }
	inline int row_coordinate(int id) const { return id / this->cols; }
	inline int col_coordinate(int id) const { return id % this->cols; }
	void printPath(std::vector<int> path);
	void saveToFile(std::string fname);
	valid_actions_t get_action(int id1, int id2) const;

	~MapLoader();
};

#endif
