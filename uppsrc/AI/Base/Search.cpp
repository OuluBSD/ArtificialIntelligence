#include "Search.h"

namespace Upp {

Vector<GridPoint> GridAStar::FindPath(const GridMap& map, const GridPoint& start, const GridPoint& goal, bool diagonal, double* out_cost)
{
	Vector<GridPoint> path;
	if(map.IsBlocked(start.x, start.y) || map.IsBlocked(goal.x, goal.y)) {
		return path;
	}

	struct Node {
		GridPoint pt;
		double g = DBL_MAX;
		double f = DBL_MAX;
		GridPoint parent;
		bool in_open = false;
		bool in_closed = false;
	};

	VectorMap<GridPoint, Node> all_nodes;

	auto get_node = [&](const GridPoint& p) -> Node& {
		int idx = all_nodes.Find(p);
		if(idx < 0) {
			Node n;
			n.pt = p;
			all_nodes.Add(p, n);
			return all_nodes.Top();
		}
		return all_nodes[idx];
	};

	Node& start_node = get_node(start);
	start_node.g = 0.0;
	start_node.f = diagonal ? GridMap::EuclideanDistance(start, goal) : GridMap::ManhattanDistance(start, goal);
	start_node.in_open = true;

	Vector<GridPoint> open_set;
	open_set.Add(start);

	while(!open_set.IsEmpty()) {
		// Find lowest f in open_set
		int best_idx = 0;
		double best_f = all_nodes.Get(open_set[0]).f;
		for(int i = 1; i < open_set.GetCount(); ++i) {
			double f = all_nodes.Get(open_set[i]).f;
			if(f < best_f) {
				best_f = f;
				best_idx = i;
			}
		}

		GridPoint current_pt = open_set[best_idx];
		open_set.Remove(best_idx);

		Node& current_node = all_nodes.Get(current_pt);
		current_node.in_open = false;
		current_node.in_closed = true;

		if(current_pt == goal) {
			// Reconstruct path
			if(out_cost) *out_cost = current_node.g;
			GridPoint curr = goal;
			while(curr != start) {
				path.Add(curr);
				curr = all_nodes.Get(curr).parent;
			}
			path.Add(start);
			// Reverse
			for(int i = 0, j = path.GetCount() - 1; i < j; ++i, --j) {
				GridPoint tmp = path[i];
				path[i] = path[j];
				path[j] = tmp;
			}
			return path;
		}

		Vector<GridPoint> neighbors = map.GetNeighbors(current_pt, diagonal);
		for(const auto& next_pt : neighbors) {
			Node& next_node = get_node(next_pt);
			if(next_node.in_closed) continue;

			double step_cost = (current_pt.x != next_pt.x && current_pt.y != next_pt.y) ? 1.41421356 : 1.0;
			double tentative_g = current_node.g + step_cost;

			if(tentative_g < next_node.g) {
				next_node.parent = current_pt;
				next_node.g = tentative_g;
				double h = diagonal ? GridMap::EuclideanDistance(next_pt, goal) : GridMap::ManhattanDistance(next_pt, goal);
				next_node.f = tentative_g + h;

				if(!next_node.in_open) {
					next_node.in_open = true;
					open_set.Add(next_pt);
				}
			}
		}
	}

	return path; // No path found
}

}
