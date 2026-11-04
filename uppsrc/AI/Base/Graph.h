#ifndef _AI_Base_Graph_h_
#define _AI_Base_Graph_h_

#include <Core/Core.h>

namespace Upp {

struct GraphEdge : Moveable<GraphEdge> {
	int target = -1;
	double weight = 1.0;

	GraphEdge() {}
	GraphEdge(int target, double weight = 1.0) : target(target), weight(weight) {}
};

class Graph {
private:
	Vector<String> node_names;
	Index<String> name_index;
	Vector<Vector<GraphEdge>> adj;

public:
	Graph() {}

	void Clear();
	int AddNode(const String& name);
	int FindNode(const String& name) const;
	int GetNodeCount() const { return node_names.GetCount(); }
	String GetNodeName(int id) const;

	void AddEdge(int from, int to, double weight = 1.0, bool bidirectional = false);
	void AddEdge(const String& from, const String& to, double weight = 1.0, bool bidirectional = false);

	const Vector<GraphEdge>& GetEdges(int node) const;
	Vector<int> GetNeighbors(int node) const;

	bool HasCycle() const;
	Vector<int> TopologicalSort() const;
	Vector<int> ShortestPathDijkstra(int start, int target, double* out_cost = nullptr) const;
};

}

#endif
