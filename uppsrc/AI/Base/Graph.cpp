#include "Graph.h"
#include <float.h>

namespace Upp {

void Graph::Clear()
{
	node_names.Clear();
	name_index.Clear();
	adj.Clear();
}

int Graph::AddNode(const String& name)
{
	int idx = name_index.Find(name);
	if(idx >= 0) return idx;

	idx = node_names.GetCount();
	node_names.Add(name);
	name_index.Add(name);
	adj.Add();
	return idx;
}

int Graph::FindNode(const String& name) const
{
	return name_index.Find(name);
}

String Graph::GetNodeName(int id) const
{
	if(id >= 0 && id < node_names.GetCount())
		return node_names[id];
	return String();
}

void Graph::AddEdge(int from, int to, double weight, bool bidirectional)
{
	ASSERT(from >= 0 && from < node_names.GetCount());
	ASSERT(to >= 0 && to < node_names.GetCount());

	adj[from].Add(GraphEdge(to, weight));
	if(bidirectional) {
		adj[to].Add(GraphEdge(from, weight));
	}
}

void Graph::AddEdge(const String& from, const String& to, double weight, bool bidirectional)
{
	int u = AddNode(from);
	int v = AddNode(to);
	AddEdge(u, v, weight, bidirectional);
}

const Vector<GraphEdge>& Graph::GetEdges(int node) const
{
	ASSERT(node >= 0 && node < adj.GetCount());
	return adj[node];
}

Vector<int> Graph::GetNeighbors(int node) const
{
	Vector<int> n;
	if(node >= 0 && node < adj.GetCount()) {
		for(const auto& e : adj[node]) {
			n.Add(e.target);
		}
	}
	return n;
}

bool Graph::HasCycle() const
{
	int n = node_names.GetCount();
	Vector<int> state(n, 0); // 0=unvisited, 1=visiting, 2=visited

	auto dfs = [&](auto& self, int u) -> bool {
		state[u] = 1;
		for(const auto& edge : adj[u]) {
			int v = edge.target;
			if(state[v] == 1) return true;
			if(state[v] == 0 && self(self, v)) return true;
		}
		state[u] = 2;
		return false;
	};

	for(int i = 0; i < n; ++i) {
		if(state[i] == 0) {
			if(dfs(dfs, i)) return true;
		}
	}
	return false;
}

Vector<int> Graph::TopologicalSort() const
{
	Vector<int> result;
	int n = node_names.GetCount();
	Vector<int> in_degree(n, 0);

	for(int u = 0; u < n; ++u) {
		for(const auto& edge : adj[u]) {
			in_degree[edge.target]++;
		}
	}

	Vector<int> queue;
	for(int i = 0; i < n; ++i) {
		if(in_degree[i] == 0) {
			queue.Add(i);
		}
	}

	int head = 0;
	while(head < queue.GetCount()) {
		int u = queue[head++];
		result.Add(u);
		for(const auto& edge : adj[u]) {
			int v = edge.target;
			if(--in_degree[v] == 0) {
				queue.Add(v);
			}
		}
	}

	if(result.GetCount() != n) {
		result.Clear(); // Cycle detected
	}
	return result;
}

Vector<int> Graph::ShortestPathDijkstra(int start, int target, double* out_cost) const
{
	Vector<int> path;
	int n = node_names.GetCount();
	if(start < 0 || start >= n || target < 0 || target >= n) return path;

	Vector<double> dist(n, DBL_MAX);
	Vector<int> prev(n, -1);
	Vector<bool> visited(n, false);

	dist[start] = 0.0;

	for(int i = 0; i < n; ++i) {
		int u = -1;
		double best_d = DBL_MAX;
		for(int j = 0; j < n; ++j) {
			if(!visited[j] && dist[j] < best_d) {
				best_d = dist[j];
				u = j;
			}
		}

		if(u < 0 || dist[u] == DBL_MAX) break;
		if(u == target) break;

		visited[u] = true;
		for(const auto& edge : adj[u]) {
			int v = edge.target;
			double alt = dist[u] + edge.weight;
			if(alt < dist[v]) {
				dist[v] = alt;
				prev[v] = u;
			}
		}
	}

	if(dist[target] == DBL_MAX) return path;

	if(out_cost) *out_cost = dist[target];

	for(int at = target; at != -1; at = prev[at]) {
		path.Add(at);
	}
	// Reverse path to start -> target
	for(int i = 0, j = path.GetCount() - 1; i < j; ++i, --j) {
		int tmp = path[i];
		path[i] = path[j];
		path[j] = tmp;
	}
	return path;
}

}
