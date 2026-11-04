#ifndef _AI_Base_Search_h_
#define _AI_Base_Search_h_

#include <Core/Core.h>
#include <float.h>

namespace Upp {

// Generic Point2D representation for 2D grid pathfinding
struct GridPoint : Moveable<GridPoint> {
	int x = 0;
	int y = 0;

	GridPoint() {}
	GridPoint(int x, int y) : x(x), y(y) {}

	bool operator==(const GridPoint& o) const { return x == o.x && y == o.y; }
	bool operator!=(const GridPoint& o) const { return !(*this == o); }
	hash_t GetHashValue() const { return CombineHash(x, y); }
	String ToString() const { return Format("(%d, %d)", x, y); }
};

class GridMap {
public:
	int width = 0;
	int height = 0;
	Vector<bool> obstacles; // true = blocked

	GridMap(int w = 0, int h = 0) : width(w), height(h), obstacles(w * h, false) {}

	void SetSize(int w, int h) {
		width = w;
		height = h;
		obstacles.SetCount(w * h, false);
	}

	bool IsValid(int x, int y) const {
		return x >= 0 && x < width && y >= 0 && y < height;
	}

	bool IsBlocked(int x, int y) const {
		if(!IsValid(x, y)) return true;
		return obstacles[y * width + x];
	}

	void SetBlocked(int x, int y, bool blocked = true) {
		if(IsValid(x, y)) obstacles[y * width + x] = blocked;
	}

	Vector<GridPoint> GetNeighbors(const GridPoint& p, bool diagonal = false) const {
		Vector<GridPoint> res;
		static const int dx[] = { 0, 1, 0, -1, 1, 1, -1, -1 };
		static const int dy[] = { -1, 0, 1, 0, -1, 1, 1, -1 };
		int count = diagonal ? 8 : 4;
		for(int i = 0; i < count; ++i) {
			int nx = p.x + dx[i];
			int ny = p.y + dy[i];
			if(!IsBlocked(nx, ny)) {
				res.Add(GridPoint(nx, ny));
			}
		}
		return res;
	}

	static double ManhattanDistance(const GridPoint& a, const GridPoint& b) {
		return abs(a.x - b.x) + abs(a.y - b.y);
	}

	static double EuclideanDistance(const GridPoint& a, const GridPoint& b) {
		int dx = a.x - b.x;
		int dy = a.y - b.y;
		return sqrt((double)(dx * dx + dy * dy));
	}
};

class GridAStar {
public:
	static Vector<GridPoint> FindPath(const GridMap& map, const GridPoint& start, const GridPoint& goal, bool diagonal = false, double* out_cost = nullptr);
};

// Game Tree MiniMax & Alpha-Beta Pruning
template <class TGameState>
class MiniMaxPlanner {
public:
	using EvaluateFn = Function<double(const TGameState& state)>;
	using GetMovesFn = Function<Vector<TGameState>(const TGameState& state, bool is_maximizing)>;
	using IsTerminalFn = Function<bool(const TGameState& state)>;

	static double AlphaBeta(const TGameState& state, int depth, double alpha, double beta, bool is_maximizing,
	                        EvaluateFn eval, GetMovesFn get_moves, IsTerminalFn is_terminal,
	                        TGameState* best_move = nullptr)
	{
		if(depth == 0 || is_terminal(state)) {
			return eval(state);
		}

		Vector<TGameState> moves = get_moves(state, is_maximizing);
		if(moves.IsEmpty()) return eval(state);

		if(is_maximizing) {
			double max_eval = -DBL_MAX;
			int best_idx = 0;
			for(int i = 0; i < moves.GetCount(); ++i) {
				double evaluation = AlphaBeta(moves[i], depth - 1, alpha, beta, false, eval, get_moves, is_terminal);
				if(evaluation > max_eval) {
					max_eval = evaluation;
					best_idx = i;
				}
				alpha = max(alpha, evaluation);
				if(beta <= alpha) break; // Beta cutoff
			}
			if(best_move) *best_move = moves[best_idx];
			return max_eval;
		}
		else {
			double min_eval = DBL_MAX;
			int best_idx = 0;
			for(int i = 0; i < moves.GetCount(); ++i) {
				double evaluation = AlphaBeta(moves[i], depth - 1, alpha, beta, true, eval, get_moves, is_terminal);
				if(evaluation < min_eval) {
					min_eval = evaluation;
					best_idx = i;
				}
				beta = min(beta, evaluation);
				if(beta <= alpha) break; // Alpha cutoff
			}
			if(best_move) *best_move = moves[best_idx];
			return min_eval;
		}
	}
};

}

#endif
