#include "ActionPlanner.h"
#include <float.h>

namespace Upp {

String WorldState::ToString() const
{
	String s;
	for(int i = 0; i < values.GetCount(); ++i) {
		if(i > 0) s << ", ";
		s << values.GetKey(i) << "=" << AsString(values[i]);
	}
	return s;
}

PlannerAction& ActionPlanner::AddAction(const String& name, double cost)
{
	PlannerAction& a = actions.Add();
	a.name = name;
	a.cost = cost;
	return a;
}

PlannerAction& ActionPlanner::AddAction(const PlannerAction& act)
{
	return actions.Add(act);
}

Vector<PlannerAction> ActionPlanner::Plan(const WorldState& initial, const WorldState& goal, double* out_total_cost)
{
	Vector<PlannerAction> plan;

	if(initial.MeetsConditions(goal)) {
		if(out_total_cost) *out_total_cost = 0.0;
		return plan;
	}

	struct PlanNode : Moveable<PlanNode> {
		WorldState state;
		int action_idx = -1; // action that led to this state
		int parent_idx = -1;
		double g = DBL_MAX;
		double f = DBL_MAX;
		bool in_closed = false;

		PlanNode() {}
		PlanNode(const PlanNode& n) : state(n.state), action_idx(n.action_idx), parent_idx(n.parent_idx), g(n.g), f(n.f), in_closed(n.in_closed) {}
		PlanNode(PlanNode&& n) : state(pick(n.state)), action_idx(n.action_idx), parent_idx(n.parent_idx), g(n.g), f(n.f), in_closed(n.in_closed) {}
		PlanNode& operator=(const PlanNode& n) {
			if(this != &n) {
				state = n.state;
				action_idx = n.action_idx;
				parent_idx = n.parent_idx;
				g = n.g;
				f = n.f;
				in_closed = n.in_closed;
			}
			return *this;
		}
		PlanNode& operator=(PlanNode&& n) {
			if(this != &n) {
				state = pick(n.state);
				action_idx = n.action_idx;
				parent_idx = n.parent_idx;
				g = n.g;
				f = n.f;
				in_closed = n.in_closed;
			}
			return *this;
		}
	};

	Array<PlanNode> pool;
	VectorMap<WorldState, int> state_to_node;

	auto get_or_add_node = [&](const WorldState& ws) -> int {
		int idx = state_to_node.Find(ws);
		if(idx >= 0) return state_to_node[idx];

		int new_idx = pool.GetCount();
		PlanNode& n = pool.Add();
		n.state = ws;
		state_to_node.Add(ws, new_idx);
		return new_idx;
	};

	int start_idx = get_or_add_node(initial);
	pool[start_idx].g = 0.0;
	pool[start_idx].f = initial.GetUnmetCount(goal);

	Vector<int> open_set;
	open_set.Add(start_idx);

	int goal_node_idx = -1;

	while(!open_set.IsEmpty()) {
		// Find lowest f
		int best_pos = 0;
		double best_f = pool[open_set[0]].f;
		for(int i = 1; i < open_set.GetCount(); ++i) {
			double f = pool[open_set[i]].f;
			if(f < best_f) {
				best_f = f;
				best_pos = i;
			}
		}

		int current_idx = open_set[best_pos];
		open_set.Remove(best_pos);

		pool[current_idx].in_closed = true;

		if(pool[current_idx].state.MeetsConditions(goal)) {
			goal_node_idx = current_idx;
			break;
		}

		double curr_g = pool[current_idx].g;
		WorldState curr_state = pool[current_idx].state;

		// Explore possible actions
		for(int act_i = 0; act_i < actions.GetCount(); ++act_i) {
			const PlannerAction& act = actions[act_i];
			if(!curr_state.MeetsConditions(act.preconditions)) {
				continue;
			}

			WorldState next_state = curr_state;
			next_state.ApplyEffects(act.effects);

			int next_idx = get_or_add_node(next_state);
			if(pool[next_idx].in_closed) continue;

			double tentative_g = curr_g + act.cost;
			if(tentative_g < pool[next_idx].g) {
				pool[next_idx].parent_idx = current_idx;
				pool[next_idx].action_idx = act_i;
				pool[next_idx].g = tentative_g;
				pool[next_idx].f = tentative_g + next_state.GetUnmetCount(goal);

				bool in_open = false;
				for(int k = 0; k < open_set.GetCount(); ++k) {
					if(open_set[k] == next_idx) { in_open = true; break; }
				}
				if(!in_open) {
					open_set.Add(next_idx);
				}
			}
		}
	}

	if(goal_node_idx < 0) {
		return plan; // No plan found
	}

	if(out_total_cost) {
		*out_total_cost = pool[goal_node_idx].g;
	}

	// Trace back path
	int at = goal_node_idx;
	while(at != start_idx && at >= 0) {
		int act_idx = pool[at].action_idx;
		if(act_idx >= 0 && act_idx < actions.GetCount()) {
			plan.Add(actions[act_idx]);
		}
		at = pool[at].parent_idx;
	}

	// Reverse to start -> goal order
	for(int i = 0, j = plan.GetCount() - 1; i < j; ++i, --j) {
		PlannerAction tmp = plan[i];
		plan[i] = plan[j];
		plan[j] = tmp;
	}

	return plan;
}

}
