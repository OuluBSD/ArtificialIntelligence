#ifndef _AI_Base_ActionPlanner_h_
#define _AI_Base_ActionPlanner_h_

#include <Core/Core.h>

namespace Upp {

struct WorldState : Moveable<WorldState> {
	VectorMap<String, Value> values;

	WorldState() {}

	WorldState(const WorldState& s) {
		for(int i = 0; i < s.values.GetCount(); ++i)
			values.Add(s.values.GetKey(i), s.values[i]);
	}

	WorldState(WorldState&& s) : values(pick(s.values)) {}

	WorldState& operator=(const WorldState& s) {
		if(this != &s) {
			values.Clear();
			for(int i = 0; i < s.values.GetCount(); ++i)
				values.Add(s.values.GetKey(i), s.values[i]);
		}
		return *this;
	}

	WorldState& operator=(WorldState&& s) {
		if(this != &s) {
			values = pick(s.values);
		}
		return *this;
	}

	void Clear() { values.Clear(); }

	void Set(const String& key, Value val) {
		values.GetAdd(key) = val;
	}

	void SetBool(const String& key, bool b) {
		Set(key, Value(b));
	}

	Value Get(const String& key, Value def = Value()) const {
		int idx = values.Find(key);
		return (idx >= 0) ? values[idx] : def;
	}

	bool GetBool(const String& key, bool def = false) const {
		Value v = Get(key);
		return v.IsNull() ? def : (bool)v;
	}

	bool Has(const String& key) const {
		return values.Find(key) >= 0;
	}

	// Checks if all conditions specified in 'conditions' are satisfied by *this
	bool MeetsConditions(const WorldState& conditions) const {
		for(int i = 0; i < conditions.values.GetCount(); ++i) {
			const String& k = conditions.values.GetKey(i);
			const Value& req = conditions.values[i];
			int idx = values.Find(k);
			if(idx < 0 || values[idx] != req) {
				return false;
			}
		}
		return true;
	}

	// Applies effects from an action into this state
	void ApplyEffects(const WorldState& effects) {
		for(int i = 0; i < effects.values.GetCount(); ++i) {
			Set(effects.values.GetKey(i), effects.values[i]);
		}
	}

	// Distance metric / heuristic to target goal state (number of unmet conditions)
	int GetUnmetCount(const WorldState& goal) const {
		int unmet = 0;
		for(int i = 0; i < goal.values.GetCount(); ++i) {
			const String& k = goal.values.GetKey(i);
			int idx = values.Find(k);
			if(idx < 0 || values[idx] != goal.values[i]) {
				unmet++;
			}
		}
		return unmet;
	}

	bool operator==(const WorldState& o) const {
		if(values.GetCount() != o.values.GetCount()) return false;
		for(int i = 0; i < values.GetCount(); ++i) {
			const String& k = values.GetKey(i);
			int o_idx = o.values.Find(k);
			if(o_idx < 0 || values[i] != o.values[o_idx]) return false;
		}
		return true;
	}

	hash_t GetHashValue() const {
		hash_t h = 0;
		for(int i = 0; i < values.GetCount(); ++i) {
			h = CombineHash(h, values.GetKey(i).GetHashValue(), values[i].GetHashValue());
		}
		return h;
	}

	String ToString() const;
};

struct PlannerAction : Moveable<PlannerAction> {
	String name;
	double cost = 1.0;
	WorldState preconditions;
	WorldState effects;

	PlannerAction() {}
	PlannerAction(const String& name, double cost = 1.0) : name(name), cost(cost) {}
	PlannerAction(const PlannerAction& a) : name(a.name), cost(a.cost), preconditions(a.preconditions), effects(a.effects) {}
	PlannerAction(PlannerAction&& a) : name(pick(a.name)), cost(a.cost), preconditions(pick(a.preconditions)), effects(pick(a.effects)) {}

	PlannerAction& operator=(const PlannerAction& a) {
		if(this != &a) {
			name = a.name;
			cost = a.cost;
			preconditions = a.preconditions;
			effects = a.effects;
		}
		return *this;
	}

	PlannerAction& operator=(PlannerAction&& a) {
		if(this != &a) {
			name = pick(a.name);
			cost = a.cost;
			preconditions = pick(a.preconditions);
			effects = pick(a.effects);
		}
		return *this;
	}

	PlannerAction& Pre(const String& key, Value val) {
		preconditions.Set(key, val);
		return *this;
	}

	PlannerAction& PreBool(const String& key, bool val = true) {
		preconditions.SetBool(key, val);
		return *this;
	}

	PlannerAction& Post(const String& key, Value val) {
		effects.Set(key, val);
		return *this;
	}

	PlannerAction& PostBool(const String& key, bool val = true) {
		effects.SetBool(key, val);
		return *this;
	}

	PlannerAction& Cost(double c) {
		cost = c;
		return *this;
	}
};

class ActionPlanner {
private:
	Vector<PlannerAction> actions;

public:
	ActionPlanner() {}

	void Clear() { actions.Clear(); }
	PlannerAction& AddAction(const String& name, double cost = 1.0);
	PlannerAction& AddAction(const PlannerAction& act);

	int GetActionCount() const { return actions.GetCount(); }
	const PlannerAction& GetAction(int i) const { return actions[i]; }

	// A* Plan generation: finds minimal-cost sequence of actions from initial state to goal state
	Vector<PlannerAction> Plan(const WorldState& initial, const WorldState& goal, double* out_total_cost = nullptr);
};

}

#endif
