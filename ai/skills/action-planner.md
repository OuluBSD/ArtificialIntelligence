# Action Planner - A* Route Search for Repository Goals

## Purpose

This skill enables an AI agent to autonomously plan and execute actions to achieve repository goals. It uses an A* search algorithm where goals are represented as `vector<bool>` atom states, and actions are state transformations.

## Core Concept

### Atoms

Atoms are boolean variables that represent the current state of the repository:

```cpp
vector<bool> state = {false, false, true, false, ...};
```

Each atom represents a specific condition or feature:
- `atom[0]`: "Core package builds successfully"
- `atom[1]`: "CtrlLib package builds successfully"
- `atom[2]`: "All tests pass"
- `atom[3]`: "Documentation complete"
- ...

### Goal State

A goal is a specific `vector<bool>` state that the agent wants to achieve:

```cpp
vector<bool> goal = {true, true, true, true, ...};  // All atoms true
vector<bool> partial_goal = {true, false, true, false, ...};  // Partial
```

### Actions

An action is a tuple of:
1. **Prerequisites**: Which atoms must be in specific states for action to work
2. **Effects**: Which atoms change state when action executes

```cpp
struct Action {
    string name;
    vector<bool> requires;    // Prerequisites (true/false/don't care)
    vector<bool> preconditions; // Atoms that must be true before action
    vector<bool> postconditions; // Atoms that become true after action
};
```

### Example Actions

```cpp
Action build_core = {
    .name = "Build Core Package",
    .preconditions = {},  // No prerequisites
    .postconditions = {true, false, false, false, ...}  // atom[0] = true
};

Action build_ctrllib = {
    .name = "Build CtrlLib Package",
    .preconditions = {true, false, false, false, ...},  // Requires Core built
    .postconditions = {true, true, false, false, ...}  // atom[1] = true
};

Action run_tests = {
    .name = "Run All Tests",
    .preconditions = {true, true, false, false, ...},  // Requires Core + CtrlLib
    .postconditions = {true, true, true, false, ...}  // atom[2] = true
};
```

## Goal Directory Structure

Goals are defined in the Manager repo at `../Manager/0-goal/<repo-name>/`.

All files are JSON per `manager-json-format.md` — never markdown.

```
../Manager/0-goal/ai-upp/
├── final/          # Final product goals
│   └── all_complete.json
├── release/        # Release milestone goals
│   └── v1_release.json
├── intermediate/   # Intermediate goals
│   ├── build_all.json
│   └── test_all.json
├── product/        # Product definition goals
│   └── core_features.json
├── marketing/      # Marketing goals
│   └── demo_ready.json
├── actions.json    # Action definitions
└── atoms.json      # Atom definitions
```

### Atoms Definition (atoms.json)

```json
{
  "id": "ai-upp/atoms",
  "kind": "report",
  "title": "Atoms Definition",
  "notes": [
    {"heading": "Repository Atoms", "items": [
      "0. core_builds: Core package builds successfully",
      "1. ctrllib_builds: CtrlLib package builds successfully",
      "2. ide_builds: IDE builds successfully",
      "3. tests_pass: All unit tests pass",
      "4. docs_complete: Documentation complete",
      "5. examples_work: All examples work"
    ]}
  ]
}
```

### Actions Definition (actions.json)

```json
{
  "id": "ai-upp/actions",
  "kind": "report",
  "title": "Actions Definition",
  "notes": [
    {"heading": "Build Actions", "items": [
      "build_core | preconditions: none | postconditions: core_builds | command: bin\\build.exe -M 2 -m MSVS26x64 Core",
      "build_ctrllib | preconditions: core_builds | postconditions: ctrllib_builds | command: bin\\build.exe -M 2 -m MSVS26x64 CtrlLib",
      "build_ide | preconditions: ctrllib_builds | postconditions: ide_builds | command: bin\\build.exe -M 2 -m MSVS26x64 ide"
    ]},
    {"heading": "Test Actions", "items": [
      "run_all_tests | preconditions: core_builds, ctrllib_builds | postconditions: tests_pass | command: bin\\build.exe -M 2 -m MSVS26x64 upptst"
    ]},
    {"heading": "Documentation Actions", "items": [
      "complete_docs | preconditions: ide_builds | postconditions: docs_complete"
    ]}
  ]
}
```

### Goal Definition (e.g., intermediate/build_all.json)

```json
{
  "id": "ai-upp/intermediate/build_all",
  "kind": "report",
  "title": "Goal: Build All Packages",
  "purpose": ["All buildable packages compile without errors."],
  "notes": [
    {"heading": "Target State", "items": [
      "core_builds: true",
      "ctrllib_builds: true",
      "ide_builds: true",
      "tests_pass: false",
      "docs_complete: false",
      "examples_work: false"
    ]},
    {"heading": "Notes", "items": ["This is an intermediate goal for repository health."]}
  ]
}
```

## A* Search Algorithm

### Heuristic: Hamming Distance

The heuristic is the Hamming distance between current state and goal state:

```cpp
int hamming_distance(vector<bool> current, vector<bool> goal) {
    int distance = 0;
    for (int i = 0; i < current.size(); i++) {
        if (current[i] != goal[i]) distance++;
    }
    return distance;
}
```

### A* Implementation

```cpp
struct Node {
    vector<bool> state;
    int g_cost;      // Cost from start
    int h_cost;      // Heuristic to goal
    int f_cost;      // g_cost + h_cost
    vector<string> path;  // Actions taken to reach this state
    
    bool operator<(const Node& other) const {
        return f_cost > other.f_cost;  // For priority queue (min-heap)
    }
};

vector<string> astar_search(
    vector<bool> start,
    vector<bool> goal,
    vector<Action> actions
) {
    priority_queue<Node> open;
    unordered_set<string> closed;  // Visited states
    
    open.push({start, 0, hamming_distance(start, goal), 0, {}});
    
    while (!open.empty()) {
        Node current = open.top();
        open.pop();
        
        string state_key = join(current.state);
        if (closed.contains(state_key)) continue;
        closed.insert(state_key);
        
        if (current.state == goal) {
            return current.path;  // Found goal
        }
        
        for (const Action& action : actions) {
            if (can_execute(current.state, action)) {
                vector<bool> new_state = execute(current.state, action);
                int g = current.g_cost + 1;
                int h = hamming_distance(new_state, goal);
                
                open.push({
                    new_state,
                    g,
                    h,
                    g + h,
                    current.path + {action.name}
                });
            }
        }
    }
    
    return {};  // No path found
}

bool can_execute(vector<bool> state, const Action& action) {
    for (int i = 0; i < state.size(); i++) {
        if (action.preconditions[i] && !state[i]) {
            return false;  // Required atom not true
        }
    }
    return true;
}

vector<bool> execute(vector<bool> state, const Action& action) {
    vector<bool> new_state = state;
    for (int i = 0; i < new_state.size(); i++) {
        if (action.postconditions[i]) {
            new_state[i] = true;
        }
    }
    return new_state;
}
```

## Agent Workflow

### Phase 1: Load Definitions

1. Read atoms definition from goal directory
2. Read actions definition from goal directory
3. Read target goal file

```powershell
# Load atoms
$atoms = Parse-Atoms "./goal/atoms.json"

# Load actions
$actions = Parse-Actions "./goal/actions.json"

# Load goal
$goal = Parse-Goal "./goal/intermediate/build_all.json"
```

### Phase 2: Discover Current State

Analyze repository to determine current atom states:

```powershell
$state = @()
$state += (Test-Path "bin/Core")           # atom[0]: core_builds
$state += (Test-Path "bin/CtrlLib")        # atom[1]: ctrllib_builds
$state += (Test-Path "bin/ide")            # atom[2]: ide_builds
# ... etc
```

### Phase 3: A* Planning

```powershell
$path = AStar-Search -Start $state -Goal $goal -Actions $actions
```

### Phase 4: Execute Plan

```powershell
foreach ($actionName in $path) {
    $action = Get-Action $actionName
    
    # Acquire locks (see multi-agent-coordination skill)
    Acquire-Lock -Area $action.target -Type write
    
    # Execute action
    Write-Output "Executing: $($action.name)"
    Invoke-Expression $action.command
    
    # Update state
    $state = Execute-Action $state $action
    
    # Release locks
    Release-Lock -Area $action.target
    
    # Log progress
    Write-Output "State: $state"
}
```

### Phase 5: Verify and Report

```powershell
# Check final state matches goal
$final_state = Discover-Current-State
if ($final_state -eq $goal) {
    Write-Output "Goal achieved!"
} else {
    Write-Output "Goal not fully achieved. Remaining: $remaining"
}
```

## Integration with Multi-Agent Coordination

### Coordination Points

1. **Lock acquisition**: Before executing action, acquire locks on target area
2. **Conflict detection**: If another agent holds lock, wait or redirect
3. **Progress sharing**: Update plan file with current action
4. **State synchronization**: Other agents may change state during execution

### Concurrent Planning

Multiple agents can run A* simultaneously but execute different actions:
- Agent A plans and executes actions 1-3
- Agent B plans and executes actions 4-6
- Both update shared state tracking

## Practical Examples

### Example 1: Build All Packages

```powershell
$goal = [true, true, true, false, false, true]
$path = AStar-Search -Start $current_state -Goal $goal -Actions $build_actions
# Returns: ["build_core", "build_ctrllib", "build_ide", "build_examples"]
```

### Example 2: Complete Testing Pipeline

```powershell
$goal = [true, true, true, true, false, false]
$path = AStar-Search -Start $current_state -Goal $goal -Actions $all_actions
# Returns: ["build_core", "build_ctrllib", "run_tests"]
```

### Example 3: Full Release

```powershell
$goal = [true, true, true, true, true, true]
$path = AStar-Search -Start $current_state -Goal $goal -Actions $all_actions
# Returns: ["build_core", "build_ctrllib", "build_ide", "run_tests", "complete_docs"]
```

## Error Handling

### No Path Found

If A* returns empty path:
- Goal may be impossible with current actions
- Some preconditions may be circular
- Check action definitions for errors

```powershell
if ($path.Count -eq 0) {
    Write-Output "No path to goal found!"
    Write-Output "Checking for circular dependencies..."
    Check-Action-Graph $actions
}
```

### Action Execution Fails

If an action fails during execution:
1. Log the failure
2. Attempt recovery or retry
3. If unrecoverable, replan from current state

```powershell
try {
    Invoke-Expression $action.command
} catch {
    Write-Output "Action failed: $action.name"
    Write-Output "Replanning from current state..."
    $path = AStar-Search -Start $current_state -Goal $goal -Actions $actions
}
```

### State Mismatch

If state changes unexpectedly:
- Another agent may have modified repository
- File system state may have changed
- Re-discover current state and replan

```powershell
$discovered = Discover-Current-State
if ($discovered -ne $state) {
    Write-Output "State mismatch detected! Re-discovering..."
    $state = $discovered
    $path = AStar-Search -Start $state -Goal $goal -Actions $actions
}
```

## Success Metrics

Action planning is successful when:
1. A* finds optimal or near-optimal path to goal
2. Actions execute successfully
3. State correctly reflects repository condition
4. Multiple agents can coordinate via shared goals
5. Failed actions trigger graceful replanning

## Checklist

- [ ] Atoms defined in goal directory
- [ ] Actions defined with pre/post conditions
- [ ] Goal files created for each target
- [ ] A* implementation handles all edge cases
- [ ] Lock coordination integrated
- [ ] State discovery is accurate
- [ ] Error handling covers all failure modes
- [ ] Progress logging is clear