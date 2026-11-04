#include <AI/Base/Base.h>

using namespace Upp;

void RunGoapCraftingScenario()
{
	Cout() << "\n=== Scenario 1: Survival Island GOAP Crafting ===\n";

	ActionPlanner planner;

	// Actions definition
	planner.AddAction("GatherSticks", 1.0)
	       .PostBool("has_wood", true);

	planner.AddAction("MineFlint", 2.0)
	       .PostBool("has_flint", true);

	planner.AddAction("CraftCampfire", 1.5)
	       .PreBool("has_wood", true)
	       .PreBool("has_flint", true)
	       .PostBool("has_fire", true);

	planner.AddAction("CatchFish", 3.0)
	       .PostBool("has_raw_fish", true);

	planner.AddAction("CookFish", 1.0)
	       .PreBool("has_fire", true)
	       .PreBool("has_raw_fish", true)
	       .PostBool("is_fed", true);

	planner.AddAction("BuildLeanToShelter", 4.0)
	       .PreBool("has_wood", true)
	       .PostBool("has_shelter", true);

	// Initial island survivor state
	WorldState initial;
	initial.SetBool("has_wood", false);
	initial.SetBool("has_flint", false);
	initial.SetBool("has_fire", false);
	initial.SetBool("has_raw_fish", false);
	initial.SetBool("is_fed", false);
	initial.SetBool("has_shelter", false);

	// Ultimate survival goal
	WorldState goal;
	goal.SetBool("is_fed", true);
	goal.SetBool("has_shelter", true);

	Cout() << "Goal State: is_fed = true, has_shelter = true\n";
	Cout() << "Computing optimal plan...\n";

	double total_cost = 0;
	Vector<PlannerAction> plan = planner.Plan(initial, goal, &total_cost);

	Cout() << "\nGenerated Plan (" << plan.GetCount() << " steps, Total Energy Cost = " << total_cost << "):\n";
	for(int i = 0; i < plan.GetCount(); ++i) {
		Cout() << "  [" << (i + 1) << "] " << plan[i].name << " (energy: " << plan[i].cost << ")\n";
	}
}

void RunGridNavigationScenario()
{
	Cout() << "\n=== Scenario 2: Autonomous Grid Navigation (A*) ===\n";

	const int W = 15;
	const int H = 9;
	GridMap map(W, H);

	// Maze walls
	for(int y = 0; y < 7; ++y) map.SetBlocked(5, y, true);
	for(int y = 2; y < 9; ++y) map.SetBlocked(10, y, true);

	GridPoint start(1, 1);
	GridPoint goal(13, 7);

	double cost = 0;
	Vector<GridPoint> path = GridAStar::FindPath(map, start, goal, false, &cost);

	Index<Point> path_points;
	for(const auto& pt : path) {
		path_points.Add(Point(pt.x, pt.y));
	}

	Cout() << "Map Legend: [S] Start, [G] Goal, [*] A* Path, [#] Wall, [.] Open\n\n";
	for(int y = 0; y < H; ++y) {
		String line;
		for(int x = 0; x < W; ++x) {
			if(x == start.x && y == start.y) line.Cat("S ");
			else if(x == goal.x && y == goal.y) line.Cat("G ");
			else if(path_points.Find(Point(x, y)) >= 0) line.Cat("* ");
			else if(map.IsBlocked(x, y)) line.Cat("# ");
			else line.Cat(". ");
		}
		Cout() << "  " << line << "\n";
	}

	Cout() << "\nFound shortest path: " << path.GetCount() << " waypoints, travel cost = " << cost << "\n";
}

CONSOLE_APP_MAIN
{
	Cout() << "========================================================\n";
	Cout() << "  ArtificialIntelligence - Planner & Navigation Demo\n";
	Cout() << "========================================================\n";

	RunGoapCraftingScenario();
	RunGridNavigationScenario();

	Cout() << "\nDemo completed successfully.\n";
}
