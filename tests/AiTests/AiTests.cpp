#include <AI/Base/Base.h>
#include <AI/DecisionTree/DecisionTree.h>
#include <AI/DecisionTree/RandomForest.h>
#include <AI/Logic/TheoremProver.h>
#include <TrustCore/TrustCore.h>

using namespace Upp;

void TestGraph()
{
	Cout() << "==> Testing Graph & Topological Sort...\n";
	Graph g;
	int a = g.AddNode("A");
	int b = g.AddNode("B");
	int c = g.AddNode("C");
	int d = g.AddNode("D");

	g.AddEdge(a, b, 1.0);
	g.AddEdge(b, c, 2.0);
	g.AddEdge(a, c, 5.0);
	g.AddEdge(c, d, 1.0);

	ASSERT(!g.HasCycle());

	Vector<int> topo = g.TopologicalSort();
	ASSERT(topo.GetCount() == 4);

	double cost = 0.0;
	Vector<int> path = g.ShortestPathDijkstra(a, d, &cost);
	ASSERT(path.GetCount() == 4); // A -> B -> C -> d
	ASSERT(cost == 4.0); // 1 + 2 + 1
	Cout() << "    Graph test passed (Dijkstra cost = " << cost << ")\n";
}

void TestGridAStar()
{
	Cout() << "==> Testing Grid A* Pathfinding...\n";
	GridMap map(10, 10);
	// Place vertical wall at x=5, y=0..7 with an opening at y=8
	for(int y = 0; y <= 7; ++y) {
		map.SetBlocked(5, y, true);
	}

	GridPoint start(2, 2);
	GridPoint goal(8, 2);

	double cost = 0.0;
	Vector<GridPoint> path = GridAStar::FindPath(map, start, goal, false, &cost);

	ASSERT(!path.IsEmpty());
	ASSERT(path[0] == start);
	ASSERT(path.Top() == goal);

	// Ensure no obstacle was crossed
	for(const auto& pt : path) {
		ASSERT(!map.IsBlocked(pt.x, pt.y));
	}
	Cout() << "    Grid A* test passed (Path steps = " << path.GetCount() << ", cost = " << cost << ")\n";
}

void TestActionPlanner()
{
	Cout() << "==> Testing GOAP ActionPlanner...\n";
	ActionPlanner planner;

	planner.AddAction("CollectWood", 1.0)
	       .PostBool("has_wood", true);

	planner.AddAction("BuildWorkbench", 2.0)
	       .PreBool("has_wood", true)
	       .PostBool("has_workbench", true);

	planner.AddAction("CraftAxe", 3.0)
	       .PreBool("has_workbench", true)
	       .PreBool("has_wood", true)
	       .PostBool("has_axe", true);

	planner.AddAction("BuildShelter", 5.0)
	       .PreBool("has_axe", true)
	       .PreBool("has_wood", true)
	       .PostBool("shelter_built", true);

	WorldState initial;
	initial.SetBool("has_wood", false);
	initial.SetBool("has_workbench", false);
	initial.SetBool("has_axe", false);
	initial.SetBool("shelter_built", false);

	WorldState goal;
	goal.SetBool("shelter_built", true);

	double total_cost = 0.0;
	Vector<PlannerAction> plan = planner.Plan(initial, goal, &total_cost);

	ASSERT(!plan.IsEmpty());
	ASSERT(plan.Top().name == "BuildShelter");

	Cout() << "    Plan generated (" << plan.GetCount() << " actions, total cost = " << total_cost << "):\n";
	for(int i = 0; i < plan.GetCount(); ++i) {
		Cout() << "      [" << (i + 1) << "] " << plan[i].name << " (cost " << plan[i].cost << ")\n";
	}
}

void TestDecisionTreeAndRandomForest()
{
	Cout() << "==> Testing DecisionTree and RandomForest...\n";

	// Generate dataset: 2 features, 2 classes (separable threshold: x0 + x1 > 1.0)
	Vector<Vector<double>> X;
	Vector<double> y;

	for(int i = 0; i < 100; ++i) {
		double x0 = (double)rand() / RAND_MAX * 2.0;
		double x1 = (double)rand() / RAND_MAX * 2.0;
		Vector<double> sample;
		sample.Add(x0);
		sample.Add(x1);
		X.Add(pick(sample));
		y.Add((x0 + x1 > 1.2) ? 1.0 : 0.0);
	}

	DecisionTree dt;
	dt.SetMaxDepth(5).SetMinSamplesSplit(2).SetClassifier(true);
	dt.Fit(X, y);

	int correct_dt = 0;
	for(int i = 0; i < X.GetCount(); ++i) {
		if(dt.Predict(X[i]) == y[i]) correct_dt++;
	}
	double acc_dt = (double)correct_dt / X.GetCount();
	ASSERT(acc_dt >= 0.90);
	Cout() << "    DecisionTree accuracy: " << Format("%.1f%%", acc_dt * 100) << "\n";

	RandomForest rf;
	rf.SetEstimators(20).SetMaxDepth(6).SetClassifier(true);
	rf.Fit(X, y);

	int correct_rf = 0;
	for(int i = 0; i < X.GetCount(); ++i) {
		if(rf.Predict(X[i]) == y[i]) correct_rf++;
	}
	double acc_rf = (double)correct_rf / X.GetCount();
	ASSERT(acc_rf >= 0.90);
	Cout() << "    RandomForest accuracy: " << Format("%.1f%%", acc_rf * 100) << "\n";
}

void TestGeneticOptimizer()
{
	Cout() << "==> Testing GeneticOptimizer...\n";
	// Chromosome: single scalar double, goal: maximize -(x - 42.0)^2 (optimum at 42.0)
	GeneticOptimizer<double> opt;
	opt.population_size = 50;
	opt.max_generations = 100;
	opt.mutation_rate = 0.3;

	Vector<double> init_pop;
	for(int i = 0; i < 50; ++i) {
		init_pop.Add((double)(rand() % 200 - 100));
	}
	opt.Initialize(init_pop);

	auto eval = [](const double& x) -> double {
		double diff = x - 42.0;
		return -(diff * diff);
	};

	auto cross = [](const double& a, const double& b) -> double {
		return (a + b) / 2.0;
	};

	auto mutate = [](double& x, double rate) {
		if((double)rand() / RAND_MAX < rate) {
			x += ((double)rand() / RAND_MAX - 0.5) * 4.0;
		}
	};

	auto best = opt.Evolve(eval, cross, mutate);
	ASSERT(fabs(best.chromosome - 42.0) < 2.0);
	Cout() << "    GeneticOptimizer test passed (Best x = " << best.chromosome << ", fitness = " << best.fitness << ")\n";
}

void TestTrustCore()
{
	Cout() << "==> Testing TrustCore (TrustNet & AppleSeed)...\n";
	TrustNet net;
	Vector<TrustEdge> edges;
	TrustEdge& e1 = edges.Add(); e1.Set(0, 1, 0.9f);
	TrustEdge& e2 = edges.Add(); e2.Set(1, 2, 0.8f);
	TrustEdge& e3 = edges.Add(); e3.Set(0, 3, 0.2f);

	Index<int> distrusted;
	net.Load(0.5, 4, 0, edges, distrusted);
	RankingMap rankings = net.GetRankings();
	ASSERT(!rankings.IsEmpty());
	Cout() << "    TrustCore test passed (" << rankings.GetCount() << " ranked nodes)\n";
}

void TestTheoremProver()
{
	Cout() << "==> Testing AI/Logic TheoremProver...\n";
	TheoremProver::flag_silent_prover = true;

	Vector<String> tokens = TheoremProver::Lex("P implies P");
	TheoremProver::NodeVar formula = TheoremProver::Parse(tokens);
	ASSERT(formula.GetNode() != nullptr);

	Index<TheoremProver::NodeVar> axioms;
	bool provable = TheoremProver::ProveFormula(axioms, formula);
	ASSERT(provable);
	Cout() << "    TheoremProver test passed (Proved 'P implies P')\n";
}

CONSOLE_APP_MAIN
{
	Cout() << "========================================\n";
	Cout() << "ArtificialIntelligence Unit Test Suite\n";
	Cout() << "========================================\n";

	TestGraph();
	TestGridAStar();
	TestActionPlanner();
	TestDecisionTreeAndRandomForest();
	TestGeneticOptimizer();
	TestTrustCore();
	TestTheoremProver();

	Cout() << "\n[PASS] All ArtificialIntelligence tests completed successfully!\n";
}
