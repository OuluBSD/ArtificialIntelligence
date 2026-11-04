# ArtificialIntelligence

[![ArtificialIntelligence CI](https://github.com/OuluBSD/ArtificialIntelligence/actions/workflows/ci.yml/badge.svg)](https://github.com/OuluBSD/ArtificialIntelligence/actions/workflows/ci.yml)

A headless, high-performance C++17 Artificial Intelligence library for the Ultimate++ ecosystem.

Zero external dependencies beyond standard C++17 and U++ `Core`.

---

## Features

- **Goal-Oriented Action Planning (GOAP)**:
  - A* search over state spaces with precondition and effect graphs.
  - Generates optimal sequence of actions to satisfy goal conditions.
- **Pathfinding & Graph Theory**:
  - 2D Grid A* pathfinder with Manhattan and Euclidean heuristics and obstacle maps.
  - Directed graph representation with cycle detection, topological sorting, and Dijkstra shortest path search.
  - Minimax search with Alpha-Beta pruning.
- **Machine Learning**:
  - **CART Decision Trees**: Gini impurity, entropy, and MSE split criteria for continuous and categorical attributes.
  - **Random Forest**: Bagging ensembles with random feature subset selection, voting classification, probability estimation, and Gini feature importances.
- **Formal Logic & Theorem Proving**:
  - Propositional and first-order logic parser (`implies`, `iff`, `and`, `or`, `not`, `forall`, `exists`).
  - AST evaluation, unification, and automated resolution theorem prover.
- **Trust Networks & Reputation Dynamics**:
  - Network-wide trust score propagation via AppleSeed and PageRank-inspired algorithms (`TrustCore`).
- **Genetic Algorithms**:
  - Customizable chromosome optimizer with tournament selection, elitism, crossover, and mutation operators.

---

## Repository Layout

```
ArtificialIntelligence/
├── uppsrc/
│   ├── AI/
│   │   ├── Base/          # GOAP, A*, Minimax, Graphs, GeneticOptimizer
│   │   ├── DecisionTree/  # DecisionTree and RandomForest
│   │   ├── Logic/         # TheoremProver, logic parser, evaluator
│   │   ├── Algo/          # Fast binary search & indexing routines
│   │   └── RefCore/       # Reference-counted pointers & AST memory
│   └── TrustCore/         # AppleSeed and trust propagation models
├── tests/
│   └── AiTests/           # Unit test suite covering all packages
├── examples/
│   ├── PlannerDemo/       # GOAP island crafting & grid navigation demo
│   └── DecisionTreeDemo/  # Decision tree & random forest ML demo
└── .github/workflows/
    └── ci.yml             # Linux (Clang/GCC), macOS (Clang), Windows (MSVC) CI
```

---

## Quick Start

### Build with `uppbuilder`

```bash
# Run unit tests
uppbuilder tests/AiTests -m CLANG
./bin/AiTests

# Run planner demonstration
uppbuilder examples/PlannerDemo -m CLANG
./bin/PlannerDemo

# Run machine learning demonstration
uppbuilder examples/DecisionTreeDemo -m CLANG
./bin/DecisionTreeDemo
```

---

## License

BSD 2-Clause License (or Ultimate++ permissive license).
