# AGENTS - ArtificialIntelligence Guide

Welcome to the `ArtificialIntelligence` repository. This repository provides foundational, headless, classical and modern AI algorithms written in modern C++17 for the Ultimate++ ecosystem. All packages depend strictly on `Core` (from `BazaarCore`) and require zero GUI or heavy external dependencies.

---

## Repository Map

| Package | Path | Description |
|---|---|---|
| `AI/Base` | `uppsrc/AI/Base` | Graph algorithms (Dijkstra, Topological Sort), 2D Grid Pathfinding (A*, Manhattan/Euclidean heuristics), Goal-Oriented Action Planning (GOAP ActionPlanner), and GeneticOptimizer. |
| `AI/DecisionTree` | `uppsrc/AI/DecisionTree` | High-performance CART Decision Trees (Gini, Entropy, MSE criteria) and Random Forest bagging ensembles with out-of-bag feature importances. |
| `AI/Logic` | `uppsrc/AI/Logic` | First-order and propositional logic parser, AST representation, evaluator, and resolution-based Theorem Prover (`TheoremProver`). |
| `AI/Algo` | `uppsrc/AI/Algo` | Fast binary search and heuristic indexing routines. |
| `AI/RefCore` | `uppsrc/AI/RefCore` | Reference-counted pointer context and AST node management. |
| `TrustCore` | `uppsrc/TrustCore` | Trust network propagation algorithms, reputation dynamics, and AppleSeed seed-based trust scoring (`TrustNet`, `TrustGraph`). |
| `tests/AiTests` | `tests/AiTests` | Comprehensive unit test suite exercising all algorithms with assert validations. |
| `examples/PlannerDemo` | `examples/PlannerDemo` | Demonstration of GOAP survival crafting and 2D grid obstacle navigation. |
| `examples/DecisionTreeDemo` | `examples/DecisionTreeDemo` | Demonstration of Decision Tree and Random Forest classifier training, prediction, and feature importance analysis. |

---

## Architectural Principles & Rules

1. **Headless & Pure Core**:
   - Zero GUI dependencies (`CtrlCore`, `CtrlLib` must never be referenced).
   - Only relies on U++ `Core` via `BazaarCore`.

2. **NTL Containers & Move Semantics**:
   - Follow `ai/rules/movable-and-ownership.md`.
   - All state structures stored in `Vector<T>` or `Array<T>` must declare `Moveable<T>`.
   - Never assume copy constructors exist on `Vector<T>` or `VectorMap<K, V>`. Use `X.Add() <<= item` or `X.Add(pick(item))` when populating nested containers.

3. **Memory Safety & Graph Pointers**:
   - For search pools that expand during traversal (e.g. A*, GOAP), use `Array<T>` or index lookups rather than persistent references into dynamic `Vector<T>` buffers to prevent iterator/reference invalidation on reallocation.

---

## Building and Testing

Build using `uppbuilder` from `OuluBSD/UppBuilder`:

```bash
# Run unit tests
uppbuilder tests/AiTests -m CLANG
./bin/AiTests

# Run GOAP and A* planner demo
uppbuilder examples/PlannerDemo -m CLANG
./bin/PlannerDemo

# Run Decision Tree and Random Forest demo
uppbuilder examples/DecisionTreeDemo -m CLANG
./bin/DecisionTreeDemo
```
