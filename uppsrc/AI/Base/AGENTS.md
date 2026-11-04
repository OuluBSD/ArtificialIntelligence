# AGENTS - AI/Base

**Scope**: Applies to `uppsrc/AI/Base`.

## Purpose
Core algorithmic foundations for classical and deterministic artificial intelligence:
- `Graph.{h,cpp}`: Graph data structures, adjacency lists, cycle detection, topological sorting, and Dijkstra shortest paths.
- `Search.{h,cpp}`: 2D Grid pathfinding with A*, Manhattan/Euclidean heuristics, and Minimax with Alpha-Beta pruning for adversarial game trees.
- `ActionPlanner.{h,cpp}`: Goal-Oriented Action Planning (GOAP) based on world state transitions and A* forward search.
- `GeneticOptimizer.h`: Genetic algorithm framework with customizable chromosome representations, selection, crossover, and mutation operators.

## Invariants
- Zero GUI dependencies (Core only).
- All custom structs inherit from or specialize `Moveable<T>`.
- Deterministic behavior: algorithms return reproducible paths and actions given identical inputs.
