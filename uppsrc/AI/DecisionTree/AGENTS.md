# AGENTS - AI/DecisionTree

**Scope**: Applies to `uppsrc/AI/DecisionTree`.

## Purpose
Native U++ implementation of CART decision trees and random forest ensembles:
- `DecisionTree.{h,cpp}`: Classification and regression trees supporting Gini impurity, Entropy, and MSE splitting, continuous features, depth limits, and feature importances.
- `RandomForest.{h,cpp}`: Bagging ensemble of decision trees with feature sub-sampling, majority voting, probability distributions (`PredictProba`), and ensemble feature importances.

## Invariants
- Zero third-party machine learning dependencies (C++17 Core only).
- Memory-safe containers (`Vector`, `Array`).
