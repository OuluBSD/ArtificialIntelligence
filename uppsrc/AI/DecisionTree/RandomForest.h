#ifndef _AI_DecisionTree_RandomForest_h_
#define _AI_DecisionTree_RandomForest_h_

#include "DecisionTree.h"

namespace Upp {

class RandomForest {
private:
	Array<DecisionTree> trees;
	int n_estimators = 50;
	int max_depth = 10;
	int min_samples_split = 2;
	int min_samples_leaf = 1;
	int max_features = -1; // -1 means sqrt(num_features)
	DecisionTreeCriterion criterion = CRITERION_GINI;
	bool is_classifier = true;
	int num_features = 0;
	Vector<double> feature_importances;

public:
	RandomForest() {}

	RandomForest& SetEstimators(int n) { n_estimators = n; return *this; }
	RandomForest& SetMaxDepth(int d) { max_depth = d; return *this; }
	RandomForest& SetMinSamplesSplit(int s) { min_samples_split = s; return *this; }
	RandomForest& SetMinSamplesLeaf(int s) { min_samples_leaf = s; return *this; }
	RandomForest& SetMaxFeatures(int f) { max_features = f; return *this; }
	RandomForest& SetCriterion(DecisionTreeCriterion c) { criterion = c; return *this; }
	RandomForest& SetClassifier(bool c = true) { is_classifier = c; return *this; }

	void Fit(const Vector<Vector<double>>& X, const Vector<double>& y);
	double Predict(const Vector<double>& x) const;
	Vector<double> Predict(const Vector<Vector<double>>& X) const;
	VectorMap<int, double> PredictProba(const Vector<double>& x) const;

	const Vector<double>& GetFeatureImportances() const { return feature_importances; }
	int GetTreeCount() const { return trees.GetCount(); }
};

}

#endif
