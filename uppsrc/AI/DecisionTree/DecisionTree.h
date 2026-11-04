#ifndef _AI_DecisionTree_DecisionTree_h_
#define _AI_DecisionTree_DecisionTree_h_

#include <Core/Core.h>
#include <float.h>

namespace Upp {

enum DecisionTreeCriterion {
	CRITERION_GINI,
	CRITERION_ENTROPY,
	CRITERION_MSE // for regression
};

class DecisionTree {
public:
	struct Node : Moveable<Node> {
		bool is_leaf = true;
		int feature_idx = -1;
		double threshold = 0.0;
		double value = 0.0; // class label or regression mean
		int left_child = -1;
		int right_child = -1;
		int samples = 0;
		double impurity = 0.0;
	};

private:
	Vector<Node> nodes;
	int max_depth = 10;
	int min_samples_split = 2;
	int min_samples_leaf = 1;
	int max_features = -1; // -1 means all features
	DecisionTreeCriterion criterion = CRITERION_GINI;
	bool is_classifier = true;
	int num_classes = 0;
	int num_features = 0;
	Vector<double> feature_importances;

	int BuildTree(const Vector<Vector<double>>& X, const Vector<double>& y, const Vector<int>& sample_indices, int current_depth);
	void FindBestSplit(const Vector<Vector<double>>& X, const Vector<double>& y, const Vector<int>& sample_indices,
	                   int& best_feature, double& best_threshold, double& best_gain,
	                   Vector<int>& left_indices, Vector<int>& right_indices);
	double CalculateImpurity(const Vector<double>& y, const Vector<int>& sample_indices) const;

public:
	DecisionTree() {}

	DecisionTree& SetMaxDepth(int d) { max_depth = d; return *this; }
	DecisionTree& SetMinSamplesSplit(int s) { min_samples_split = s; return *this; }
	DecisionTree& SetMinSamplesLeaf(int s) { min_samples_leaf = s; return *this; }
	DecisionTree& SetMaxFeatures(int f) { max_features = f; return *this; }
	DecisionTree& SetCriterion(DecisionTreeCriterion c) { criterion = c; return *this; }
	DecisionTree& SetClassifier(bool c = true) { is_classifier = c; return *this; }

	void Fit(const Vector<Vector<double>>& X, const Vector<double>& y);
	double Predict(const Vector<double>& x) const;
	Vector<double> Predict(const Vector<Vector<double>>& X) const;

	const Vector<double>& GetFeatureImportances() const { return feature_importances; }
	int GetNodeCount() const { return nodes.GetCount(); }
};

}

#endif
