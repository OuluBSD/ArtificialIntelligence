#include "RandomForest.h"
#include <cmath>

namespace Upp {

void RandomForest::Fit(const Vector<Vector<double>>& X, const Vector<double>& y)
{
	ASSERT(!X.IsEmpty() && X.GetCount() == y.GetCount());
	num_features = X[0].GetCount();
	trees.Clear();
	feature_importances.SetCount(num_features, 0.0);

	int m_features = max_features;
	if(m_features <= 0) {
		m_features = max(1, (int)sqrt((double)num_features));
	}

	int n_samples = X.GetCount();

	for(int t = 0; t < n_estimators; ++t) {
		// Bootstrap sampling with replacement
		Vector<Vector<double>> X_boot;
		Vector<double> y_boot;
		X_boot.SetCount(n_samples);
		y_boot.SetCount(n_samples);

		for(int i = 0; i < n_samples; ++i) {
			int pick_idx = rand() % n_samples;
			X_boot[i] <<= X[pick_idx];
			y_boot[i] = y[pick_idx];
		}

		DecisionTree& tree = trees.Add();
		tree.SetMaxDepth(max_depth)
		    .SetMinSamplesSplit(min_samples_split)
		    .SetMinSamplesLeaf(min_samples_leaf)
		    .SetMaxFeatures(m_features)
		    .SetCriterion(criterion)
		    .SetClassifier(is_classifier);

		tree.Fit(X_boot, y_boot);

		// Accumulate feature importances
		const Vector<double>& tree_imp = tree.GetFeatureImportances();
		if(tree_imp.GetCount() == num_features) {
			for(int f = 0; f < num_features; ++f) {
				feature_importances[f] += tree_imp[f];
			}
		}
	}

	// Normalize feature importances
	double sum = 0.0;
	for(double f : feature_importances) sum += f;
	if(sum > 1e-9) {
		for(double& f : feature_importances) f /= sum;
	}
}

double RandomForest::Predict(const Vector<double>& x) const
{
	ASSERT(!trees.IsEmpty());

	if(!is_classifier) {
		double sum = 0.0;
		for(int i = 0; i < trees.GetCount(); ++i) {
			sum += trees[i].Predict(x);
		}
		return sum / trees.GetCount();
	}

	// Majority vote
	VectorMap<int, int> votes;
	for(int i = 0; i < trees.GetCount(); ++i) {
		int cls = (int)trees[i].Predict(x);
		votes.GetAdd(cls, 0)++;
	}

	int best_cls = 0;
	int max_votes = -1;
	for(int i = 0; i < votes.GetCount(); ++i) {
		if(votes[i] > max_votes) {
			max_votes = votes[i];
			best_cls = votes.GetKey(i);
		}
	}
	return best_cls;
}

Vector<double> RandomForest::Predict(const Vector<Vector<double>>& X) const
{
	Vector<double> preds;
	preds.SetCount(X.GetCount());
	for(int i = 0; i < X.GetCount(); ++i) {
		preds[i] = Predict(X[i]);
	}
	return preds;
}

VectorMap<int, double> RandomForest::PredictProba(const Vector<double>& x) const
{
	VectorMap<int, double> proba;
	if(trees.IsEmpty()) return proba;

	for(int i = 0; i < trees.GetCount(); ++i) {
		int cls = (int)trees[i].Predict(x);
		proba.GetAdd(cls, 0.0) += 1.0;
	}

	double total = trees.GetCount();
	for(int i = 0; i < proba.GetCount(); ++i) {
		proba[i] /= total;
	}
	return proba;
}

}
