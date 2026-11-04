#include "DecisionTree.h"
#include <cmath>
#include <algorithm>

namespace Upp {

double DecisionTree::CalculateImpurity(const Vector<double>& y, const Vector<int>& sample_indices) const
{
	int count = sample_indices.GetCount();
	if(count == 0) return 0.0;

	if(!is_classifier || criterion == CRITERION_MSE) {
		// Variance / MSE
		double mean = 0.0;
		for(int idx : sample_indices) mean += y[idx];
		mean /= count;

		double variance = 0.0;
		for(int idx : sample_indices) {
			double diff = y[idx] - mean;
			variance += diff * diff;
		}
		return variance / count;
	}

	// Classification: Gini or Entropy
	VectorMap<int, int> counts;
	for(int idx : sample_indices) {
		int cls = (int)y[idx];
		counts.GetAdd(cls, 0)++;
	}

	if(criterion == CRITERION_ENTROPY) {
		double entropy = 0.0;
		for(int i = 0; i < counts.GetCount(); ++i) {
			double p = (double)counts[i] / count;
			if(p > 1e-12) entropy -= p * log2(p);
		}
		return entropy;
	}

	// Gini impurity: 1 - sum(p_i^2)
	double sum_sq = 0.0;
	for(int i = 0; i < counts.GetCount(); ++i) {
		double p = (double)counts[i] / count;
		sum_sq += p * p;
	}
	return 1.0 - sum_sq;
}

void DecisionTree::FindBestSplit(const Vector<Vector<double>>& X, const Vector<double>& y,
                                 const Vector<int>& sample_indices,
                                 int& best_feature, double& best_threshold, double& best_gain,
                                 Vector<int>& left_indices, Vector<int>& right_indices)
{
	best_feature = -1;
	best_threshold = 0.0;
	best_gain = -1.0;
	left_indices.Clear();
	right_indices.Clear();

	int total_samples = sample_indices.GetCount();
	if(total_samples < min_samples_split) return;

	double current_impurity = CalculateImpurity(y, sample_indices);

	// Determine feature subset to evaluate
	Vector<int> feature_candidates;
	for(int f = 0; f < num_features; ++f) feature_candidates.Add(f);

	if(max_features > 0 && max_features < num_features) {
		// Random sub-sample of features (used by Random Forest)
		for(int i = 0; i < feature_candidates.GetCount(); ++i) {
			int swap_with = i + rand() % (feature_candidates.GetCount() - i);
			int tmp = feature_candidates[i];
			feature_candidates[i] = feature_candidates[swap_with];
			feature_candidates[swap_with] = tmp;
		}
		feature_candidates.Trim(max_features);
	}

	struct ValIdx {
		double val;
		int idx;
		bool operator<(const ValIdx& o) const { return val < o.val; }
	};

	Vector<int> best_left, best_right;

	for(int f : feature_candidates) {
		Vector<ValIdx> sorted_vals;
		sorted_vals.SetCount(total_samples);
		for(int i = 0; i < total_samples; ++i) {
			sorted_vals[i].val = X[sample_indices[i]][f];
			sorted_vals[i].idx = sample_indices[i];
		}
		std::sort(sorted_vals.Begin(), sorted_vals.End());

		// Test split points between adjacent distinct values
		for(int i = 0; i < total_samples - 1; ++i) {
			if(sorted_vals[i].val == sorted_vals[i + 1].val) continue;

			double threshold = (sorted_vals[i].val + sorted_vals[i + 1].val) * 0.5;

			Vector<int> left, right;
			for(int j = 0; j < total_samples; ++j) {
				if(sorted_vals[j].val <= threshold) {
					left.Add(sorted_vals[j].idx);
				} else {
					right.Add(sorted_vals[j].idx);
				}
			}

			if(left.GetCount() < min_samples_leaf || right.GetCount() < min_samples_leaf) {
				continue;
			}

			double left_imp = CalculateImpurity(y, left);
			double right_imp = CalculateImpurity(y, right);
			double split_imp = ((double)left.GetCount() / total_samples) * left_imp
			                 + ((double)right.GetCount() / total_samples) * right_imp;

			double gain = current_impurity - split_imp;
			if(gain > best_gain) {
				best_gain = gain;
				best_feature = f;
				best_threshold = threshold;
				best_left = pick(left);
				best_right = pick(right);
			}
		}
	}

	left_indices = pick(best_left);
	right_indices = pick(best_right);
}

int DecisionTree::BuildTree(const Vector<Vector<double>>& X, const Vector<double>& y,
                            const Vector<int>& sample_indices, int current_depth)
{
	int node_idx = nodes.GetCount();
	Node& node = nodes.Add();
	node.samples = sample_indices.GetCount();
	node.impurity = CalculateImpurity(y, sample_indices);

	// Compute node prediction value
	if(!is_classifier) {
		double sum = 0.0;
		for(int idx : sample_indices) sum += y[idx];
		node.value = sample_indices.IsEmpty() ? 0.0 : (sum / sample_indices.GetCount());
	} else {
		VectorMap<int, int> counts;
		for(int idx : sample_indices) {
			counts.GetAdd((int)y[idx], 0)++;
		}
		int max_cls = 0;
		int max_count = -1;
		for(int i = 0; i < counts.GetCount(); ++i) {
			if(counts[i] > max_count) {
				max_count = counts[i];
				max_cls = counts.GetKey(i);
			}
		}
		node.value = max_cls;
	}

	// Check stopping criteria
	if(current_depth >= max_depth || node.impurity <= 1e-7 || sample_indices.GetCount() < min_samples_split) {
		node.is_leaf = true;
		return node_idx;
	}

	int best_feature = -1;
	double best_threshold = 0.0;
	double best_gain = 0.0;
	Vector<int> left_indices, right_indices;

	FindBestSplit(X, y, sample_indices, best_feature, best_threshold, best_gain, left_indices, right_indices);

	if(best_feature < 0 || best_gain <= 1e-7) {
		nodes[node_idx].is_leaf = true;
		return node_idx;
	}

	// Record feature importance gain
	feature_importances[best_feature] += best_gain * sample_indices.GetCount();

	nodes[node_idx].is_leaf = false;
	nodes[node_idx].feature_idx = best_feature;
	nodes[node_idx].threshold = best_threshold;

	int left_child = BuildTree(X, y, left_indices, current_depth + 1);
	int right_child = BuildTree(X, y, right_indices, current_depth + 1);

	nodes[node_idx].left_child = left_child;
	nodes[node_idx].right_child = right_child;

	return node_idx;
}

void DecisionTree::Fit(const Vector<Vector<double>>& X, const Vector<double>& y)
{
	ASSERT(!X.IsEmpty() && X.GetCount() == y.GetCount());
	num_features = X[0].GetCount();
	nodes.Clear();
	feature_importances.SetCount(num_features, 0.0);

	Vector<int> all_indices;
	all_indices.SetCount(X.GetCount());
	for(int i = 0; i < X.GetCount(); ++i) all_indices[i] = i;

	BuildTree(X, y, all_indices, 0);

	// Normalize feature importances
	double sum = 0.0;
	for(double f : feature_importances) sum += f;
	if(sum > 1e-9) {
		for(double& f : feature_importances) f /= sum;
	}
}

double DecisionTree::Predict(const Vector<double>& x) const
{
	ASSERT(!nodes.IsEmpty());
	int curr = 0;
	while(!nodes[curr].is_leaf) {
		int f = nodes[curr].feature_idx;
		if(x[f] <= nodes[curr].threshold) {
			curr = nodes[curr].left_child;
		} else {
			curr = nodes[curr].right_child;
		}
	}
	return nodes[curr].value;
}

Vector<double> DecisionTree::Predict(const Vector<Vector<double>>& X) const
{
	Vector<double> preds;
	preds.SetCount(X.GetCount());
	for(int i = 0; i < X.GetCount(); ++i) {
		preds[i] = Predict(X[i]);
	}
	return preds;
}

}
