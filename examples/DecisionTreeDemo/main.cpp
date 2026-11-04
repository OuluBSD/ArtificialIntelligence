#include <AI/DecisionTree/DecisionTree.h>
#include <AI/DecisionTree/RandomForest.h>

using namespace Upp;

CONSOLE_APP_MAIN
{
	Cout() << "========================================================\n";
	Cout() << "  ArtificialIntelligence - DecisionTree & Forest Demo\n";
	Cout() << "========================================================\n\n";

	// Synthetic dataset: Predict if a loan applicant is approved (1) or rejected (0)
	// Features:
	//   [0]: Credit Score (normalized 0.0 - 1.0)
	//   [1]: Annual Income ($k)
	//   [2]: Debt-to-Income Ratio (0.0 - 1.0)
	//   [3]: Years of Employment

	Vector<Vector<double>> X_train;
	Vector<double> y_train;
	Vector<Vector<double>> X_test;
	Vector<double> y_test;

	auto generate_applicant = [](bool is_good) -> Vector<double> {
		Vector<double> f;
		if(is_good) {
			f.Add(0.65 + ((double)rand() / RAND_MAX) * 0.35); // Credit: 0.65 - 1.0
			f.Add(60.0 + ((double)rand() / RAND_MAX) * 90.0);  // Income: 60k - 150k
			f.Add(0.10 + ((double)rand() / RAND_MAX) * 0.25); // DTI: 0.10 - 0.35
			f.Add(3.0 + ((double)rand() / RAND_MAX) * 15.0);   // Emp: 3 - 18 yrs
		} else {
			f.Add(0.20 + ((double)rand() / RAND_MAX) * 0.45); // Credit: 0.20 - 0.65
			f.Add(20.0 + ((double)rand() / RAND_MAX) * 45.0);  // Income: 20k - 65k
			f.Add(0.35 + ((double)rand() / RAND_MAX) * 0.45); // DTI: 0.35 - 0.80
			f.Add(0.0 + ((double)rand() / RAND_MAX) * 3.0);    // Emp: 0 - 3 yrs
		}
		return f;
	};

	// 200 Training samples
	for(int i = 0; i < 100; ++i) {
		Vector<double> g = generate_applicant(true);
		X_train.Add(pick(g));
		y_train.Add(1.0);

		Vector<double> b = generate_applicant(false);
		X_train.Add(pick(b));
		y_train.Add(0.0);
	}

	// 50 Test samples
	for(int i = 0; i < 25; ++i) {
		Vector<double> g = generate_applicant(true);
		X_test.Add(pick(g));
		y_test.Add(1.0);

		Vector<double> b = generate_applicant(false);
		X_test.Add(pick(b));
		y_test.Add(0.0);
	}

	Cout() << "Dataset summary:\n";
	Cout() << "  Training samples: " << X_train.GetCount() << " (4 features)\n";
	Cout() << "  Testing samples:  " << X_test.GetCount() << "\n\n";

	// 1. Single Decision Tree
	Cout() << "--- Training CART Decision Tree ---\n";
	DecisionTree dt;
	dt.SetMaxDepth(4).SetMinSamplesSplit(4).SetClassifier(true);
	dt.Fit(X_train, y_train);

	int dt_correct = 0;
	for(int i = 0; i < X_test.GetCount(); ++i) {
		if(dt.Predict(X_test[i]) == y_test[i]) dt_correct++;
	}
	double dt_acc = (double)dt_correct / X_test.GetCount();
	Cout() << "  Decision Tree Test Accuracy: " << Format("%.1f%%", dt_acc * 100)
	       << " (" << dt_correct << "/" << X_test.GetCount() << ")\n\n";

	// 2. Random Forest Ensemble
	Cout() << "--- Training Random Forest Ensemble (25 Trees) ---\n";
	RandomForest rf;
	rf.SetEstimators(25).SetMaxDepth(5).SetClassifier(true);
	rf.Fit(X_train, y_train);

	int rf_correct = 0;
	for(int i = 0; i < X_test.GetCount(); ++i) {
		if(rf.Predict(X_test[i]) == y_test[i]) rf_correct++;
	}
	double rf_acc = (double)rf_correct / X_test.GetCount();
	Cout() << "  Random Forest Test Accuracy: " << Format("%.1f%%", rf_acc * 100)
	       << " (" << rf_correct << "/" << X_test.GetCount() << ")\n\n";

	// 3. Inspect Feature Importances
	const Vector<double>& importances = rf.GetFeatureImportances();
	const char* feature_names[] = { "Credit Score", "Annual Income", "Debt-To-Income", "Years Employed" };
	Cout() << "Feature Importances (Gini importance across ensemble):\n";
	for(int i = 0; i < importances.GetCount() && i < 4; ++i) {
		Cout() << "  [" << (i + 1) << "] " << feature_names[i]
		       << ": " << Format("%.2f%%", importances[i] * 100.0) << "\n";
	}

	Cout() << "\nDemo completed successfully.\n";
}
