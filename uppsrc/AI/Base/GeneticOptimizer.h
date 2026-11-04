#ifndef _AI_Base_GeneticOptimizer_h_
#define _AI_Base_GeneticOptimizer_h_

#include <Core/Core.h>
#include <cstdlib>
#include <float.h>

namespace Upp {

template <class TChromosome>
class GeneticOptimizer {
public:
	using FitnessFn = Function<double(const TChromosome& ind)>;
	using MutateFn = Function<void(TChromosome& ind, double mutation_rate)>;
	using CrossoverFn = Function<TChromosome(const TChromosome& a, const TChromosome& b)>;

	struct Individual : Moveable<Individual> {
		TChromosome chromosome;
		double fitness = -DBL_MAX;

		Individual() {}
		Individual(const TChromosome& c) : chromosome(c) {}
	};

	int population_size = 50;
	int max_generations = 100;
	double mutation_rate = 0.05;
	double crossover_rate = 0.8;
	int tournament_size = 4;
	bool elitism = true;

	Vector<Individual> population;
	Individual best_overall;

	GeneticOptimizer() {}

	void Initialize(const Vector<TChromosome>& initial_population) {
		population.Clear();
		for(const auto& c : initial_population) {
			population.Add(Individual(c));
		}
	}

	Individual Evolve(FitnessFn evaluate, CrossoverFn crossover, MutateFn mutate) {
		ASSERT(!population.IsEmpty());

		// Evaluate initial population
		for(auto& ind : population) {
			ind.fitness = evaluate(ind.chromosome);
			if(ind.fitness > best_overall.fitness) {
				best_overall = ind;
			}
		}

		for(int gen = 0; gen < max_generations; ++gen) {
			Vector<Individual> next_pop;

			if(elitism && best_overall.fitness != -DBL_MAX) {
				next_pop.Add(best_overall);
			}

			while(next_pop.GetCount() < population_size) {
				// Tournament selection for parent A and B
				const Individual& parent_a = TournamentSelect();
				const Individual& parent_b = TournamentSelect();

				TChromosome child_c;
				double r = (double)rand() / RAND_MAX;
				if(r < crossover_rate) {
					child_c = crossover(parent_a.chromosome, parent_b.chromosome);
				} else {
					child_c = parent_a.chromosome;
				}

				mutate(child_c, mutation_rate);

				Individual child(child_c);
				child.fitness = evaluate(child.chromosome);
				if(child.fitness > best_overall.fitness) {
					best_overall = child;
				}
				next_pop.Add(child);
			}

			population = pick(next_pop);
		}

		return best_overall;
	}

private:
	const Individual& TournamentSelect() const {
		int best_i = rand() % population.GetCount();
		double best_fit = population[best_i].fitness;

		for(int i = 1; i < tournament_size; ++i) {
			int idx = rand() % population.GetCount();
			if(population[idx].fitness > best_fit) {
				best_fit = population[idx].fitness;
				best_i = idx;
			}
		}
		return population[best_i];
	}
};

}

#endif
