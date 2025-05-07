/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 *
 * @author Javier Almenara Herrera
 * @brief Declaración de clase que representa una solución al problema de asignación hospitalaria.
 * @file MetaHeuristicSolver.h
 */

#ifndef METAHEURISTICSOLVER_H
#define METAHEURISTICSOLVER_H

#include "ProblemInstance.h"
#include "Structs.h"
#include <vector>
#include <random>
#include <thread>
#include <mutex>

class MetaHeuristicSolver {
 public:
  MetaHeuristicSolver(ProblemInstance &problemInstance);
  std::tuple<std::vector<EncodedPatientSolution>, int> geneticAlgorithm(const int populationSize, const int eliteCount, const int crossoverRate, const int duration, std::string outputFile);
  void evaluacion();
  
 private:
  ProblemInstance& problem;
  std::mt19937 rng;
  Solution solution;

  bool isMandatory(const std::string &patient_id);
  std::vector<EncodedPatientSolution> pmxCrossover(const std::vector<EncodedPatientSolution>& parent1, const std::vector<EncodedPatientSolution>& parent2);
  std::vector<EncodedPatientSolution> applyPMX(const std::vector<EncodedPatientSolution>& parent1, const std::vector<EncodedPatientSolution>& parent2);
  void mutate(std::vector<EncodedPatientSolution>& individual);
  std::pair<std::vector<EncodedPatientSolution>, std::vector<EncodedPatientSolution>> selectParents(const std::vector<std::vector<EncodedPatientSolution>>& population, const std::vector<std::pair<int, int>>& fitnesses);
  std::vector<std::vector<EncodedPatientSolution>> generateInitialPopulation(int populationSize);
  int countIdenticalSolutions(const std::vector<std::pair<int, int>>& fitnesses);
};

#endif
