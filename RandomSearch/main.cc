/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * 
 * @author Javier Almenara Herrera
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include "./src/ProblemInstance.h"
#include "./src/RandomSolver.h"

/**
 * Función principal.
 */
int main(int argc, char* argv[]) {
  
  if (argc != 4) {
    std::cerr << "Uso: " << argv[0] << " <archivo.json> <archivo_salida.json> <duracion>" << std::endl;
    return 1;
  }

  // Abrir el archivo JSON
  std::string filename = argv[1];
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "No se pudo abrir el archivo JSON." << std::endl;
    return 1;
  }

  std::string outputFile = argv[2];
  int duration = std::stoi(argv[3]);
 
  // Metaheurística
  /// Poner la hora en formato legible HH::MM::SS
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time = *std::localtime(&now_time);
  std::cout << "Hora de inicio: " << std::put_time(&local_time, "%H:%M:%S") << std::endl;

  ProblemInstance problem;
  problem.loadFromJSON(filename);

  // Generación de soluciones
  auto start = std::chrono::high_resolution_clock::now();
  int bestSoftConstraints = std::numeric_limits<int>::max();
  while (std::chrono::high_resolution_clock::now() - start < std::chrono::minutes(duration)) {
    try {
      ProblemInstance problem_aux;
      problem_aux = problem;
      RandomSolver solver(problem_aux);
      Solution solution = solver.generateSolution();
      if (solution.total_soft_constraints < bestSoftConstraints) {
        bestSoftConstraints = solution.total_soft_constraints;
        solution.exportToJSON(outputFile);
      }
    } catch (const std::exception& e) {
      continue;
    }
  }
  // Hora de finalización
  now = std::chrono::system_clock::now();
  now_time = std::chrono::system_clock::to_time_t(now);
  local_time = *std::localtime(&now_time);
  std::cout << "Hora de finalización: " << std::put_time(&local_time, "%H:%M:%S") << std::endl;

  return 0;
}
