/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 *
 * @author Javier Almenara Herrera
 * @brief Implementación de clase que representa una solución al problema de asignación hospitalaria.
 * @file MetaHeuristicSolver.cc
 */

#include "MetaHeuristicSolver.h"
#include "Solver.h"
#include <algorithm>
#include <stdexcept>
#include <chrono>
#include <vector>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>

/**
 * Constructor de la clase MetaHeuristicSolver.
 * 
 * @param problemInstance Instancia del problema a resolver.
 */
MetaHeuristicSolver::MetaHeuristicSolver(ProblemInstance &problemInstance)
    : problem(problemInstance), rng(std::random_device{}()) {}

/**
 * Comprueba si un paciente es obligatorio.
 * 
 * @param patient Paciente a comprobar.
 * @return true si el paciente es obligatorio, false en caso contrario.
 */
bool MetaHeuristicSolver::isMandatory(const std::string &patient_id) {
  auto patientIt = std::find_if(problem.patients.begin(), problem.patients.end(), 
                              [&](const Patient& p) { return p.id == patient_id; });
  if (patientIt == problem.patients.end()) {
    throw std::runtime_error("Paciente codificado " + patient_id + " no encontrado.");
  }
  return patientIt->mandatory;
}

/**
 * Genera una población inicial de soluciones codificadas.
 * 
 * @param populationSize Tamaño de la población a generar.
 * @return Población inicial de soluciones codificadas.
 */
std::vector<std::vector<EncodedPatientSolution>> MetaHeuristicSolver::generateInitialPopulation(int populationSize) {
  std::vector<std::vector<EncodedPatientSolution>> population;
  for (int i = 0; i < populationSize; ++i) {
    std::vector<EncodedPatientSolution> encodedSolution;

    /// Alterar el orden de los pacientes para generar una solución aleatoria guardandolo en un vector.
    std::shuffle(problem.patients.begin(), problem.patients.end(), rng);

    for (const auto &patient : problem.patients) {
      // Seleccionar un día aleatorio para el paciente entre sus días disponibles.
      /// A) Si es un paciente obligatorio, seleccionar un día aleatorio entre surgery_release_day y surgery_due_day.
      /// B) Si no es un paciente obligatorio, seleccionar un día aleatorio entre surgery_release_day y el último día disponible.
      int day = patient.mandatory
                    ? std::uniform_int_distribution<int>(patient.surgery_release_day, patient.surgery_due_day)(rng)
                    : std::uniform_int_distribution<int>(patient.surgery_release_day, problem.days - 1)(rng);
      
      // Seleccionar una sala aleatoria para el paciente fuera de las lista de incompatible_rooms.
      std::vector<std::string> availableRooms;
      for (const auto &room : problem.rooms) {
        if (std::find(patient.incompatible_room_ids.begin(), patient.incompatible_room_ids.end(), room.id) == patient.incompatible_room_ids.end()) {
          availableRooms.push_back(room.id);
        }
      }
      std::string room = availableRooms[std::uniform_int_distribution<int>(0, availableRooms.size() - 1)(rng)];

      encodedSolution.push_back({ patient.id, day, room });
    }
    population.push_back(encodedSolution);
  }
  return population;
}

/**
 * Algoritmo genético para resolver el problema de asignación hospitalaria,
 * con evaluación paralela (4 hilos), límite de tiempo de 10 minutos
 * y exportación de la mejor solución al final.
 *
 * @param populationSize Tamaño de la población.
 * @return Tupla con la mejor solución encontrada y su fitness de penalización blanda.
 */
std::tuple<std::vector<EncodedPatientSolution>, int> MetaHeuristicSolver::geneticAlgorithm(const int populationSize, const int eliteCount, const int crossoverRate, const int maxDuration, std::string outputFile) {
  // Parámetros del algoritmo
  const int maxThreads = 4;
  const auto maxDuration_c = std::chrono::minutes(maxDuration);
  auto startTime = std::chrono::steady_clock::now();
  auto endTime   = startTime + maxDuration_c;
  int generation = 0;

  // Generar población inicial
  auto population = generateInitialPopulation(populationSize);

  // Fitness de cada individuo y mejor solución global
  std::vector<std::pair<int,int>> fitnesses(populationSize);
  std::vector<EncodedPatientSolution> bestSolution;
  std::pair<int,int> bestFitness = {
    std::numeric_limits<int>::max(),
    std::numeric_limits<int>::max()
  };
  std::mutex bestMutex;

  // Función para evaluar un rango de individuos
  auto evalRange = [&](size_t start, size_t end) {
    for (size_t i = start; i < end; ++i) {
      Solver solver(problem);
      auto [hard, soft] = solver.solve(population[i]);  // ← una sola llamada

      // Actualizar fitness y posible mejor global
      std::lock_guard<std::mutex> lock(bestMutex);
      fitnesses[i] = {hard, soft};
      if (hard < bestFitness.first ||
         (hard == bestFitness.first && soft < bestFitness.second)) {
        bestFitness = {hard, soft};
        bestSolution = population[i];
      }
    }
  };

  // Evaluación inicial en paralelo
  {
    std::vector<std::thread> threads;
    size_t chunk = (populationSize + maxThreads - 1) / maxThreads;
    for (int t = 0; t < maxThreads; ++t) {
      size_t s = t * chunk;
      size_t e = std::min<size_t>(populationSize, s + chunk);
      if (s < e)
        threads.emplace_back(evalRange, s, e);
    }
    for (auto &th : threads) th.join();
  }

  // Bucle principal con límite de tiempo
  while (std::chrono::steady_clock::now() < endTime) {
    std::cout << "Generación " << generation++
              << " - Mejor fitness: Hard=" << bestFitness.first
              << ", Soft=" << bestFitness.second << std::endl;

    // --- Elitismo y nueva población
    std::vector<size_t> idx(populationSize);
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(),
      [&](size_t a, size_t b) {
        return fitnesses[a] < fitnesses[b];
      });

    std::vector<std::vector<EncodedPatientSolution>> newPop;
    
    // Conservamos los elites
    for (int i = 0; i < eliteCount; ++i)
      newPop.push_back(population[idx[i]]);
    // Rellenar con cruces y mutaciones
    while ((int)newPop.size() < populationSize) {
      auto parents = selectParents(population, fitnesses);
      int option = std::uniform_int_distribution<int>(0, 10)(rng);
      if (option < crossoverRate) {
        auto child = pmxCrossover(parents.first, parents.second);
        mutate(child);
        newPop.push_back(std::move(child));
      } else { // 20% de probabilidad de mutación
        auto child = parents.first;
        mutate(child);
        newPop.push_back(std::move(child));
      }
    }
    population.swap(newPop);

    // Reevaluar fitness en paralelo
    {
      std::vector<std::thread> threads;
      size_t chunk = (populationSize + maxThreads - 1) / maxThreads;
      for (int t = 0; t < maxThreads; ++t) {
        size_t s = t * chunk;
        size_t e = std::min<size_t>(populationSize, s + chunk);
        if (s < e)
          threads.emplace_back(evalRange, s, e);
      }
      for (auto &th : threads) th.join();
    }
  }

  // Exportar la mejor solución encontrada
  {
    Solver finalSolver(problem);
    auto [hard, soft] = finalSolver.solve(bestSolution);  // una sola llamada
    finalSolver.exportSolution(outputFile);
  }
  std::cout << "Fin GA: HardV=" << bestFitness.first
            << ", SoftP=" << bestFitness.second << std::endl;

  return {bestSolution, bestFitness.second};
}

/**
 * Realiza el operador de cruce PMX asegurando que se apliquen restricciones
 * separadas a pacientes obligatorios y opcionales, evitando duplicados.
 */
std::vector<EncodedPatientSolution> MetaHeuristicSolver::pmxCrossover(const std::vector<EncodedPatientSolution>& parent1, const std::vector<EncodedPatientSolution>& parent2) {
  int size = parent1.size();
  std::vector<EncodedPatientSolution> offspring(size);

  std::uniform_int_distribution<int> dist(0, size - 1);
  int point1 = dist(rng);
  int point2 = dist(rng);
  if (point1 > point2) std::swap(point1, point2);

  std::unordered_map<std::string, EncodedPatientSolution> mappingP1toP2;
  std::unordered_map<std::string, EncodedPatientSolution> mappingP2toP1;

  // Copiar la sección intercambiada y establecer mapeo
  for (int i = point1; i <= point2; i++) {
    offspring[i] = parent2[i];
    mappingP1toP2[parent1[i].patient_id] = parent2[i];
    mappingP2toP1[parent2[i].patient_id] = parent1[i];
  }

  // Copiar los valores restantes del primer padre con corrección
  for (int i = 0; i < size; i++) {
    if (i >= point1 && i <= point2) continue;

    EncodedPatientSolution newSolution = parent1[i];
    while (mappingP2toP1.find(newSolution.patient_id) != mappingP2toP1.end()) {
      newSolution = mappingP2toP1[newSolution.patient_id];
    }
    
    offspring[i] = newSolution;
  }

  return offspring;
}

/**
 * Aplica el operador PMX sobre un conjunto de pacientes.
 * 
 * @param parent1 Primer padre.
 * @param parent2 Segundo padre.
 * @return Vector con la descendencia generada.
 */
std::vector<EncodedPatientSolution> MetaHeuristicSolver::applyPMX(const std::vector<EncodedPatientSolution>& parent1, const std::vector<EncodedPatientSolution>& parent2) {
  
  int size = parent1.size();
  if (size == 0) return {}; // Si la subpoblación está vacía, retornamos vacío

  std::vector<EncodedPatientSolution> offspring(size);

  // Selección aleatoria de dos puntos de cruce
  std::uniform_int_distribution<int> dist(0, size - 1);
  int point1 = dist(rng);
  int point2 = dist(rng);

  if (point1 > point2) std::swap(point1, point2);

  // Mapas para corrección de mapeo
  std::unordered_map<std::string, std::string> mappingP1toP2;
  std::unordered_map<std::string, std::string> mappingP2toP1;

  // Copiar la sección intercambiada y establecer el mapeo
  for (int i = point1; i <= point2; i++) {
    offspring[i] = parent2[i];  // Copia desde el segundo padre
    mappingP1toP2[parent1[i].patient_id] = parent2[i].patient_id;
    mappingP2toP1[parent2[i].patient_id] = parent1[i].patient_id;
  }

  // Copiar los valores restantes del primer padre con corrección
  for (int i = 0; i < size; i++) {
    if (i >= point1 && i <= point2) continue; // Saltar la sección intercambiada

    std::string patient_id = parent1[i].patient_id;
    while (mappingP2toP1.find(patient_id) != mappingP2toP1.end()) {
      patient_id = mappingP2toP1[patient_id]; // Aplicar corrección de mapeo
    }

    offspring[i] = parent1[i];
    offspring[i].patient_id = patient_id; // Corregir ID si es necesario
  }

  return offspring;
}

/**
 * Operador de mutación: selecciona un paciente aleatorio y modifica su día de admisión en ±1 día dentro del rango permitido.
 * Se ejecuta en un bucle hasta encontrar una mutación válida.
 * 
 * @param individual Individuo a mutar.
 */
void MetaHeuristicSolver::mutate(std::vector<EncodedPatientSolution>& individual) {
  
  if (individual.empty()) return; // No mutar si el individuo está vacío

  std::uniform_int_distribution<int> dist(0, individual.size() - 1);
  std::uniform_int_distribution<int> dayDist(-1, 1); // Cambio de -1 a +1 días

  bool mutated = false;
  while (!mutated) {
    int patientIndex = dist(rng);
    EncodedPatientSolution& selectedPatient = individual[patientIndex];

    // Buscar paciente real en la instancia del problema
    auto patientIt = std::find_if(problem.patients.begin(), problem.patients.end(),
                                  [&](const Patient& p) { return p.id == selectedPatient.patient_id; });

    if (patientIt != problem.patients.end()) {
      const Patient& realPatient = *patientIt;

      // Intentar modificar el día hasta que se encuentre un cambio válido
      for (int attempt = 0; attempt < 3; ++attempt) { // Máximo 3 intentos de mutación
        int delta = dayDist(rng);
        int newAdmissionDay = selectedPatient.admission_day + delta;

        // Si el paciente es obligatorio, el día debe estar dentro del rango de admisión obligatorio
        if (realPatient.mandatory) {
          if (newAdmissionDay >= realPatient.surgery_release_day && newAdmissionDay <= realPatient.surgery_due_day) {
            selectedPatient.admission_day = newAdmissionDay;
            mutated = true;
            break;
          }
        } else {
          if (newAdmissionDay >= realPatient.surgery_release_day && newAdmissionDay < problem.days) {
            selectedPatient.admission_day = newAdmissionDay;
            mutated = true;
            break;
          }
        }
        
      }
    }
  }
}

/**
 * Selecciona dos padres de la población mediante torneo.
 * 
 * @param population Población de soluciones.
 * @param fitnesses Fitness de cada solución.
 * @return Par de padres seleccionados.
 */
std::pair<std::vector<EncodedPatientSolution>, std::vector<EncodedPatientSolution>> MetaHeuristicSolver::selectParents(const std::vector<std::vector<EncodedPatientSolution>>& population, const std::vector<std::pair<int, int>>& fitnesses) {

  std::uniform_int_distribution<int> dist(0, population.size() - 1);

  auto tournamentSelect = [&]() -> int {
    const int tournamentSize = 3; // Tamaño del torneo (ajustable)
    int bestIndex = -1;
    std::pair<int, int> bestFitness = {std::numeric_limits<int>::max(), std::numeric_limits<int>::max()};

    for (int i = 0; i < tournamentSize; ++i) {
      int idx = dist(rng);

      // Verificar si el individuo actual es mejor que el mejor encontrado
      if (fitnesses[idx].first < bestFitness.first || 
         (fitnesses[idx].first == bestFitness.first && fitnesses[idx].second < bestFitness.second)) {
        bestFitness = fitnesses[idx];
        bestIndex = idx;
      }
    }

    return (bestIndex != -1) ? bestIndex : dist(rng);  // Si no encontró, elige al azar
  };

  int parent1 = tournamentSelect();
  int parent2;
  int attempts = 0;

  do {
    parent2 = tournamentSelect();
    attempts++;
    // Esto se podría comentar es complicado que de forma aleatoria se elija el mismo padre varias veces seguidas
    if (attempts > 100) { // Máximo intentos para evitar bucles infinitos
      break;
    }
  } while (parent2 == parent1);

  return {population[parent1], population[parent2]};
}

/**
 * Cuenta el número de soluciones idénticas en una población.
 * 
 * @param fitnesses Vector con los fitness de cada individuo.
 * @return Número de soluciones idénticas.
 */
int MetaHeuristicSolver::countIdenticalSolutions(const std::vector<std::pair<int, int>>& fitnesses) {
  if (fitnesses.empty()) return 0; // Si la población está vacía, no hay soluciones idénticas

  // 1. Encontrar la mejor solución basada en (first, second)
  std::pair<int, int> bestFitness = {std::numeric_limits<int>::max(), std::numeric_limits<int>::max()};

  for (const auto& pair : fitnesses) {
    if (pair.first < bestFitness.first || (pair.first == bestFitness.first && pair.second < bestFitness.second)) {
      bestFitness = pair;
    }
  }

  // 2. Contar cuántas veces aparece la mejor solución en la población
  int countBestSolutions = 0;
  for (const auto& pair : fitnesses) {
    if (pair == bestFitness) {  // Comparación directa de std::pair
      countBestSolutions++;
    }
  }

  return countBestSolutions;
}

/**
 * Evalúa la solución actual. Para detectar posibles errores en la implementación.
 */
void MetaHeuristicSolver::evaluacion() {
  std::vector<std::vector<EncodedPatientSolution>> population = generateInitialPopulation(1);

  // Calcula el tiempo medio entre las ejecuciones de los individuos de la población
  std::vector<double> times;
  for (auto &individual : population) {
    Solver solver(problem);
    auto start = std::chrono::steady_clock::now();
    solver.solve(individual);
    auto end = std::chrono::steady_clock::now();
    // times.push_back(std::chrono::duration<double>(end - start).count());
    solver.exportSolution("prueba.json");
  }
}