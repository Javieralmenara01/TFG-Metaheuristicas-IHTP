/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 * 
 * @author Javier Almenara Herrera
 * @brief Declaración de clase que representa una búsqueda aleatoria para el problema de asignación hospitalaria IHTP.
 * @file RandomSolver.h
 */

#ifndef SOLUTION_H
#define SOLUTION_H

#include "ProblemInstance.h" 
#include "Structs.h"       
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <random>

/**
 * Template para buscar un elemento por su id en un vector.
 * @param items Vector de elementos.
 * @param id Identificador del elemento a buscar.
 * @return Referencia al elemento encontrado.
 */
template <typename T> T& findById(std::vector<T>& items, const std::string& id) {
  auto it = std::find_if(items.begin(), items.end(), [&id](const T& item) {
    return item.id == id;
  });
  if (it == items.end()) {
    throw std::runtime_error("No se encontró el objeto con id: " + id);
  }
  return *it;
}

class RandomSolver {
 private:
  ProblemInstance& problem; // Referencia a la instancia del problema
  Solution solution;        // Solución generada
  std::default_random_engine rng; // Generador de números pseudoaleatorios

  OperatingTheater& findTheaterById(const std::string& id) { return findById(problem.theaters, id); }
  Surgeon& findSurgeonById(const std::string& id) { return findById(problem.surgeons, id); }
  Room& findRoomById(const std::string& id) { return findById(problem.rooms, id); }
  Nurse& findNurseById(const std::string& id) { return findById(problem.nurses, id); }
  Patient& findPatientById(const std::string& id) { return findById(problem.patients, id); }
  
  // Métodos de asignación
  void assignNursesToRooms();
  void assignMandatoryPatients();
  void assignOptionalPatients();
  void calculateSoftConstraints();

  // Métodos de cálculo de restricciones blandas
  int calculateAgeGroupPenalty(); // Cálculo de S1
  int calculateMinimumSkillPenalty(); // Cálculo de S2
  int calculateContinuityOfCarePenalty(); // Cálculo de S3
  int calculateMaximumWorkloadPenalty(); // Cálculo de S4
  int calculateOpenOperatingTheatersPenalty(); // Cálculo de S5
  int calculateSurgeonTransferPenalty(); // Cálculo de S6
  int calculateAdmissionDelayPenalty(); // Cálculo de S7
  int calculateUnscheduledOptionalPatientsPenalty(); // Cálculo de S8

  // Métodos de asignación de pacientes a quirófanos y habitaciones
  void assignmentPatientsToRoomsAndTheaters();
 public:
  RandomSolver(ProblemInstance& problemInstance);
  Solution generateSolution(); // Genera una solución aleatoria
  void exportSolution(const std::string& filename);
};

#endif