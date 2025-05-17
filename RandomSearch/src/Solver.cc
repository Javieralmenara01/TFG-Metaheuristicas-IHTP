/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 *
 * @author Javier Almenara Herrera
 * @brief Implementación de clase que representa una solución al problema de asignación hospitalaria.
 * @file Solver.cc
 */

#include "Solver.h"
#include <algorithm>
#include <stdexcept>
#include <chrono>
#include <vector>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

/**
 * Constructor de la clase Solver.
 * @param problemInstance Instancia del problema a resolver.
 */
Solver::Solver(ProblemInstance problemInstance)
    : problem(problemInstance), originalProblem(problemInstance), rng(std::random_device{}()) {}

/**
 * Inicializa los estados dinámicos de la solución.
 * Se inicializan las habitaciones, quirófanos y enfermeras en la solución.
 */
void Solver::initializeDynamicStates() {
  
  solution.roomStates.clear();
  solution.theaterStates.clear();
  solution.nurses.clear();

  // Inicializar estructuras dinámicas
  for (const auto &room : problem.rooms) { /// Inicializar las habitaciones
    RoomState roomState;
    roomState.room_info = room;
    for (int day = 0; day < problem.days; ++day) { /// Inicializar los días
      roomState.capacity_per_day[day] = room.capacity;
    }
    for (const auto &occupant : problem.occupants) { /// Inicializar los ocupantes
      if (occupant.room_id == room.id) { // Asignar ocupantes a la habitación
        for (int day = 0; day < occupant.length_of_stay && day < problem.days; ++day) { // Asignar ocupantes cada día de esa habitación
          roomState.occupants_per_day[day].push_back(occupant);
          roomState.capacity_per_day[day]--;
        }
      }
    }
    solution.roomStates.push_back(roomState);
  }

  for (const auto &theater : problem.theaters) {
    OperatingTheaterState theaterState;
    theaterState.theater_info = theater;
    for (int day = 0; day < problem.days; ++day) {
      theaterState.availability_per_day[day] = theater.availability[day];
    }
    solution.theaterStates.push_back(theaterState);
  }

  for (const auto &nurse : problem.nurses) {
    NurseAssignment nurseAssignment = {nurse.id, {}};
    // Inicializar todos los turnos de trabajo que vienen en la instancia del problema.
    for (const auto &shift : nurse.working_shifts) {
      NurseAssignment::ShiftAssignment shiftAssignment = {shift.day, shift.shift, {}};
      nurseAssignment.assignments.push_back(shiftAssignment);
    }
    solution.nurses.push_back(nurseAssignment);
  }
}

/**
 * Comprueba la disponibilidad del cirujano para el paciente en un día concreto.
 * @param patient Paciente a programar
 * @param day Día de admisión
 * @return true si el cirujano tiene disponibilidad, false en caso contrario.
 */
bool Solver::checkSurgeonAvailability(const Patient &patient, int day) {
  // Buscar el cirujano y comprobar su capacidad en ese día.
  auto surgeonIt = std::find_if(problem.surgeons.begin(), problem.surgeons.end(),
                                [&patient](const Surgeon &s) { return s.id == patient.surgeon_id; });
  if (surgeonIt == problem.surgeons.end())
    throw std::runtime_error("Cirujano no encontrado para el paciente " + patient.id);
  
  return surgeonIt->max_surgery_time[day] >= patient.surgery_duration;
}

/**
 * Busca que exista un quirófano disponible para el paciente en un día concreto.
 * @param patient Paciente a programar
 * @param day Día de admisión
 * @return Índice del quirófano seleccionado.
 */
bool Solver::checkOperatingTheaterAvailability(const Patient &patient, int day) {
  for (size_t i = 0; i < solution.theaterStates.size(); ++i) {
    if (solution.theaterStates[i].availability_per_day[day] >= patient.surgery_duration) {
      return true;
    }
  }
  return false;
}

/**
 * Verifica si la habitación asignada cumple los criterios de capacidad y compatibilidad de género
 * para la estancia del paciente.
 * 
 * @param patient Paciente a asignar.
 * @param admission_day Día de admisión.
 * @param room_id Habitación sugerida.
 * @return true si la habitación es adecuada para toda la estancia del paciente, false en caso contrario.
 */
bool Solver::checkRoomAvailability(const Patient &patient, int admission_day, const std::string &room_id) {
  // Buscar la habitación en el estado dinámico.
  auto roomIt = std::find_if(solution.roomStates.begin(), solution.roomStates.end(),
                             [&room_id](const RoomState &roomState) {
                               return roomState.room_info.id == room_id;
                             });
  if (roomIt == solution.roomStates.end()) {
    throw std::runtime_error("Habitación '" + room_id + "' no encontrada en los estados dinámicos.");
  }

  RoomState &roomState = *roomIt;
  
  // Calcular el rango de días de la estancia: desde el día de admisión hasta el final de la estancia,
  // sin exceder el número total de días del problema.
  const int start_day = admission_day;
  const int end_day = std::min(admission_day + patient.length_of_stay, problem.days);

  // Recorrer cada día en el que el paciente estará en la habitación.
  for (int day = start_day; day < end_day; ++day) {
    // Verificar que exista capacidad disponible en la habitación para el día actual.
    auto capacityIt = roomState.capacity_per_day.find(day);
    if (capacityIt == roomState.capacity_per_day.end() || capacityIt->second <= 0) {
      // No hay capacidad en este día.
      return false;
    }

    // Comprobar que no se rompa la compatibilidad de género en los pacientes asignados.
    auto patientsIt = roomState.patients_per_day.find(day);
    if (patientsIt != roomState.patients_per_day.end()) {
      for (const auto &assignedPatient : patientsIt->second) {
        if (assignedPatient.gender != patient.gender) {
          return false;
        }
      }
    }

    // Comprobar que los ocupantes existentes sean compatibles en cuanto a género.
    auto occupantsIt = roomState.occupants_per_day.find(day);
    if (occupantsIt != roomState.occupants_per_day.end()) {
      for (const auto &occupant : occupantsIt->second) {
        if (occupant.gender != patient.gender) {
          return false;
        }
      }
    }
  }
  // Si para todos los días hay capacidad suficiente y la compatibilidad de género es correcta, se devuelve true.
  return true;
}

/**
 * Asigna un cirujano a un paciente en un día concreto.
 * @param patient Paciente a programar
 * @param day Día de admisión
 * @return true si la asignación es factible, false en caso contrario.
 */
void Solver::assignSurgeon(const Patient &patient, int day) {
  // Buscar el cirujano y comprobar su capacidad en ese día.
  auto surgeonIt = std::find_if(problem.surgeons.begin(), problem.surgeons.end(),
                                [&patient](const Surgeon &s) { return s.id == patient.surgeon_id; });
  if (surgeonIt == problem.surgeons.end())
    throw std::runtime_error("Cirujano no encontrado para el paciente " + patient.id);

  Surgeon &surgeon = *surgeonIt;
  surgeon.max_surgery_time[day] -= patient.surgery_duration;
}

/**
 * Asigna un quirófano a un paciente en un día concreto.
 * @param patient Paciente a programar.
 * @param day Día de admisión.
 * @return ID del quirófano seleccionado.
 */
std::string Solver::assignOperatingTheater(const Patient &patient, int day) {
  // Vector de punteros a quirófanos abiertos (ya con al menos un paciente asignado en el día)
  // y de quirófanos cerrados (sin pacientes asignados ese día), pero con capacidad.
  std::vector<OperatingTheaterState*> openTheaters;
  std::vector<OperatingTheaterState*> closedTheaters;

  // Recorrer todos los quirófanos disponibles en la solución.
  for (size_t i = 0; i < solution.theaterStates.size(); ++i) {
    OperatingTheaterState &theaterState = solution.theaterStates[i];
    // Comprobar que la capacidad disponible para el día es suficiente para la cirugía del paciente.
    if (theaterState.availability_per_day[day] >= patient.surgery_duration) {
      // Si el quirófano ya tiene asignado al menos un paciente en el día, se considera "abierto".
      if (theaterState.patients_per_day.find(day) != theaterState.patients_per_day.end() &&
          !theaterState.patients_per_day[day].empty()) {
        openTheaters.push_back(&theaterState);
      } else { 
        // De lo contrario, se considera "cerrado" (no usado aún ese día).
        closedTheaters.push_back(&theaterState);
      }
    }
  }

  // Si existen quirófanos abiertos disponibles, seleccionamos el que tenga la menor capacidad residual
  // (es decir, el que quede más "lleno" al asignarle la cirugía).
  if (!openTheaters.empty()) {
    OperatingTheaterState* bestTheater = openTheaters[0];
    int bestRemaining = bestTheater->availability_per_day[day] - patient.surgery_duration;
    for (auto *theater : openTheaters) {
      int remaining = theater->availability_per_day[day] - patient.surgery_duration;
      if (remaining < bestRemaining) {
        bestRemaining = remaining;
        bestTheater = theater;
      }
    }
    // Asignar el paciente al quirófano seleccionado.
    bestTheater->patients_per_day[day].push_back(patient);
    bestTheater->availability_per_day[day] -= patient.surgery_duration;
    return bestTheater->theater_info.id;
  }

  // Si no hay quirófanos abiertos disponibles, se selecciona entre los cerrados
  // el que tenga la mayor capacidad para "abrir" el quirófano.
  if (!closedTheaters.empty()) {
    OperatingTheaterState* bestTheater = closedTheaters[0];
    int bestCapacity = bestTheater->availability_per_day[day];
    for (auto *theater : closedTheaters) {
      int cap = theater->availability_per_day[day];
      if (cap > bestCapacity) {
        bestCapacity = cap;
        bestTheater = theater;
      }
    }
    // Asignar el paciente al quirófano seleccionado.
    bestTheater->patients_per_day[day].push_back(patient);
    bestTheater->availability_per_day[day] -= patient.surgery_duration;
    return bestTheater->theater_info.id;
  }

  // Si no se encuentra ningún quirófano con capacidad suficiente, se lanza una excepción.
  throw std::runtime_error("No hay quirófano disponible para el paciente " + patient.id + " en el día " + std::to_string(day));
}

/**
 * Asigna una habitación a un paciente a partir de un día concreto.
 * Para cada día de la estancia del paciente se actualiza la capacidad disponible
 * y se añade el paciente a la lista de pacientes asignados en ese día.
 *
 * @param patient Paciente a asignar.
 * @param day Día de admisión (inicio de la estancia).
 * @param room_id Identificador de la habitación a asignar.
 */
void Solver::assignRoom(const Patient &patient, int day, const std::string &room_id) {
  // Buscar la habitación en el estado dinámico usando el identificador.
  auto roomIt = std::find_if(solution.roomStates.begin(), solution.roomStates.end(),
                             [&room_id](const RoomState &roomState) {
                               return roomState.room_info.id == room_id;
                             });
  if (roomIt == solution.roomStates.end()) {
    throw std::runtime_error("Habitación '" + room_id + "' no encontrada en los estados dinámicos.");
  }
  RoomState &roomState = *roomIt;

  // Determinar el rango de días en que el paciente ocupará la habitación.
  // Se asume que el paciente se queda desde 'day' hasta 'day + patient.length_of_stay'
  // sin exceder el total de días del problema.
  const int start_day = day;
  const int end_day = std::min(day + patient.length_of_stay, problem.days);

  // Para cada día de la estancia, actualizar la capacidad y asignar al paciente.
  for (int current_day = start_day; current_day < end_day; ++current_day) {
    // Buscar la capacidad disponible para el día actual en el mapa.
    auto capacityIt = roomState.capacity_per_day.find(current_day);
    if (capacityIt == roomState.capacity_per_day.end() || capacityIt->second <= 0) {
      throw std::runtime_error("La habitación '" + room_id + "' no tiene capacidad disponible el día " +
                               std::to_string(current_day));
    }
    // Decrementar la capacidad disponible para el día.
    capacityIt->second--;

    // Asignar al paciente en la lista de pacientes para el día.
    roomState.patients_per_day[current_day].push_back(patient);
  }
}

/**
 * Intenta reparar la asignación de un paciente obligatorio modificando el día de admisión y la habitación asignada.
 *
 * @param patient El paciente obligatorio a reparar.
 * @param enc Referencia a la solución codificada correspondiente al paciente, cuyos parámetros se modificarán.
 * @return true si se encontró una asignación factible; false en caso contrario.
 */
bool Solver::repairMandatoryPatient(const Patient &patient, EncodedPatientSolution &enc) {
  // Recorremos todos los días válidos (desde surgery_release_day hasta surgery_due_day)
  for (int newDay = patient.surgery_release_day; newDay <= patient.surgery_due_day && newDay < problem.days; ++newDay) {
    // Recorremos todas las habitaciones disponibles en la instancia
    for (const auto &room : problem.rooms) {
      // Saltamos las habitaciones incompatibles para el paciente.
      if (std::find(patient.incompatible_room_ids.begin(), patient.incompatible_room_ids.end(), room.id) != patient.incompatible_room_ids.end()) {
        continue;
      }
      
      // Guardamos los valores actuales para poder revertir en caso de no lograr la reparación.
      int oldDay = enc.admission_day;
      std::string oldRoom = enc.room_id;
      
      // Asignamos los valores candidatos.
      enc.admission_day = newDay;
      enc.room_id = room.id;
      
      // Verificamos que se cumplan las restricciones de cirujano, quirófano y habitación.
      if ( checkSurgeonAvailability(patient, newDay) &&
           checkOperatingTheaterAvailability(patient, newDay) &&
           checkRoomAvailability(patient, newDay, room.id) ) {
        // std::cout << "Reparación exitosa para el paciente obligatorio " << patient.id << " con día " << newDay << " y habitación " << room.id << std::endl;
        return true;
      } else {
        // Si no cumple, revertimos y seguimos probando.
        enc.admission_day = oldDay;
        enc.room_id = oldRoom;
      }
    }
  }
  // Si se agotaron todas las posibilidades, la reparación falla.
  // std::cout << "No se pudo reparar la asignación para el paciente obligatorio " << patient.id << std::endl;
  return false;
}

/**
 * Intenta reparar la asignación de un paciente opcional modificando el día de admisión y la habitación asignada.
 * Se realizan varios intentos antes de abandonar.
 *
 * @param patient El paciente opcional a reparar.
 * @param enc Referencia a la solución codificada correspondiente al paciente, cuyos parámetros se modificarán.
 * @return true si se encontró una asignación factible; false en caso contrario.
 */
bool Solver::repairOptionalPatient(const Patient &patient, EncodedPatientSolution &enc) {
  const int MAX_ATTEMPTS = 5;
  int attempt = 0;
  
  while (attempt < MAX_ATTEMPTS) {
    // Recorremos todos los días válidos desde la fecha de liberación hasta el último día disponible
    for (int newDay = patient.surgery_release_day; newDay < problem.days; ++newDay) {
      for (const auto &room : problem.rooms) {
        // Saltamos las habitaciones incompatibles.
        if (std::find(patient.incompatible_room_ids.begin(), patient.incompatible_room_ids.end(), room.id) != patient.incompatible_room_ids.end()) {
          continue;
        }
        
        int oldDay = enc.admission_day;
        std::string oldRoom = enc.room_id;
        
        enc.admission_day = newDay;
        enc.room_id = room.id;
        
        if (checkSurgeonAvailability(patient, newDay) &&
            checkOperatingTheaterAvailability(patient, newDay) &&
            checkRoomAvailability(patient, newDay, room.id)) {
          // std::cout << "Reparación exitosa para el paciente opcional " << patient.id << " en intento " << attempt + 1 << " con día " << newDay << " y habitación " << room.id << std::endl;
          return true;
        } else {
          enc.admission_day = oldDay;
          enc.room_id = oldRoom;
        }
      }
    }
    attempt++;
  }
  
  // std::cout << "No se pudo reparar la asignación para el paciente opcional " << patient.id << " tras " << MAX_ATTEMPTS << " intentos." << std::endl;
  return false;
}

/**
 * Resuelve el problema de asignación hospitalaria a partir de una solución codificada.
 * @param encodedSolution Solución codificada inicial.
 */
std::pair<int,int> Solver::solve(std::vector<EncodedPatientSolution>& encodedSolution, const bool method) {
  // Reinicializa los estados dinámicos de la solución (habitaciones, quirófanos y enfermeras)
  initializeDynamicStates();
  
  // Particionar la población codificada en pacientes obligatorios y opcionales.
  std::vector<EncodedPatientSolution> mandatoryEnc, optionalEnc;
  for (const auto &enc : encodedSolution) {
    // Buscar el paciente correspondiente.
    auto patientIt = std::find_if(problem.patients.begin(), problem.patients.end(),
                                  [&enc](const Patient &p) { return p.id == enc.patient_id; });
    if (patientIt == problem.patients.end()) {
      throw std::runtime_error("Paciente codificado " + enc.patient_id + " no encontrado.");
    }
    if (patientIt->mandatory)
      mandatoryEnc.push_back(enc);
    else
      optionalEnc.push_back(enc);
  }

  int numMandatory = mandatoryEnc.size();
  int numMandatoryRepairs = 0;
  int numOptional = optionalEnc.size();
  int numOptionalRepairs = 0;
  
  // --- Procesar pacientes obligatorios ---
  for (auto &enc : mandatoryEnc) {
    
    auto patientIt = std::find_if(problem.patients.begin(), problem.patients.end(),
                                  [&enc](const Patient &p) { return p.id == enc.patient_id; });
    if (patientIt == problem.patients.end()) {
      throw std::runtime_error("Paciente codificado " + enc.patient_id + " no encontrado.");
    }
    Patient patient = *patientIt;
    int admissionDay = enc.admission_day;
    
    // Verificar disponibilidad
    if (!checkSurgeonAvailability(patient, admissionDay) ||
        !checkOperatingTheaterAvailability(patient, admissionDay) ||
        !checkRoomAvailability(patient, admissionDay, enc.room_id)) {
      
      // Intentar reparar para el paciente obligatorio.
      if (!repairMandatoryPatient(patient, enc)) {
        
        // std::cout << "Paciente obligatorio " << patient.id << " no pudo ser reparado. Solución infactible." << std::endl;
        continue;
      }
      numMandatoryRepairs++;
      // Actualizar el día de admisión tras la reparación.
      admissionDay = enc.admission_day;
    }
    
    // Realizar asignaciones (cirujano, quirófano y habitación)
    assignSurgeon(patient, admissionDay);
    std::string theater_id = assignOperatingTheater(patient, admissionDay);
    assignRoom(patient, admissionDay, enc.room_id);
    solution.patients.push_back({ patient.id, admissionDay, enc.room_id, theater_id });
  }
  
  // --- Procesar pacientes opcionales ---
  for (auto &enc : optionalEnc) {
    auto patientIt = std::find_if(problem.patients.begin(), problem.patients.end(),
                                  [&enc](const Patient &p) { return p.id == enc.patient_id; });
    if (patientIt == problem.patients.end()) {
      throw std::runtime_error("Paciente codificado " + enc.patient_id + " no encontrado.");
    }
    Patient patient = *patientIt;
    int admissionDay = enc.admission_day;
    
    // Verificar disponibilidad
    if (!checkSurgeonAvailability(patient, admissionDay) ||
        !checkOperatingTheaterAvailability(patient, admissionDay) ||
        !checkRoomAvailability(patient, admissionDay, enc.room_id)) {
      
      // Intentar reparar para el paciente opcional.
      if (!repairOptionalPatient(patient, enc)) {
        // std::cout << "Paciente opcional " << patient.id << " no pudo ser reparado. Se omite." << std::endl;
        continue;  // Omitir el paciente opcional si no se puede reparar.
      }

      numOptionalRepairs++;
      admissionDay = enc.admission_day;
    }
    
    // Realizar asignaciones para el paciente opcional reparado.
    assignSurgeon(patient, admissionDay);
    std::string theater_id = assignOperatingTheater(patient, admissionDay);
    assignRoom(patient, admissionDay, enc.room_id);
    solution.patients.push_back({ patient.id, admissionDay, enc.room_id, theater_id });
  }
  
  // Asignar enfermeras a las habitaciones y calcular penalizaciones.
  assignNursesToRooms(); 

  // **Calcular restricciones duras y blandas**
  int hardViolations = calculateHardConstraintViolations();
  int softPenalty = calculateSoftConstraints();

  return {hardViolations, softPenalty};
}

/**
 * Calcula todas las restricciones blandas y actualiza la solución.
 */
int Solver::calculateSoftConstraints() {
  solution.soft_constraints.clear();
  
  // Calcular cada restricción blanda individualmente y almacenarla
  solution.soft_constraints.push_back(calculateAgeGroupPenalty());
  solution.soft_constraints.push_back(calculateMinimumSkillPenalty());
  solution.soft_constraints.push_back(calculateContinuityOfCarePenalty());
  solution.soft_constraints.push_back(calculateMaximumWorkloadPenalty());
  solution.soft_constraints.push_back(calculateOpenOperatingTheatersPenalty());
  solution.soft_constraints.push_back(calculateSurgeonTransferPenalty());
  solution.soft_constraints.push_back(calculateAdmissionDelayPenalty());
  solution.soft_constraints.push_back(calculateUnscheduledOptionalPatientsPenalty());
  
  // Sumar todas las restricciones blandas para obtener el total
  solution.total_soft_constraints = std::accumulate(solution.soft_constraints.begin(), solution.soft_constraints.end(), 0);
  
  return solution.total_soft_constraints;
}

/**
 * Calcula la penalización por mezcla de grupos de edad en las habitaciones.
 * @return Penalización por mezcla de grupos de edad.
 */
int Solver::calculateAgeGroupPenalty() {
    int penalty = 0;

    for (const auto &roomState : solution.roomStates) {
        // Conjunto para almacenar todos los días en los que hay pacientes u ocupantes
        std::set<int> occupiedDays;
        
        // Añadir días donde hay pacientes
        for (const auto &[day, _] : roomState.patients_per_day) {
            occupiedDays.insert(day);
        }

        // Añadir días donde hay ocupantes
        for (const auto &[day, _] : roomState.occupants_per_day) {
            occupiedDays.insert(day);
        }

        // Iterar sobre todos los días en los que hay ocupación
        for (int day : occupiedDays) {
            std::set<int> ageIndices;

            // Incluir pacientes en el cálculo (si existen en este día)
            if (roomState.patients_per_day.count(day) > 0) {
                for (const auto &patient : roomState.patients_per_day.at(day)) {
                    auto it = std::find(problem.age_groups.begin(), problem.age_groups.end(), patient.age_group);
                    if (it != problem.age_groups.end()) {
                        ageIndices.insert(std::distance(problem.age_groups.begin(), it));
                    }
                }
            }

            // Incluir ocupantes en el cálculo (si existen en este día)
            if (roomState.occupants_per_day.count(day) > 0) {
                for (const auto &occupant : roomState.occupants_per_day.at(day)) {
                    auto it = std::find(problem.age_groups.begin(), problem.age_groups.end(), occupant.age_group);
                    if (it != problem.age_groups.end()) {
                        ageIndices.insert(std::distance(problem.age_groups.begin(), it));
                    }
                }
            }

            // Calcular la penalización si hay más de un grupo de edad
            if (ageIndices.size() > 1) {
                int maxAge = *std::max_element(ageIndices.begin(), ageIndices.end());
                int minAge = *std::min_element(ageIndices.begin(), ageIndices.end());
                penalty += (problem.weights.room_mixed_age * (maxAge - minAge)); // Diferencia máxima
            }
        }
    }

    return penalty;
}

/**
 * Calcula la penalización para aquellas enfermeras que no cumplen con el nivel de habilidad mínimo.
 * @return Penalización por habilidades mínimas.
 */
int Solver::calculateMinimumSkillPenalty() {
  int penalty = 0;

  // Crear índices para acceso rápido
  std::unordered_map<std::string, const Nurse *> nurseMap;
  for (const auto &nurse : problem.nurses) {
    nurseMap[nurse.id] = &nurse;
  }

  std::unordered_map<std::string, const RoomState *> roomMap;
  for (const auto &roomState : solution.roomStates) {
    roomMap[roomState.room_info.id] = &roomState;
  }

  std::unordered_map<std::string, const PatientAssignment *> patientMap;
  for (const auto &patientAssignment : solution.patients) {
    patientMap[patientAssignment.id] = &patientAssignment;
  }

  // Iterar sobre asignaciones de enfermeras
  for (const auto &nurseAssignment : solution.nurses) {
    // Encontrar la enfermera en el índice
    auto nurseIt = nurseMap.find(nurseAssignment.id);
    if (nurseIt == nurseMap.end()) {
      throw std::runtime_error("Enfermera no encontrada en los datos del problema.");
    }
    const auto &nurse = *(nurseIt->second);

    // Iterar sobre los turnos de la enfermera
    for (const auto &shiftAssignment : nurseAssignment.assignments) {
      int day = shiftAssignment.day;

      // Iterar sobre las habitaciones asignadas en este turno
      for (const auto &roomId : shiftAssignment.rooms) {
        // Buscar la habitación en el índice
        auto roomIt = roomMap.find(roomId);
        if (roomIt == roomMap.end()) {
          throw std::runtime_error("Habitación no encontrada en los estados dinámicos.");
        }
        const auto &roomState = *(roomIt->second);

        // Verificar los pacientes en la habitación en este día
        if (roomState.patients_per_day.count(day)) {
          for (const auto &patient : roomState.patients_per_day.at(day)) {
            // Buscar al paciente en el índice
            auto patientIt = patientMap.find(patient.id);
            if (patientIt == patientMap.end()) {
              throw std::runtime_error("Paciente no encontrado en la solución.");
            }
            const auto &patientAssignment = *(patientIt->second);

            // Calcular el índice del turno
            int index = (shiftAssignment.day - patientAssignment.admission_day);
            // std::cout << "\nIndex: " << index << std::endl;

            int shiftIndex = std::distance(problem.shift_types.begin(),
                                           std::find(problem.shift_types.begin(), problem.shift_types.end(), shiftAssignment.shift));
            // std::cout << "ShiftIndex: " << shiftIndex << std::endl;

            int skillIndex = (index * problem.shift_types.size()) + shiftIndex;
            // std::cout << "SkillIndex: " << skillIndex << std::endl;

            // Calcular la penalización
            if (skillIndex < patient.skill_level_required.size() &&
                patient.skill_level_required[skillIndex] > nurse.skill_level) {
              penalty += (patient.skill_level_required[skillIndex] - nurse.skill_level) * problem.weights.room_nurse_skill;
              // std::cout << "Paciente: " << patient.id << " Enfermera: " << nurse.id << " Día: " << day << " Turno: " << shiftAssignment.shift << " SkillIndex: " << skillIndex << " Penalización: " << (patient.skill_level_required[skillIndex] - nurse.skill_level * problem.weights.room_nurse_skill) << std::endl;
            }
          }
        }

        // Verificar los ocupantes en la habitación en este día
        if (roomState.occupants_per_day.count(day)) {
          for (const auto &occupant : roomState.occupants_per_day.at(day)) {
            int shiftIndex = std::distance(problem.shift_types.begin(),
                                           std::find(problem.shift_types.begin(), problem.shift_types.end(), shiftAssignment.shift));
            // std::cout << "\nShiftIndex: " << shiftIndex << std::endl;
            
            int skillIndex = (day * problem.shift_types.size()) + shiftIndex;
            // std::cout << "SkillIndex: " << skillIndex << std::endl;
  
            if (skillIndex < occupant.skill_level_required.size() &&
                occupant.skill_level_required[skillIndex] > nurse.skill_level) {
              penalty += (occupant.skill_level_required[skillIndex] - nurse.skill_level) * problem.weights.room_nurse_skill;
              // std::cout << "Ocupante: " << occupant.id << " Enfermera: " << nurse.id << " Día: " << day << " Turno: " << shiftAssignment.shift << " SkillIndex: " << skillIndex << " Penalización: " << (occupant.skill_level_required[skillIndex] - nurse.skill_level * problem.weights.room_nurse_skill) << std::endl;
            }
          }
        }
      }
    }
  }

  return penalty;
}

/**
 * Calcula la penalización por la continuidad de la atención.
 * @return Penalización por continuidad de la atención.
 */
int Solver::calculateContinuityOfCarePenalty() {
  int penalty = 0;

  for (const auto &occupantAssignment : problem.occupants) {
    // Buscar al ocupante en los datos del problema
    auto occupantIt = std::find_if(
      problem.occupants.begin(), problem.occupants.end(),
      [&occupantAssignment](const Occupant &occupant) {
        return occupant.id == occupantAssignment.id;
      });


    if (occupantIt == problem.occupants.end()) {
      throw std::runtime_error("Ocupante no encontrado en los datos del problema.");
    }

    const auto &occupant = *occupantIt;

    // Conjunto de enfermeras únicas asignadas al ocupante
    std::set<std::string> uniqueNurses;

    // Iterar por los días de la estancia del ocupante
    for (int day = 0; day < occupant.length_of_stay; ++day) {
      if (day >= problem.days)
        break;

      // Iterar por cada turno
      for (const auto &shift : problem.shift_types) {
        // Buscar enfermeras asignadas a la habitación del ocupante en este turno
        auto roomIt = std::find_if(solution.roomStates.begin(), solution.roomStates.end(),
                                   [&occupantAssignment](const RoomState &roomState)
                                   {
                                     return roomState.room_info.id == occupantAssignment.room_id;
                                   });

        if (roomIt == solution.roomStates.end()) {
          throw std::runtime_error("Habitación no encontrada en los estados dinámicos.");
        }

        const auto &roomState = *roomIt;

        // Buscar las enfermeras asignadas al turno
        for (const auto &nurseAssignment : solution.nurses) {
          auto shiftIt = std::find_if(nurseAssignment.assignments.begin(),
                                      nurseAssignment.assignments.end(),
                                      [day, &shift, &roomState](const NurseAssignment::ShiftAssignment &assignment) {
                                          return assignment.day == day &&
                                                assignment.shift == shift &&
                                                std::find(assignment.rooms.begin(), assignment.rooms.end(), roomState.room_info.id) != assignment.rooms.end();
                                      }
                                  ); 

          if (shiftIt != nurseAssignment.assignments.end()) {
            // Agregar la enfermera al conjunto de enfermeras únicas
            // std::cout << "Occupant: " << occupantAssignment.id << " Nurse: " << nurseAssignment.id << std::endl;
            uniqueNurses.insert(nurseAssignment.id);
          }
        }
        
      }
    }

    // Calcular la penalización basada en el número de enfermeras adicionales
    // std::cout << "Occupant: " << occupant.id << " Unique nurses: " << uniqueNurses.size() << std::endl;
    penalty += uniqueNurses.size() * problem.weights.continuity_of_care;
  }

  for (const auto &patientAssignment : solution.patients) {
    // Buscar el paciente en los datos del problema
    auto patientIt = std::find_if(problem.patients.begin(), problem.patients.end(),
                                  [&patientAssignment](const Patient &patient)
                                  {
                                    return patient.id == patientAssignment.id;
                                  });

    if (patientIt == problem.patients.end()) {
      throw std::runtime_error("Paciente no encontrado en los datos del problema.");
    }

    const auto &patient = *patientIt;

    // Conjunto de enfermeras únicas asignadas al paciente
    std::set<std::string> uniqueNurses;

    // Iterar por los días de la estancia del paciente
    for (int day = patientAssignment.admission_day;
         day < patientAssignment.admission_day + patient.length_of_stay; ++day) {
      
      if (day >= problem.days)
        break;

      // Iterar por cada turno
      for (const auto &shift : problem.shift_types) {
        // Buscar enfermeras asignadas a la habitación del paciente en este turno
        auto roomIt = std::find_if(
            solution.roomStates.begin(),
            solution.roomStates.end(),
            [&patientAssignment](const RoomState &roomState) {
                return roomState.room_info.id == patientAssignment.room;
            }
        );

        if (roomIt == solution.roomStates.end()) {
            throw std::runtime_error("Habitación no encontrada en los estados dinámicos.");
        }

        const auto &roomState = *roomIt;

        // Buscar las enfermeras asignadas al turno
        for (const auto &nurseAssignment : solution.nurses) {
            auto shiftIt = std::find_if(
                nurseAssignment.assignments.begin(),
                nurseAssignment.assignments.end(),
                [day, &shift, &roomState](const NurseAssignment::ShiftAssignment &assignment) {
                    return assignment.day == day &&
                          assignment.shift == shift &&
                          std::find(assignment.rooms.begin(), assignment.rooms.end(), roomState.room_info.id) != assignment.rooms.end();
                }
            );

            if (shiftIt != nurseAssignment.assignments.end()) {
              // Agregar la enfermera al conjunto de enfermeras únicas
              // std::cout << "Patient: " << patientAssignment.id << " Nurse: " << nurseAssignment.id << std::endl;
              uniqueNurses.insert(nurseAssignment.id);
            }
        }
      }
    }
    // std::cout << "Patient: " << patient.id << " Unique nurses: " << uniqueNurses.size() << std::endl;
    penalty += uniqueNurses.size() * problem.weights.continuity_of_care;
  }

  // Multiplicar la penalización acumulada por el peso
  return penalty;
}

/**
 * Calcula la penalización por carga de trabajo excesiva.
 * @return Penalización por carga de trabajo excesiva.
 */
int Solver::calculateMaximumWorkloadPenalty() {
  int penalty = 0;

  for (const auto &nurseAssignment : solution.nurses) {
    // Encontrar la enfermera en los datos del problema
    auto nurseIt = std::find_if(problem.nurses.begin(), problem.nurses.end(),
                                [&nurseAssignment](const Nurse &nurse)
                                {
                                  return nurse.id == nurseAssignment.id;
                                });

    if (nurseIt == problem.nurses.end()) {
      throw std::runtime_error("Enfermera no encontrada en los datos del problema.");
    }

    const auto &nurse = *nurseIt;

    // Iterar por los turnos asignados a la enfermera
    for (const auto &shiftAssignment : nurseAssignment.assignments) {
      int day = shiftAssignment.day;

      // Buscar el turno correspondiente en los turnos de trabajo de la enfermera
      auto shiftIt = std::find_if(nurse.working_shifts.begin(), nurse.working_shifts.end(),
                                  [day, &shiftAssignment](const Nurse::WorkingShift &workingShift)
                                  {
                                    return workingShift.day == day && workingShift.shift == shiftAssignment.shift;
                                  });

      if (shiftIt == nurse.working_shifts.end()) {
        throw std::runtime_error("Turno no encontrado en los turnos de trabajo de la enfermera. Día: " + std::to_string(day) + " Turno: " + shiftAssignment.shift + " Enfermera: " + nurse.id);
      }

      // Encontrar el turno de trabajo en los datos del problema
      int shiftIndex = std::distance(problem.shift_types.begin(),
                                     std::find(problem.shift_types.begin(), problem.shift_types.end(), shiftAssignment.shift));

      const auto &workingShift = *shiftIt;

      // Calcular la carga total de trabajo en este turno
      int totalWorkload = 0;

      for (const auto &roomId : shiftAssignment.rooms) {
        // Buscar la habitación
        auto roomIt = std::find_if(solution.roomStates.begin(), solution.roomStates.end(),
                                   [&roomId](const RoomState &roomState)
                                   {
                                     return roomState.room_info.id == roomId;
                                   });

        if (roomIt == solution.roomStates.end()) {
          throw std::runtime_error("Habitación no encontrada en los estados dinámicos.");
        }

        const auto &roomState = *roomIt;

        // Sumar la carga de trabajo de los pacientes
        if (roomState.patients_per_day.count(day)) {
          for (const auto &patient : roomState.patients_per_day.at(day)) {
            // Buscar el paciente en la solución del problema
            auto patientIt = std::find_if(solution.patients.begin(), solution.patients.end(),
                                          [&patient](const PatientAssignment &solutionPatient) {
                                            return solutionPatient.id == patient.id;
                                          });

            if (patientIt == solution.patients.end()) {
              throw std::runtime_error("Paciente no encontrado en la solución.");
            }

            const auto &patientSolution = *patientIt;

            // Calcular el índice basado en el día y el turno
            int index = (day - patientSolution.admission_day) * 3 + shiftIndex;

            if (index < 0 || index >= patient.workload_produced.size()) {
              throw std::out_of_range("Índice fuera de los límites en workload_produced.");
            }

            totalWorkload += patient.workload_produced[index];
          }
 
        }

        // Sumar la carga de trabajo de los ocupantes
        if (roomState.occupants_per_day.count(day)) {
          for (const auto &occupant : roomState.occupants_per_day.at(day)) {
            totalWorkload += occupant.workload_produced[day * 3 + shiftIndex];
          }
        }
      }

      if (totalWorkload == 0) {
        continue;
      }

      // std::cout << "Enfermera: " << nurse.id << " Habitaciones: ";
      // for (const auto &roomId : shiftAssignment.rooms) {
      //   std::cout << roomId << " ";
      // }
      // std::cout << " Día: " << day << " Turno: " << shiftAssignment.shift << "(" << (day * 3) + shiftIndex << ")" << " Carga total: " << totalWorkload << " Máximo: " << workingShift.max_load << std::endl;

      // Verificar si la carga total excede el máximo permitido
      if (totalWorkload > workingShift.max_load) {
        penalty += (totalWorkload - workingShift.max_load) * problem.weights.nurse_eccessive_workload;
      }
    }
  }

  // Multiplicar la penalización acumulada por el peso
  return penalty;
}

/**
 * Calcula la penalización por quirófanos abiertos
 * @return Penalización por quirófanos abiertos.
 */
int Solver::calculateOpenOperatingTheatersPenalty() {
  int penalty = 0;

  // Iterar por cada día del periodo de planificación
  for (int day = 0; day < problem.days; ++day) {
    int openTheaters = 0;

    // Verificar cada quirófano
    for (const auto &theaterState : solution.theaterStates) {
      if (theaterState.patients_per_day.count(day) && !theaterState.patients_per_day.at(day).empty()) {
        openTheaters++;
      }
    }

    // Penalizar por cada quirófano abierto
    penalty += openTheaters;
  }

  // Multiplicar la penalización acumulada por el peso
  return penalty * problem.weights.open_operating_theater;
}

/**
 * Calcula la penalización por transferencia de cirujanos
 * @return Penalización por transferencia de cirujanos.
 */
int Solver::calculateSurgeonTransferPenalty() {
  int penalty = 0;

  // Iterar por cada día del periodo de planificación
  for (int day = 0; day < problem.days; ++day) {
    // Mapa para rastrear quirófanos asignados a cada cirujano en un día
    std::map<std::string, std::set<std::string>> surgeonToTheaters;

    // Iterar por cada quirófano
    for (const auto &theaterState : solution.theaterStates) {
      // Verificar si hay pacientes asignados al quirófano en el día actual
      if (theaterState.patients_per_day.count(day)) {
        for (const auto &patient : theaterState.patients_per_day.at(day)) {
          // Asignar quirófano al cirujano correspondiente
          surgeonToTheaters[patient.surgeon_id].insert(theaterState.theater_info.id);
        }
      }
    }

    // Calcular la penalización para cada cirujano
    for (const auto &[surgeonId, theaters] : surgeonToTheaters) {
      if (theaters.size() > 1) {
        penalty += (theaters.size() - 1) * problem.weights.surgeon_transfer;
      }
    }
  }

  // Multiplicar la penalización acumulada por el peso
  return penalty;
}

/**
 * Calcula la penalización por retraso en la admisión de pacientes.
 * @return Penalización por retraso en la admisión de pacientes.
 */
int Solver::calculateAdmissionDelayPenalty() {
  int penalty = 0;

  for (const auto &patientAssignment : solution.patients) {
    // Buscar el paciente en los datos del problema
    auto patientIt = std::find_if(problem.patients.begin(), problem.patients.end(),
                                  [&patientAssignment](const Patient &patient)
                                  {
                                    return patient.id == patientAssignment.id;
                                  });

    if (patientIt == problem.patients.end()) {
      throw std::runtime_error("Paciente no encontrado en los datos del problema.");
    }

    const auto &patient = *patientIt;

    // Calcular el retraso
    if (patientAssignment.admission_day > patient.surgery_release_day) {
      penalty += patientAssignment.admission_day - patient.surgery_release_day;
    }
  }

  // Multiplicar la penalización acumulada por el peso
  return penalty * problem.weights.patient_delay;
}

/**
 * Calcula la penalización por pacientes no asignados a la solución.
 * @return Penalización por pacientes no asignados.
 */
int Solver::calculateUnscheduledOptionalPatientsPenalty() {
  int penalty = 0;

  for (const auto &patient : problem.patients) {
    // Verificar si el paciente es opcional
    if (!patient.mandatory) {
      // Verificar si el paciente está en la solución
      auto it = std::find_if(solution.patients.begin(), solution.patients.end(),
                             [&patient](const PatientAssignment &patientAssignment)
                             {
                               return patientAssignment.id == patient.id;
                             });

      // Si no se encuentra, penalizar
      if (it == solution.patients.end()) {
        penalty++;
      }
    }
  }

  // Multiplicar la penalización acumulada por el peso
  return penalty * problem.weights.unscheduled_optional;
}

/**
 * Calcula todas las restricciones duras y actualiza la solución.
 */
int Solver::calculateHardConstraintViolations() {
  int violations = 0;

  // H1: No mezcla de géneros en una habitación (considerando ocupantes)
  for (const auto &roomState : solution.roomStates) {
    std::set<int> all_days;

    // Recopilar todos los días en los que hay pacientes o ocupantes
    for (const auto &[day, _] : roomState.patients_per_day) {
      all_days.insert(day);
    }
    for (const auto &[day, _] : roomState.occupants_per_day) {
      all_days.insert(day);
    }

    // Revisar la mezcla de géneros en cada día donde hay ocupación
    for (int day : all_days) {
      std::set<std::string> genders;

      // Agregar géneros de los pacientes
      auto itPatients = roomState.patients_per_day.find(day);
      if (itPatients != roomState.patients_per_day.end()) {
        for (const auto &patient : itPatients->second) {
          genders.insert(patient.gender);
        }
      }

      // Agregar géneros de los ocupantes
      auto itOccupants = roomState.occupants_per_day.find(day);
      if (itOccupants != roomState.occupants_per_day.end()) {
        for (const auto &occupant : itOccupants->second) {
          genders.insert(occupant.gender);
        }
      }

      // Si hay más de un género en la misma habitación, es una violación
      if (genders.size() > 1) {
        violations++;
      }
    }
  }

  // std::cout << "H1: " << violations << std::endl;

  // H2: Compatibilidad de habitaciones (Solo pacientes nuevos)
  for (const auto &patientAssignment : solution.patients) {
    auto patientIt = std::find_if(originalProblem.patients.begin(), originalProblem.patients.end(),
                                  [&patientAssignment](const Patient &p) {
                                    return p.id == patientAssignment.id;
                                  });

    if (patientIt != originalProblem.patients.end()) {
      const auto &patient = *patientIt;
      if (std::find(patient.incompatible_room_ids.begin(), patient.incompatible_room_ids.end(),
                    patientAssignment.room) != patient.incompatible_room_ids.end()) {
        violations++;
      }
    }
  }

  // std::cout << "H2: " << violations << std::endl;

  // H3: Límite de tiempo del cirujano
  for (const auto &surgeon : originalProblem.surgeons) {
    std::vector<int> surgeryTime(originalProblem.days, 0);
    for (const auto &patientAssignment : solution.patients) {
      auto patientIt = std::find_if(originalProblem.patients.begin(), originalProblem.patients.end(),
                                    [&patientAssignment](const Patient &p) {
                                      return p.id == patientAssignment.id;
                                    });
      if (patientIt != originalProblem.patients.end() && patientIt->surgeon_id == surgeon.id) {
        surgeryTime[patientAssignment.admission_day] += patientIt->surgery_duration;
      }
    }
    for (int day = 0; day < originalProblem.days; ++day) {
      if (surgeryTime[day] > surgeon.max_surgery_time[day]) {
        violations++;
      }
    }
  }

  // std::cout << "H3: " << violations << std::endl;

  // H4: Límite de capacidad de los quirófanos
  for (const auto &theaterState : solution.theaterStates) {
    for (const auto &[day, patients] : theaterState.patients_per_day) {
      int totalSurgeryTime = std::accumulate(patients.begin(), patients.end(), 0, 
        [](int sum, const Patient& p) { return sum + p.surgery_duration; });

      // Buscar el quirófano original en originalProblem.theaters usando find_if
      auto itTheater = std::find_if(originalProblem.theaters.begin(), originalProblem.theaters.end(),
                                    [&theaterState](const OperatingTheater &t) {
                                      return t.id == theaterState.theater_info.id;
                                    });

      if (itTheater != originalProblem.theaters.end()) {
        // Verificar que el día sea un índice válido dentro del vector availability
        if (day >= 0 && day < static_cast<int>(itTheater->availability.size()) &&
            totalSurgeryTime > itTheater->availability[day]) {
          violations++;
        }
      }
    }
  }

  // std::cout << "H4: " << violations << std::endl;

  // H5: Todos los pacientes obligatorios deben ser admitidos
  for (const auto &patient : originalProblem.patients) {
    if (patient.mandatory) {
      auto it = std::find_if(solution.patients.begin(), solution.patients.end(),
                             [&patient](const PatientAssignment &pa) {
                               return pa.id == patient.id;
                             });
      if (it == solution.patients.end()) {
        violations++;
      }
    }
  }

  // std::cout << "H5: " << violations << std::endl;

  // H6: Restricción de fecha de admisión
  for (const auto &patientAssignment : solution.patients) {
    auto patientIt = std::find_if(originalProblem.patients.begin(), originalProblem.patients.end(),
                                  [&patientAssignment](const Patient &p) {
                                    return p.id == patientAssignment.id;
                                  });

    if (patientIt != originalProblem.patients.end()) {
      const auto &patient = *patientIt;
      if (patientAssignment.admission_day < patient.surgery_release_day ||
          (patient.mandatory && patientAssignment.admission_day > patient.surgery_due_day)) {
        violations++;
      }
    }
  }

  // std::cout << "H6: " << violations << std::endl;

  // H7: Capacidad de habitaciones (No puede excederse la capacidad)
  for (const auto &roomState : solution.roomStates) {
    for (const auto &[day, patients] : roomState.patients_per_day) {
      int totalOccupants = patients.size();
      if (roomState.occupants_per_day.count(day)) {
        totalOccupants += roomState.occupants_per_day.at(day).size();
      }
      if (totalOccupants > roomState.room_info.capacity) {
        violations++;
      }
    }
  }

  // std::cout << "H7: " << violations << std::endl;

  // H8: Presencia de enfermeras (Toda habitación ocupada debe tener una enfermera asignada)
  
  for (const auto &roomState : solution.roomStates) {
    for (int day = 0; day < originalProblem.days; ++day) {

      // 1) Verificar si la habitación está ocupada en "day"
      bool isOccupied = false;

      // ¿Hay pacientes ese día?
      auto patIt = roomState.patients_per_day.find(day);
      if (patIt != roomState.patients_per_day.end() && !patIt->second.empty()) {
        isOccupied = true;
      }
      // ¿Hay ocupantes ese día?
      auto occIt = roomState.occupants_per_day.find(day);
      if (!isOccupied && occIt != roomState.occupants_per_day.end() && !occIt->second.empty()) {
        isOccupied = true;
      }

      // 2) Si la habitación está ocupada, revisamos cada turno
      if (isOccupied) {
        // Recorre la lista de turnos definida en el problem (por ejemplo, morning, late, night)
        for (const auto &shift : originalProblem.shift_types) {
          bool nurseAssigned = false;

          // 3) Buscar si existe "alguna" enfermera que cubra (day, shift) de esta habitación
          for (const auto &nurseAssignment : solution.nurses) {
            for (const auto &shiftAssignment : nurseAssignment.assignments) {
              // Mismo día y mismo turno
              if (shiftAssignment.day == day &&
                  shiftAssignment.shift == shift) {
                // ¿Incluye esta habitación en su vector de rooms?
                auto found = std::find(shiftAssignment.rooms.begin(),
                                       shiftAssignment.rooms.end(),
                                       roomState.room_info.id);
                if (found != shiftAssignment.rooms.end()) {
                  nurseAssigned = true;
                  break; // Dejamos de buscar más en esta enfermera
                }
              }
            }
            if (nurseAssigned) {
              break; // Ya encontramos una enfermera para este turno
            }
          }

          // 4) Si no hay enfermera, esto viola la restricción H8
          if (!nurseAssigned) {
            violations++;
          }
        } // fin for(shift)
      }
    } // fin for(day)
  }   // fin for(roomStates)

  // std::cout << "H8: " << violations << std::endl;

  return violations;
}

/**
 * Exporta la solución generada a un archivo JSON.
 * @param filename Nombre del archivo de salida.
 */
void Solver::exportSolution(const std::string &filename) {
  solution.exportToJSON(filename);
}

/**
 * Asigna enfermeras a habitaciones según la carga de trabajo y habilidades.
 */
void Solver::assignNursesToRooms() {
  struct RoomData {
    RoomState* roomState;
    int totalWorkload;
    int requiredSkill;
  };

  // Carga de trabajo de cada enfermera por día y turno
  std::unordered_map<std::string, std::unordered_map<int, std::unordered_map<std::string, int>>> nurseLoad;
  // Para rastrear qué pacientes ya atendió cada enfermera
  std::unordered_map<std::string, std::unordered_set<std::string>> nurseAssignedPatients;
  for (auto &nurse : problem.nurses) {
    for (auto &ws : nurse.working_shifts)
      nurseLoad[nurse.id][ws.day][ws.shift] = 0;
    nurseAssignedPatients[nurse.id] = {};
  }

  // Mapa paciente -> día de admisión
  std::unordered_map<std::string,int> patientAdmissionMap;
  for (auto &pa : solution.patients)
    patientAdmissionMap[pa.id] = pa.admission_day;

  auto shiftIdx = [&](const std::string &s){ return s=="early"?0:(s=="late"?1:2); };
  std::vector<std::string> shifts = {"early","late","night"};

  for (int day = 0; day < problem.days; ++day) {
    for (auto &shift : shifts) {
      // 1) Recolectar datos de todas las habitaciones ocupadas
      std::vector<RoomData> rooms;
      for (auto &rs : solution.roomStates) {
        if (!rs.patients_per_day.count(day) && !rs.occupants_per_day.count(day)) continue;
        RoomData rd{&rs, 0, 0};
        // Nuevos pacientes
        if (rs.patients_per_day.count(day)) {
          for (auto &p : rs.patients_per_day[day]) {
            int rel = day - patientAdmissionMap[p.id];
            int idx = rel*3 + shiftIdx(shift);
            if (idx < (int)p.workload_produced.size()) rd.totalWorkload += p.workload_produced[idx];
            if (idx < (int)p.skill_level_required.size()) rd.requiredSkill = std::max(rd.requiredSkill, p.skill_level_required[idx]);
          }
        }
        // Ocupantes
        if (rs.occupants_per_day.count(day)) {
          for (auto &o : rs.occupants_per_day[day]) {
            int idx = day*3 + shiftIdx(shift);
            if (idx < (int)o.workload_produced.size()) rd.totalWorkload += o.workload_produced[idx];
            if (idx < (int)o.skill_level_required.size()) rd.requiredSkill = std::max(rd.requiredSkill, o.skill_level_required[idx]);
          }
        }
        rooms.push_back(rd);
      }

      // 2) Ordenar habitaciones por requerimientos: primero skill, luego carga
      std::sort(rooms.begin(), rooms.end(), [](const RoomData &a, const RoomData &b){
        if (a.requiredSkill != b.requiredSkill) return a.requiredSkill > b.requiredSkill;
        return a.totalWorkload > b.totalWorkload;
      });

      // 3) Asignar enfermeras habitación por habitación según el orden anterior
      for (auto &rd : rooms) {
        double bestCost = std::numeric_limits<double>::max();
        std::string bestNurse;
        for (auto &n : problem.nurses) {
          bool available = false;
          int maxL = -1;
          for (auto &ws : n.working_shifts) {
            if (ws.day == day && ws.shift == shift) { available = true; maxL = ws.max_load; break; }
          }
          if (!available) continue;
          int curr = nurseLoad[n.id][day][shift];
          double skillCost = n.skill_level < rd.requiredSkill
            ? (rd.requiredSkill - n.skill_level) * problem.weights.room_nurse_skill : 0;
          double loadCost = curr + rd.totalWorkload > maxL
            ? (curr + rd.totalWorkload - maxL) * problem.weights.nurse_eccessive_workload : 0;
          double cost = curr + skillCost + loadCost;
          // Penalizar si no ha atendido a ninguno de los pacientes actuales
          bool had = false;
          if (rd.roomState->patients_per_day.count(day)) {
            for (auto &p : rd.roomState->patients_per_day[day]) {
              if (nurseAssignedPatients[n.id].count(p.id)) { had = true; break; }
            }
          }
          if (!had) cost += problem.weights.continuity_of_care;
          if (cost < bestCost) { bestCost = cost; bestNurse = n.id; }
        }
        if (bestNurse.empty()) continue;
        // Registrar atención y carga
        for (auto &p : rd.roomState->patients_per_day[day])
          nurseAssignedPatients[bestNurse].insert(p.id);
        nurseLoad[bestNurse][day][shift] += rd.totalWorkload;
        // Actualizar solution.nurses
        bool found = false;
        for (auto &na : solution.nurses) if (na.id == bestNurse) {
          bool sf = false;
          for (auto &sa : na.assignments) {
            if (sa.day == day && sa.shift == shift) {
              sa.rooms.push_back(rd.roomState->room_info.id);
              sf = true; break;
            }
          }
          if (!sf) na.assignments.push_back({day, shift, {rd.roomState->room_info.id}});
          found = true; break;
        }
        if (!found) {
          solution.nurses.push_back({bestNurse, {{day, shift, {rd.roomState->room_info.id}}}});
        }
      }
    }
  }
}