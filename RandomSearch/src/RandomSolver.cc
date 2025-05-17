/**
 * Universidad de La Laguna
 * Escuela Superior de Ingeniería y Tecnología
 * Grado en Ingeniería Informática
 *
 * @author Javier Almenara Herrera
 * @brief Implementación de clase que representa una búsqueda aleatoria para el problema de asignación hospitalaria IHTP.
 * @file RandomSolver.cc
 */

#include "RandomSolver.h"
#include <vector>
#include <string>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <set>

/**
 * Constructor de la clase RandomSolver.
 * @param problemInstance Instancia del problema a resolver.
 */
RandomSolver::RandomSolver(ProblemInstance &problemInstance)
    : problem(problemInstance), rng(std::random_device{}()) {}

/**
 * Genera una solución válida al problema de asignación hospitalaria.
 */
Solution RandomSolver::generateSolution() {
  rng.seed(static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count()));

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

  solution.nurses.clear(); // Limpiar asignaciones previas
  for (const auto &nurse : problem.nurses) {
    NurseAssignment nurseAssignment = {nurse.id, {}};
    // Inicializar todos los turnos de trabajo
    for (const auto &shift : nurse.working_shifts) {
      NurseAssignment::ShiftAssignment shiftAssignment = {shift.day, shift.shift, {}};
      nurseAssignment.assignments.push_back(shiftAssignment);
    }
    solution.nurses.push_back(nurseAssignment);
  }
  
  // Calcular la solución aleatoria
  assignMandatoryPatients();
  assignOptionalPatients();
  assignNursesToRooms();

  // Calcular las restricciones blandas
  calculateSoftConstraints();

  return solution;
}

/**
 * Asigna pacientes obligatorios a quirófanos y habitaciones.
 */
void RandomSolver::assignMandatoryPatients() {
  // Crear un vector para almacenar solo los pacientes obligatorios
  std::vector<Patient> mandatoryPatients;

  // Filtrar los pacientes obligatorios
  for (const auto& patient : problem.patients) {
    if (patient.mandatory) {
      mandatoryPatients.push_back(patient);
    }
  }

  // Ordenar los pacientes por el margen de tiempo disponible (menor primero)
  std::sort(mandatoryPatients.begin(), mandatoryPatients.end(), [](const Patient& a, const Patient& b) {
    int marginA = a.surgery_due_day - a.surgery_release_day;
    int marginB = b.surgery_due_day - b.surgery_release_day;
    return marginA < marginB; // Menor margen tiene prioridad
  });

  // Iterar sobre los pacientes obligatorios ordenados
  for (const auto& patient : mandatoryPatients) {
    bool assigned = false;
    int firstDay = patient.surgery_release_day;
    int operationDay = -1;
    int selectedTheaterIndex = -1;
    RoomState* selectedRoom = nullptr;

    while (!assigned && firstDay <= patient.surgery_due_day) {
      // PASO 1: Intentar asignar quirófano
      bool theaterAvailable = false;

      for (int day = firstDay; day <= patient.surgery_due_day && !theaterAvailable; ++day) {
        auto& surgeon = findSurgeonById(patient.surgeon_id);

        if (surgeon.max_surgery_time[day] < patient.surgery_duration) continue;

        std::uniform_int_distribution<> theaterDist(0, problem.theaters.size() - 1);
        int theaterIndex = theaterDist(rng);

        auto& theaterState = solution.theaterStates[theaterIndex];
        if (theaterState.availability_per_day[day] < patient.surgery_duration) continue;

        operationDay = day;
        selectedTheaterIndex = theaterIndex;
        theaterAvailable = true;
      }

      if (!theaterAvailable) {
        std::cerr << "Error: No se pudo asignar quirófano para el paciente " << patient.id << ".\n";
        break;
      }

      // PASO 2: Intentar asignar habitación
      bool roomAvailable = false;
      std::vector<RoomState*> compatibleRooms;
      for (auto& roomState : solution.roomStates) {
        if (std::find(patient.incompatible_room_ids.begin(), patient.incompatible_room_ids.end(), roomState.room_info.id) == patient.incompatible_room_ids.end()) {
          compatibleRooms.push_back(&roomState);
        }
      }

      while (!roomAvailable && !compatibleRooms.empty()) {
        std::uniform_int_distribution<> roomDist(0, compatibleRooms.size() - 1);
        int roomIndex = roomDist(rng);
        auto& roomState = *compatibleRooms[roomIndex];

        bool isRoomSuitable = true;
        for (int day = operationDay; day < std::min(operationDay + patient.length_of_stay, problem.days); ++day) {
          if (roomState.capacity_per_day[day] <= 0) {
            isRoomSuitable = false;
            break;
          }

          auto& occupants = roomState.occupants_per_day[day];
          if (!occupants.empty() &&
              std::any_of(occupants.begin(), occupants.end(), [&patient](const Occupant& o) { return o.gender != patient.gender; })) {
            isRoomSuitable = false;
            break;
          }

          auto& patients = roomState.patients_per_day[day];
          if (!patients.empty() &&
              std::any_of(patients.begin(), patients.end(), [&patient](const Patient& p) { return p.gender != patient.gender; })) {
            isRoomSuitable = false;
            break;
          }
        }

        if (isRoomSuitable) {
          selectedRoom = &roomState;
          roomAvailable = true;
        } else {
          compatibleRooms.erase(compatibleRooms.begin() + roomIndex);
        }
      }

      if (!roomAvailable) {
        firstDay++;
        selectedTheaterIndex = -1; // Reiniciar la selección del quirófano
        continue;
      }

      // Si ambos recursos están disponibles, realizar la asignación
      auto& theaterState = solution.theaterStates[selectedTheaterIndex];
      theaterState.patients_per_day[operationDay].push_back(patient);
      theaterState.availability_per_day[operationDay]--;
      theaterState.availability_per_day[operationDay] -= patient.surgery_duration;

      auto& surgeon = findSurgeonById(patient.surgeon_id);
      surgeon.max_surgery_time[operationDay] -= patient.surgery_duration;

      for (int day = operationDay; day < std::min(operationDay + patient.length_of_stay, problem.days); ++day) {
        selectedRoom->capacity_per_day[day]--;
        selectedRoom->patients_per_day[day].push_back(patient);
      }

      solution.patients.push_back({patient.id, operationDay, selectedRoom->room_info.id, theaterState.theater_info.id});
      assigned = true;
    }

    if (!assigned) {
      // std::cerr << "Error: No se pudo asignar ni quirófano ni habitación para el paciente " << patient.id << ".\n";
      throw std::runtime_error("No se pudo asignar ni quirófano ni habitación para un paciente obligatorio."); 
    }
  }
}

/**
 * Asigna pacientes opcionales a quirófanos y habitaciones.
 */
void RandomSolver::assignOptionalPatients() {
  // Crear un vector para almacenar solo los pacientes opcionales
  std::vector<Patient> optionalPatients;

  // Filtrar los pacientes opcionales
  for (const auto& patient : problem.patients) {
    if (!patient.mandatory) {
      optionalPatients.push_back(patient);
    }
  }

  // Ordenar los pacientes opcionales por duración de cirugía (mayor primero)
  std::sort(optionalPatients.begin(), optionalPatients.end(), [](const Patient& a, const Patient& b) {
    return a.surgery_duration > b.surgery_duration; // Mayor duración tiene prioridad
  });

  // Iterar sobre los pacientes opcionales ordenados
  for (const auto& patient : optionalPatients) {
    bool assigned = false;
    int firstDay = patient.surgery_release_day;
    int operationDay = -1;
    int selectedTheaterIndex = -1;
    RoomState* selectedRoom = nullptr;

    while (!assigned && firstDay <= problem.days) {
      // PASO 1: Intentar asignar quirófano
      bool theaterAvailable = false;

      for (int day = firstDay; day <= problem.days && !theaterAvailable; ++day) {
        auto& surgeon = findSurgeonById(patient.surgeon_id);

        if (surgeon.max_surgery_time[day] < patient.surgery_duration) continue;

        std::uniform_int_distribution<> theaterDist(0, problem.theaters.size() - 1);
        int theaterIndex = theaterDist(rng);

        auto& theaterState = solution.theaterStates[theaterIndex];
        if (theaterState.availability_per_day[day] < patient.surgery_duration) continue;

        operationDay = day;
        selectedTheaterIndex = theaterIndex;
        theaterAvailable = true;
      }

      // PASO 2: Intentar asignar habitación
      bool roomAvailable = false;
      std::vector<RoomState*> compatibleRooms;
      for (auto& roomState : solution.roomStates) {
        if (std::find(patient.incompatible_room_ids.begin(), patient.incompatible_room_ids.end(), roomState.room_info.id) == patient.incompatible_room_ids.end()) {
          compatibleRooms.push_back(&roomState);
        }
      }

      while (!roomAvailable && !compatibleRooms.empty()) {
        std::uniform_int_distribution<> roomDist(0, compatibleRooms.size() - 1);
        int roomIndex = roomDist(rng);
        auto& roomState = *compatibleRooms[roomIndex];

        bool isRoomSuitable = true;
        for (int day = operationDay; day < std::min(operationDay + patient.length_of_stay, problem.days); ++day) {
          if (roomState.capacity_per_day[day] <= 0) {
            isRoomSuitable = false;
            break;
          }

          auto& occupants = roomState.occupants_per_day[day];
          if (!occupants.empty() &&
              std::any_of(occupants.begin(), occupants.end(), [&patient](const Occupant& o) { return o.gender != patient.gender; })) {
            isRoomSuitable = false;
            break;
          }

          auto& patients = roomState.patients_per_day[day];
          if (!patients.empty() &&
              std::any_of(patients.begin(), patients.end(), [&patient](const Patient& p) { return p.gender != patient.gender; })) {
            isRoomSuitable = false;
            break;
          }
        }

        if (isRoomSuitable) {
          selectedRoom = &roomState;
          roomAvailable = true;
        } else {
          compatibleRooms.erase(compatibleRooms.begin() + roomIndex);
        }
      }

      if (!roomAvailable) {
        firstDay++;
        selectedTheaterIndex = -1; // Reiniciar la selección del quirófano
        continue;
      }

      // Si ambos recursos están disponibles, realizar la asignación
      auto& theaterState = solution.theaterStates[selectedTheaterIndex];
      theaterState.patients_per_day[operationDay].push_back(patient);
      theaterState.availability_per_day[operationDay]--;
      theaterState.availability_per_day[operationDay] -= patient.surgery_duration;

      auto& surgeon = findSurgeonById(patient.surgeon_id);
      surgeon.max_surgery_time[operationDay] -= patient.surgery_duration;

      for (int day = operationDay; day < std::min(operationDay + patient.length_of_stay, problem.days); ++day) {
        selectedRoom->capacity_per_day[day]--;
        selectedRoom->patients_per_day[day].push_back(patient);
      }

      solution.patients.push_back({patient.id, operationDay, selectedRoom->room_info.id, theaterState.theater_info.id});
      assigned = true;
    }

    // if (!assigned) {
    //   std::cerr << "No se pudo asignar ni quirófano ni habitación para el paciente " << patient.id << " (OPCIONAL).\n";
    // }
  }
}

/**
 * Asigna enfermeras a las habitaciones de forma aleatoria.
 */
void RandomSolver::assignNursesToRooms() {
  std::uniform_int_distribution<> nurseDist(0, problem.nurses.size() - 1);

  for (auto &roomState : solution.roomStates) {
    for (int day = 0; day < problem.days; ++day) {

      // Verificar si hay ocupantes o pacientes asignados a la habitación en el día
      auto occupants = roomState.occupants_per_day[day];
      auto patients = roomState.patients_per_day[day];
      if (occupants.empty() && patients.empty()) {
        // Si no hay ocupantes ni pacientes, no asignar enfermeras
        continue;
      }

      for (const auto &shift : problem.shift_types) { // Iterar por turnos
        
        bool assigned = false;

        while (!assigned) {
          // Seleccionar una enfermera aleatoriamente
          int nurseIndex = nurseDist(rng);
          auto &nurse = problem.nurses[nurseIndex];
          auto &nurseAssignment = solution.nurses[nurseIndex];

          // Verificar si la enfermera tiene el turno disponible
          auto it = std::find_if(nurse.working_shifts.begin(), nurse.working_shifts.end(),
                                 [day, &shift](const Nurse::WorkingShift &workingShift) {
                                   return workingShift.day == day && workingShift.shift == shift;
                                 });
          // Si la enfermera no tiene el turno, intentar con otra enfermera
          if (it == nurse.working_shifts.end()) {
            continue;
          }

          // Buscar la asignación de turno existente en la solución
          auto shiftIt = std::find_if(nurseAssignment.assignments.begin(), nurseAssignment.assignments.end(),
                                      [day, &shift](const NurseAssignment::ShiftAssignment& shiftAssignment) {
                                        return shiftAssignment.day == day && shiftAssignment.shift == shift;
                                      });

          // Agregar la habitación al turno correspondiente
          if (shiftIt != nurseAssignment.assignments.end()) {
            shiftIt->rooms.push_back(roomState.room_info.id);
          } else {
            NurseAssignment::ShiftAssignment newShiftAssignment = {day, shift, {roomState.room_info.id}};
            nurseAssignment.assignments.push_back(newShiftAssignment);
          }

          assigned = true;
        }
      }
    }
  }
}


/**
 * Calcula todas las restricciones blandas y actualiza la solución.
 */
void RandomSolver::calculateSoftConstraints() {
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
}

/**
 * Calcula la penalización por mezcla de grupos de edad en las habitaciones.
 * @return Penalización por mezcla de grupos de edad.
 */
int RandomSolver::calculateAgeGroupPenalty() {
  int penalty = 0;

  for (const auto &roomState : solution.roomStates) {
    for (const auto &[day, patients] : roomState.patients_per_day) {
      std::set<int> ageIndices;

      // Incluir pacientes en el cálculo
      for (const auto &patient : patients) {
        auto it = std::find(problem.age_groups.begin(), problem.age_groups.end(), patient.age_group);
        if (it != problem.age_groups.end()) {
          // std::cout << "Habitaion: " << roomState.room_info.id << " Patient: " << patient.id << " Age group: " << patient.age_group << std::endl;
          ageIndices.insert(std::distance(problem.age_groups.begin(), it));
        }
      }

      // Incluir ocupantes en el cálculo
      const auto &occupants = roomState.occupants_per_day.at(day);
      for (const auto &occupant : occupants) {
        auto it = std::find(problem.age_groups.begin(), problem.age_groups.end(), occupant.age_group);
        if (it != problem.age_groups.end()) {
          // std::cout << "Habitaion: " << roomState.room_info.id << " Occupant: " << occupant.id << " Age group: " << occupant.age_group << std::endl;
          ageIndices.insert(std::distance(problem.age_groups.begin(), it));
        }
      }

      // Calcular la penalización si hay más de un grupo de edad
      if (ageIndices.size() > 1) {
        int maxAge = *std::max_element(ageIndices.begin(), ageIndices.end());
        int minAge = *std::min_element(ageIndices.begin(), ageIndices.end());
        // std::cout << "Habitación: " << roomState.room_info.id << " Día: " << day << " Máximo: " << maxAge << " Mínimo: " << minAge << std::endl;
        // std::cout << "Penalización: " << (problem.weights.room_mixed_age * (maxAge - minAge)) << std::endl;
        penalty += (problem.weights.room_mixed_age * (maxAge - minAge)); // Diferencia máxima
      }
    }
  }

  // Multiplicar la penalización acumulada por el peso
  return penalty; 
}

/**
 * Calcula la penalización para aquellas enfermeras que no cumplen con el nivel de habilidad mínimo.
 * @return Penalización por habilidades mínimas.
 */
int RandomSolver::calculateMinimumSkillPenalty() {
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
int RandomSolver::calculateContinuityOfCarePenalty() {
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
int RandomSolver::calculateMaximumWorkloadPenalty() {
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
        throw std::runtime_error("Turno no encontrado en los turnos de trabajo de la enfermera.");
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
int RandomSolver::calculateOpenOperatingTheatersPenalty() {
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
int RandomSolver::calculateSurgeonTransferPenalty() {
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
int RandomSolver::calculateAdmissionDelayPenalty() {
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
int RandomSolver::calculateUnscheduledOptionalPatientsPenalty() {
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
 * Exporta la solución generada a un archivo JSON.
 * @param filename Nombre del archivo de salida.
 */
void RandomSolver::exportSolution(const std::string &filename) {
  solution.exportToJSON(filename);
}

/**
 * Resuelve el problema de asignación hospitalaria.
 */
void RandomSolver::assignmentPatientsToRoomsAndTheaters() {
  for (const auto& patient : problem.patients) {
    if (!patient.mandatory) continue; // Solo pacientes obligatorios

    bool assigned = false;
    int firstDay = patient.surgery_release_day;
    int operationDay = -1;
    int selectedTheaterIndex = -1;
    RoomState* selectedRoom = nullptr;

    while (!assigned && firstDay <= patient.surgery_due_day) {
      // PASO 1: Intentar asignar quirófano
      bool theaterAvailable = false;

      for (int day = firstDay; day <= patient.surgery_due_day && !theaterAvailable; ++day) {
        auto& surgeon = findSurgeonById(patient.surgeon_id);

        if (surgeon.max_surgery_time[day] < patient.surgery_duration) continue;

        std::uniform_int_distribution<> theaterDist(0, problem.theaters.size() - 1);
        int theaterIndex = theaterDist(rng);

        auto& theaterState = solution.theaterStates[theaterIndex];
        if (theaterState.availability_per_day[day] < patient.surgery_duration) continue;

        operationDay = day;
        selectedTheaterIndex = theaterIndex;
        theaterAvailable = true;
      }

      if (!theaterAvailable) {
        std::cerr << "Error: No se pudo asignar quirófano para el paciente " << patient.id << ".\n";
        break;
      }

      // PASO 2: Intentar asignar habitación
      bool roomAvailable = false;
      std::vector<RoomState*> compatibleRooms;
      for (auto& roomState : solution.roomStates) {
        if (std::find(patient.incompatible_room_ids.begin(), patient.incompatible_room_ids.end(), roomState.room_info.id) == patient.incompatible_room_ids.end()) {
          compatibleRooms.push_back(&roomState);
        }
      }

      while (!roomAvailable && !compatibleRooms.empty()) {
        std::uniform_int_distribution<> roomDist(0, compatibleRooms.size() - 1);
        int roomIndex = roomDist(rng);
        auto& roomState = *compatibleRooms[roomIndex];

        bool isRoomSuitable = true;
        for (int day = operationDay; day < std::min(operationDay + patient.length_of_stay, problem.days); ++day) {
          if (roomState.capacity_per_day[day] <= 0) {
            isRoomSuitable = false;
            break;
          }

          auto& occupants = roomState.occupants_per_day[day];
          if (!occupants.empty() &&
              std::any_of(occupants.begin(), occupants.end(), [&patient](const Occupant& o) { return o.gender != patient.gender; })) {
            isRoomSuitable = false;
            break;
          }

          auto& patients = roomState.patients_per_day[day];
          if (!patients.empty() &&
              std::any_of(patients.begin(), patients.end(), [&patient](const Patient& p) { return p.gender != patient.gender; })) {
            isRoomSuitable = false;
            break;
          }
        }

        if (isRoomSuitable) {
          selectedRoom = &roomState;
          roomAvailable = true;
        } else {
          compatibleRooms.erase(compatibleRooms.begin() + roomIndex);
        }
      }

      if (!roomAvailable) {
        firstDay++;
        selectedTheaterIndex = -1; // Reiniciar la selección del quirófano
        continue;
      }

      // Si ambos recursos están disponibles, realizar la asignación
      auto& theaterState = solution.theaterStates[selectedTheaterIndex];
      theaterState.patients_per_day[operationDay].push_back(patient);
      theaterState.availability_per_day[operationDay]--;
      theaterState.availability_per_day[operationDay] -= patient.surgery_duration;

      auto& surgeon = findSurgeonById(patient.surgeon_id);
      surgeon.max_surgery_time[operationDay] -= patient.surgery_duration;

      for (int day = operationDay; day < std::min(operationDay + patient.length_of_stay, problem.days); ++day) {
        selectedRoom->capacity_per_day[day]--;
        selectedRoom->patients_per_day[day].push_back(patient);
      }

      solution.patients.push_back({patient.id, operationDay, selectedRoom->room_info.id, theaterState.theater_info.id});
      assigned = true;
    }

    if (!assigned) {
      std::cerr << "Error: No se pudo asignar ni quirófano ni habitación para el paciente " << patient.id << ".\n";
    }
  }
}