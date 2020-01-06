#include "models/mip_solution_info.hpp"

#include <ilcplex/ilocplex.h>

using namespace models;

Mip_solution_info::Mip_solution_info(double gap_, IloAlgorithm::Status status_) : gap(gap_), status(status_){
}
