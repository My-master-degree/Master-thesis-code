#include "models/mip_depth.hpp"

#include <ilcplex/ilocplex.h>

using namespace models;

Depth::Depth(IloInt d) : depth(d) {}
