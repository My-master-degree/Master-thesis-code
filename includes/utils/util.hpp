#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <string>
#include "models/gvrp_instance.hpp"

using namespace models;

namespace utils {
  Gvrp_instance erdogan_instance_reader(string file_path);
} 
#endif
