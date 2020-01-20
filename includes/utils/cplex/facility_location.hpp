#ifndef FACILITY_LOCATION_HPP_
#define FACILITY_LOCATION_HPP_

#include "models/vertex.hpp"
#include "models/mip_solution_info.hpp"
#include "models/facility_location_instance.hpp"

#include <list>

using namespace models;

namespace utils {
  namespace cplex {
    class Facility_location {
      public:
        explicit Facility_location (Facility_location_instance& facility_location_instance);
        pair<list<Vertex>, Mip_solution_info> run();
      private:
        Facility_location_instance& facility_location_instance;


    };
  }
}

#endif
