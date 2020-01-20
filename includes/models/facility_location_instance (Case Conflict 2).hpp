#ifndef FACILITY_LOCATION_HPP_
#define FACILITY_LOCATION_HPP_

namespace models {
  class Facility_location_instance {
    public:
      explicit Facility_location (list<Vertex> nodes, vector<vector<double> > distances);
    private:
      list<Vertex> nodes;
      vector<vector<double> > distances;
  };
}

#define
