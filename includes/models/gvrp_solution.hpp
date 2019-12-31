#ifndef GVRP_SOLUTION_HPP_
#define GVRP_SOLUTION_HPP_

#include "vertex.hpp"
#include "gvrp_instance.hpp"

#include <list>
#include <string>
#include <sstream>

using namespace std;

namespace models{

  class Gvrp_solution {
    public:
      explicit Gvrp_solution(list<list<Vertex> > routes, Gvrp_instance gvrp_instance);
      friend ostream& operator<<(ostream& strm, const Gvrp_solution& gvrp_solution){
        stringstream output;
        int i = 1;
        double costs = 0;
        for (list<Vertex> route : gvrp_solution.routes){
          double cost = 0;
          output<<"Route "<<i++<<": ";
          auto it = route.begin();
          int curr = it->id,
              next;
          output<<curr;
          for (int i = 0; i < route.size() - 1; i++){
            it++;
            next = it->id;
            output<<" "<<next;
            cost += gvrp_solution.gvrp_instance.distances[curr][next];
            curr = next;            
          }
          output<<" (cost: "<<cost<<")"<<endl;
          costs += cost;
        }
        output<<"Total cost: "<<costs;
        return strm << output.str();
      };
//      void 
      list<list<Vertex> > routes;
      Gvrp_instance gvrp_instance;
  };

}
#endif
