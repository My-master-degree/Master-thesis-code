#ifndef INSTANCES_GENERATION_MODEL_HPP_
#define INSTANCES_GENERATION_MODEL_HPP_

#include "models/vrp_instance.hpp"
#include "models/gvrp_instance.hpp"
#include "models/cplex_model.hpp"
#include "models/mip_solution_info.hpp"
#include "models/instance_generation_model/lazy_constraint.hpp"
#include "models/instance_generation_model/user_constraint.hpp"

#include <ilcplex/ilocplex.h>
#include <list>

ILOSTLBEGIN

using namespace std;
using namespace models;

typedef IloArray<IloNumVarArray> Matrix2DVar;
typedef IloArray<IloNumArray> Matrix2DVal;

namespace models {
  namespace instance_generation_model {
    class Lazy_constraint;
    class User_constraint;
    class Instance_generation_model : public Cplex_model<Vrp_instance, Gvrp_instance> {
      public:
        explicit Instance_generation_model(Vrp_instance& vrp_instance, unsigned int time_limit);
        pair<Gvrp_instance, Mip_solution_info> run();
        Matrix2DVar x;
        Matrix2DVal x_vals;
        IloNumVarArray z;
        IloNumArray z_vals;
        list<User_constraint*> user_constraints;
        size_t sNodes;
        void fillVals();
      private:
        Lazy_constraint* separation_algorithm();
        void createVariables();
        void createObjectiveFunction();
        void createModel();
        void setCustomParameters();
        void createGvrp_instance();
        void endVars();
    };
  }
}

#endif
