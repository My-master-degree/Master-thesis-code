#ifndef COMPACT_MODEL_HPP_
#define COMPACT_MODEL_HPP_

#include <map>
#include <list>
#include <models/gvrp_instance.hpp>
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

using namespace std;
using namespace models;

typedef map<int, Vertex> IDVertex;
typedef IloArray<IloArray<IloNumVarArray> > Matrix3DVar;
typedef IloArray<IloNumVarArray> Matrix2DVar;
typedef IloArray<IloArray<IloNumArray> > Matrix3DVal;
typedef IloArray<IloNumArray> Matrix2DVal;

namespace utils {
  namespace cplex {
    class Compact_model {
      public:
        explicit Compact_model(Gvrp_instance& gvrp_instance, unsigned int time_limit); 
        explicit Compact_model(Gvrp_instance& gvrp_instance, unsigned int time_limit, unsigned int max_num_feasible_integer_sol);
        list<list<Vertex> > run();
        Gvrp_instance gvrp_instance;
        unsigned int time_limit;//seconds
        unsigned int max_num_feasible_integer_sol;//0 to 2100000000
      private:
        IloModel model;
        IloEnv env;
        IloCplex cplex;
        Matrix3DVar x;
        Matrix3DVar t;
        IloNumVarArray e;
        Matrix3DVal x_vals;
        IDVertex all;
        list<list<Vertex> > routes;
        int ub_edge_visit;
        void createVariables();
        void createObjectiveFunction();
        void createModel() throw ();
        void setCustomParameters();
        void getSolution();
        void getRoutes();
    };

  } 
}
#endif
