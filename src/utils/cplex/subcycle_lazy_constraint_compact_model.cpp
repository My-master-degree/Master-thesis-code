#include "utils/cplex/subcycle_lazy_constraint_compact_model.hpp"
#include "utils/cplex/compact_model.hpp"

#include <set>
#include <ilcplex/ilocplex.h>
//ILOSTLBEGIN

using namespace std;
using namespace utils::cplex;

Subcycle_lazy_constraint_compact_model::Subcycle_lazy_constraint_compact_model (Compact_model& compact_model_) : LazyConstraintCallbackI (compact_model_.env), compact_model(compact_model_)  {}

IloCplex::CallbackI* Subcycle_lazy_constraint_compact_model::duplicateCallback() const {
  return new(getEnv()) Subcycle_lazy_constraint_compact_model (*this);
}

void Subcycle_lazy_constraint_compact_model::main() {
//  const auto x_value = getValue(x);
// if(x_value < 2.0) {
//   try {
//     add(x >= 2.0);
//   } catch(IloException& e) {
//     std::cerr << "Exception while adding lazy constraint for x = " << x_value << ": " << e.getMessage() << "\n";
//     throw;
//   }
  int depot = compact_model.gvrp_instance.depot.id;
  IloEnv env = getEnv();
  IloExpr expr(env);
  //get values
  Matrix3DVal x_vals (env, compact_model.gvrp_instance.customers.size());
  for (unsigned int k = 0; k < compact_model.gvrp_instance.customers.size(); k++) {
    x_vals[k] = IloArray<IloNumArray> (env, compact_model.all.size());
    for (pair<int, Vertex> p : compact_model.all) {
      int i = p.first;
      x_vals[k][i] = IloNumArray (env, compact_model.all.size(), 0, compact_model.ub_edge_visit, IloNumVar::Int);
      getValues(x_vals[k][i], compact_model.x[k][i]);
    }
  }
  //for each route
  for (int k = 0; k < int(compact_model.gvrp_instance.customers.size()); k++){
    set<int> vertexes;
    for (pair<int, Vertex> p : compact_model.all){
      int i = p.first;
      for (pair<int, Vertex> p1 : compact_model.all){
        int j = p1.first;
        if (x_vals[k][i][j] > 0){
          vertexes.insert(i);
          vertexes.insert(j);
        }
      }
    }
    //subcycle found
    if (vertexes.size() > 0 && vertexes.find(depot) == vertexes.end()){
      for (pair<int, Vertex> p : compact_model.all){
        int i = p.first;
        for (int j : vertexes)
          if (vertexes.find(i) == vertexes.end())
            expr += compact_model.x[k][i][j];
      }       
      expr -= 1;
      try {
        add(expr >= 0).end();
      } catch(IloException& e) {
        cerr << "Exception while adding lazy constraint" << e.getMessage() << "\n";
        throw;
      } 
      expr.end();
      expr = IloExpr(env);
    }
    for (pair<int, Vertex> p : compact_model.all){
      int i = p.first;
      x_vals[k][i].end();
    }
  }
  x_vals.end();

}
