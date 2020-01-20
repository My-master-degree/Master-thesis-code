#include "utils/cplex/lazy_constraint_compact_model.hpp"
#include "utils/cplex/compact_model.hpp"
#include "utils/cplex/subcycle_lazy_constraint_compact_model.hpp"

#include <set>
#include <queue>
#include <list>
#include <ilcplex/ilocplex.h>

using namespace std;
using namespace utils::cplex;

Subcycle_lazy_constraint_compact_model::Subcycle_lazy_constraint_compact_model (Compact_model& compact_model_) : Lazy_constraint_compact_model (compact_model_) {}

IloCplex::CallbackI* Subcycle_lazy_constraint_compact_model::duplicateCallback() const {
  return new(getEnv()) Subcycle_lazy_constraint_compact_model (*this);
}

void Subcycle_lazy_constraint_compact_model::main() {
  int depot = compact_model.gvrp_instance.depot.id;
  IloEnv env = getEnv();
  IloExpr lhs(env);
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
  //get subcycles
  for (int k = 0; k < int(compact_model.gvrp_instance.customers.size()); k++) {
    //bfs to remove all edges connected to the depot
    queue<int> q;
    q.push(depot);
    while (!q.empty()) {
      int curr = q.front();
      q.pop();
      for (pair<int, Vertex> p : compact_model.all)
        if (x_vals[k][curr][p.first] > 0){
          x_vals[k][curr][p.first] = 0;
          q.push(p.first);
        }        
    }
    //bfs to remove all edges not connected to the depot
    for (auto customer : compact_model.gvrp_instance.customers) {
      set<int> component, customersComponent;
      //checking for neighboring
      bool hasNeighboring = false;
      for (pair<int, Vertex> p : compact_model.all)
        if (x_vals[k][customer.id][p.first] > 0){
          hasNeighboring = true;
          break; 
        }
      //checking if bfs is needed
      if (!hasNeighboring)
        continue;
      q.push(customer.id);
      while (!q.empty()) {
        int curr = q.front();
        q.pop();
        component.insert(curr);
        //if it is a customer
        if (compact_model.customers.count(curr))
          customersComponent.insert(curr);
        for (pair<int, Vertex> p : compact_model.all)
          if (x_vals[k][curr][p.first] > 0){
            x_vals[k][curr][p.first] = 0;
            q.push(p.first);
          }
      }
      for (int k_ = 0; k_ < int(compact_model.gvrp_instance.customers.size()); k_++) { 
        for (int customer_ : customersComponent) {
          //getting lhs
          for (pair<int, Vertex> p2 : compact_model.all) {
            int a = p2.first;
            if (!component.count(a))
              for (int b : component) 
                lhs += compact_model.x[k_][a][b];
          }
          //getting rhs
          for (int b : component)
            lhs -= compact_model.x[k_][b][customer_];
          try {
            add(lhs >= 0).end();
          } catch(IloException& e) {
            cerr << "Exception while adding lazy constraint" << e.getMessage() << "\n";
            throw;
          }
          lhs.end();
          lhs = IloExpr(env);
        } 
      }
    }
    for (pair<int, Vertex> p : compact_model.all){
      int i = p.first;
      x_vals[k][i].end();
    }
  }
  x_vals.end();
}
