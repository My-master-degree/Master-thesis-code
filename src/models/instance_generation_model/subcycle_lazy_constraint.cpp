#include "models/instance_generation_model/lazy_constraint.hpp"
#include "models/instance_generation_model/instance_generation_model.hpp"
#include "models/instance_generation_model/subcycle_lazy_constraint.hpp"

#include <set>
#include <list>
#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models::instance_generation_model;

Subcycle_lazy_constraint::Subcycle_lazy_constraint (Instance_generation_model& instance_generation_model_) : Lazy_constraint (instance_generation_model_) {}

IloCplex::CallbackI* Subcycle_lazy_constraint::duplicateCallback() const {
  return new(getEnv()) Subcycle_lazy_constraint (*this);
}

void Subcycle_lazy_constraint::main() {
  IloEnv env = getEnv();
  IloExpr lhs(env);
  size_t i,
         j, 
         curr, 
         next;
  set<int> component;
  list<set<int>> components;
  //get values
  instance_generation_model.fillVals();
  //get components
  for (i = 0; i < instance_generation_model.sNodes; i++)
    //if i is a facility
    if (instance_generation_model.z_vals[i] > 0) {
      //dfs
      next = i;
      do {
        curr = next;
        component.insert(curr);
        for (j = 0; j < instance_generation_model.sNodes; j++)
          if (instance_generation_model.x_vals[curr][j] > 0) {
            instance_generation_model.x_vals[curr][j] = 0;
            next = j;
            break;
          }
      } while (curr != next);
      components.push_back(component);
      component = set<int> ();
    }
  //inequallitites
  for (const set<int>& T : components) 
    for (const set<int>& S : components) 
      if (*T.begin() != *S.begin()) 
        //for ech component facility        
        //\sum_{v_j \in T} \sum_{v_k \in S} y_{jk} + y_{kj} \geqslant z_{k*}, \forall k* \in S
        for (int k_ : S) {
          for (int j : T) 
            for (int k : S) 
              lhs += instance_generation_model.x[k][j] + instance_generation_model.x[j][k];
          lhs -= instance_generation_model.z[k_];
          try {
            add(lhs >= 0).end();
          } catch(IloException& e) {
            cerr << "Exception while adding lazy constraint" << e.getMessage() << "\n";
            throw;
          }
          lhs.end();
          lhs = IloExpr(env);
        }
  //end vals
  for (i = 0; i < instance_generation_model.sNodes; i++)
    instance_generation_model.x_vals[i].end();
  instance_generation_model.x_vals.end();
  instance_generation_model.z_vals.end();
}
