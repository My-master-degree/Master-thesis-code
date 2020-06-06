#include "models/vertex.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/cplex/matheus_model/greedy_lp_heuristic.hpp"
#include "models/gvrp_models/cplex/gvrp_lp_kk_heuristic.hpp"

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <list>
#include <ilcplex/ilocplex.h>
#include <iostream>

using namespace std;
using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::matheus_model;

Greedy_lp_heuristic::Greedy_lp_heuristic (Matheus_model& matheus_model) : Heuristic_callback (matheus_model) {}

IloCplex::CallbackI* Greedy_lp_heuristic::duplicateCallback() const {
  return new(getEnv()) Greedy_lp_heuristic (*this);
}

void Greedy_lp_heuristic::main() {
  //node callback
  Depth const* const d = (Depth *) getNodeData();
  IloInt depth = d ? d->depth : 0;
  if (depth > 0) {
//    abortCutLoop();
//    return;
  }
  //setup
  const size_t sc0 = matheus_model.c0.size(),
        sf0 = matheus_model.f0.size();
  IloEnv env = getEnv();
  IloExpr lhs (env);
  list<list<Vertex>> routes;
  list<Vertex> route;
  size_t curr;    
  unordered_set<int> customers;
  bool valid = true,
       found;
  double maxFirst,
         remainingFuel,
         remainingTime;
  int nextCustomer,
      nextAFS;
  //get values
  Matrix2DVal x_vals (env, sc0);
  Matrix3DVal y_vals (env, sc0);
  for (size_t i = 0; i < sc0; ++i) {
    x_vals[i] = IloNumArray (env, sc0, 0, 1, IloNumVar::Float);
    y_vals[i] = Matrix2DVal (env, sc0);
    getValues(x_vals[i], matheus_model.x[i]);
    for (size_t f = 0; f < sf0; ++f) {
      y_vals[i][f] = IloNumArray (env, sc0, 0, 1, IloNumVar::Float);
      getValues(y_vals[i][f], matheus_model.y[i][f]);
    }
  }
  //checking the depot neighboring
  while (true) {
    //setup
    maxFirst = -1;
    remainingFuel = matheus_model.instance.vehicleFuelCapacity;
    remainingTime = matheus_model.instance.timeLimit;
    nextCustomer = 0;
    nextAFS = 0;
    found = false;
    //get route beginning
    for (size_t i = 1; i < matheus_model.c0.size(); ++i) 
      if (!customers.count(i)) {
        if (x_vals[0][i] > maxFirst) {
          maxFirst = x_vals[0][i];
          nextCustomer = i;
          found = true;
        } else
          for (size_t f = 0; f < matheus_model.f0.size(); ++f)
            if (y_vals[0][f][i] > maxFirst) {
              maxFirst = y_vals[0][f][i];
              nextAFS = f;
              nextCustomer = i;
              found = true;
            }
      }
    if (!found)
      break;
    //update dss
    route.push_back(Vertex(*matheus_model.c0[0]));
    //get remaining route
    if (maxFirst != x_vals[0][nextCustomer]) {
      if (matheus_model.customerToAfsFuel(0, nextAFS) > remainingFuel || matheus_model.afsToCustomerFuel(nextAFS, nextCustomer) > remainingFuel || remainingTime - matheus_model.time(0, nextAFS, nextCustomer) < 0) {
        valid = false;
        break;
      }
      remainingFuel -= matheus_model.afsToCustomerFuel(nextAFS, nextCustomer);
      remainingTime -= matheus_model.time(0, nextAFS, nextCustomer);
      route.push_back(Vertex(*matheus_model.f0[nextAFS]));
    } else if (matheus_model.customersFuel(0, nextCustomer) > remainingFuel || remainingTime - matheus_model.time(0, nextCustomer) < 0) {
      valid = false;
      break;
    } else {
      remainingFuel -= matheus_model.afsToCustomerFuel(0, nextCustomer);
      remainingTime -= matheus_model.time(0, nextCustomer);
    }
    route.push_back(Vertex(*matheus_model.c0[nextCustomer]));
    customers.insert(nextCustomer);
    curr = nextCustomer;
    //dfs
    while (curr != 0) {
      maxFirst = 0;
      nextCustomer = 0;
      nextAFS = 0;
      found = false;
      for (size_t i = 0; i < matheus_model.c0.size(); ++i) 
        if (!customers.count(i)) {
          if (x_vals[curr][i] > maxFirst) {
            maxFirst = x_vals[curr][i];
            nextCustomer = i;
            found = true;
          } else
            for (size_t f = 0; f < matheus_model.f0.size(); ++f)
              if (y_vals[curr][f][i] > 0) {
                maxFirst = y_vals[curr][f][i];
                nextAFS = f;
                nextCustomer = i;
                found = true;
              }
        }
      //repeated customer
      if (!found) {
        valid = false;
        break;
      }
      //update dss
      if (maxFirst != x_vals[curr][nextCustomer]) {
        if (matheus_model.customerToAfsFuel(curr, nextAFS) > remainingFuel || matheus_model.afsToCustomerFuel(nextAFS, nextCustomer) > matheus_model.instance.vehicleFuelCapacity || remainingTime - matheus_model.time(curr, nextAFS, nextCustomer) < 0) {
          valid = false;
          break;
        }
        remainingFuel = matheus_model.instance.vehicleFuelCapacity - matheus_model.afsToCustomerFuel(nextAFS, nextCustomer);
        remainingTime -= matheus_model.time(curr, nextAFS, nextCustomer);
        route.push_back(Vertex(*matheus_model.f0[nextAFS]));
      } else if (matheus_model.customersFuel(curr, nextCustomer) > remainingFuel || remainingTime - matheus_model.time(curr, nextCustomer) < 0) {
        valid = false;
        break;
      } else {
        remainingFuel -= matheus_model.customersFuel(curr, nextCustomer);
        remainingTime -= matheus_model.time(curr, nextCustomer);
      }
      route.push_back(Vertex(*matheus_model.c0[nextCustomer]));
      if (nextCustomer != 0)
        customers.insert(nextCustomer);
      curr = nextCustomer;
    }
    if (!valid)
      break;
    routes.push_back(route);
    route = list<Vertex> ();
  }
  if (valid) {
    for (const list<Vertex>& route : routes) {
      for (const Vertex& v : route)
        cout<<v.id<<" ";
      cout<<endl;
    }
    exit(0);
  } 

}
