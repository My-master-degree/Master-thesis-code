#include "models/vertex.hpp"
#include "models/cubic_model/cubic_model.hpp"
#include "models/cubic_model/mip_start_cubic_model.hpp"

#include <iostream>
#include <string>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN
using namespace std;
using namespace models;
using namespace models::cubic_model;

Mip_start_cubic_model::Mip_start_cubic_model (Gvrp_instance& gvrp_instance, unsigned int time_limit, Gvrp_solution& gvrp_solution_) : Cubic_model (gvrp_instance, time_limit), gvrp_solution (gvrp_solution_) {
  gvrp_instance.nRoutes = gvrp_instance.customers.size();
}

void Mip_start_cubic_model::extraStepsAfterModelCreation () {
  try {
    Cubic_model::extraStepsAfterModelCreation();
    //setup
    IloNumArray e_vals (env, all.size(), 0, gvrp_instance.vehicleFuelCapacity, IloNumVar::Float);
    x_vals = Matrix3DVal (env, gvrp_instance.customers.size());
    //create vals
    for (size_t k = 0; k < gvrp_instance.customers.size(); k++) {
      x_vals[k] = IloArray<IloNumArray> (env, all.size());
      for (pair<int, Vertex> p : all){
        int i = p.first;
        x_vals[k][i] = IloNumArray (env, all.size(), 0, ub_edge_visit, IloNumVar::Int);
        for (pair<int, Vertex> p1 : all)
          x_vals[k][i][p1.first] = 0;
      }
    }
    for (pair<int, Vertex> p : all) 
      e_vals[p.first] = gvrp_instance.vehicleFuelCapacity;
    //get values
    int k = 0, 
        i;
    double fuel;
    for (list<Vertex> route : gvrp_solution.routes) {
      auto node = route.begin();
      i = node->id;
      node++;
      fuel = gvrp_instance.vehicleFuelCapacity;
      for (; node != route.end(); i = node->id, node++) {
        //update fuel
        if (customers.count(node->id)) {
          fuel -= gvrp_instance.distances[i][node->id] * gvrp_instance.vehicleFuelConsumptionRate;
          e_vals[node->id] = fuel;
        } else
          fuel = gvrp_instance.vehicleFuelCapacity;
        x_vals[k][i][node->id] = 1;
      }
      k++;
    }
    //mip start
//    IloCplex::MIPStartEffort effort = IloCplex::MIPStartCheckFeas;
    IloNumVarArray startVar(env);
    IloNumArray startVal(env);
    for (pair<int, Vertex> p : all) {
      startVar.add(e[p.first]);
      startVal.add(e_vals[p.first]);
    }
    for (size_t k = 0; k < gvrp_instance.customers.size(); k++)
      for (pair<int, Vertex> p : all) {
        int i = p.first;
        for (pair<int, Vertex> p1 : all) {
          int j = p1.first;
          startVar.add(x[k][i][j]);
          startVal.add(x_vals[k][i][j]);
        }
      }
    cplex.addMIPStart (startVar, startVal);
    //clean vals
    for (size_t k = 0; k < gvrp_instance.customers.size(); x_vals[k++].end()) 
      for (pair<int, Vertex> p : all)
        x_vals[k][p.first].end();
    x_vals.end();
    e_vals.end();
    startVar.end();
    startVal.end();
  } catch (IloException& e) {
    throw e;
  } catch (string& s) {
    throw s;
  } catch (...) {
    throw string("Error in setting MIP start solution");
  }
}

