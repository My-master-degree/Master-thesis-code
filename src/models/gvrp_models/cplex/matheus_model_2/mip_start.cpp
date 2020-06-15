#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/matheus_model_2.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/mip_start.hpp"

#include <iostream>
#include <string>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN
using namespace std;
using namespace models;
using namespace models::gvrp_models::cplex::matheus_model_2;

Mip_start::Mip_start (const Gvrp_instance& instance, unsigned int time_limit, const Gvrp_solution& gvrp_solution_) : Matheus_model_2 (instance, time_limit), gvrp_solution (gvrp_solution_) {
}

void Mip_start::extraStepsAfterModelCreation () {
  try {
    Matheus_model_2::extraStepsAfterModelCreation();
    //setup
    IloNumArray e_vals = IloNumArray (env, c0.size() - 1, 0, instance.vehicleFuelCapacity, IloNumVar::Float),
                t_vals = IloNumArray (env, c0.size() - 1, 0, instance.timeLimit, IloNumVar::Float);
    x_vals = Matrix2DVal (env, c0.size());
    y_vals = Matrix3DVal (env, c0.size());
    //create vals
    for (size_t i = 0; i < c0.size(); ++i) {
      if (i > 0) {
        t_vals[i] = 0.0;
        e_vals[i] = 0.0;
      }
      //x vars
      x_vals[i] = IloNumArray (env, c0.size(), 0, 1, IloNumVar::Int);
      //y var
      y_vals[i] = Matrix2DVal (env, f0.size());
      for (size_t j = 0; j < c0.size(); ++j) 
        //x
        x_vals[i][j] = 0.0;
      for (size_t f = 0; f < f0.size(); ++f) {
        y_vals[i][f] = IloNumArray(env, c0.size(), 0, 1, IloNumVar::Int);
        for (size_t j = 0; j < c0.size(); ++j) 
          y_vals[i][f][j] = 0.0;
      }
    }
    //get values
    double currFuel, 
           currTime;
    for (const list<Vertex>& route : gvrp_solution.routes) {
      currFuel = instance.vehicleFuelCapacity;
      currTime = 0.0;
      list<Vertex>::const_iterator curr = route.begin(), 
        prev = curr;
      for (++curr; curr != route.end(); prev = curr, ++curr) {
        auto currIndex = customersC0Indexes.find(curr->id);
        int i = customersC0Indexes[prev->id];
        //is a customer
        if (currIndex != customersC0Indexes.end()) {
          int j = currIndex->second;
          x_vals[i][j] = 1;
          currFuel -= customersFuel(i, j);
          currTime += time(i, j);
          if (j != 0) {
            t_vals[j - 1] = currTime;
            e_vals[j - 1] = currFuel;
          }
        } else {
          //is an afs 
          int f = afssF0Indexes[curr->id];
          ++curr;
          int j = customersC0Indexes[curr->id];
          y_vals[i][f][j] = 1;
          currTime += time(i, f, j);
          currFuel = instance.vehicleFuelCapacity - afsToCustomerFuel(f, j);
          if (j != 0) {
            t_vals[j - 1] = currTime;
            e_vals[j - 1] = currFuel;
          }
        }
      }
    }
    //mip start
    IloCplex::MIPStartEffort effort = IloCplex::MIPStartCheckFeas;
    IloNumVarArray startVar(env);
    IloNumArray startVal(env);
    for (size_t i = 0; i < c0.size(); ++i) {
      if (i > 0) {
        //e
        startVar.add(e[i - 1]);
        startVal.add(e_vals[i - 1]);
        //t
        startVar.add(t[i - 1]);
        startVal.add(t_vals[i - 1]);
      }
      for (size_t j = 0; j < c0.size(); ++j) {
        //x
        startVar.add(x[i][j]);
        startVal.add(x_vals[i][j]);
      }
      //y 
      for (size_t f = 0; f < f0.size(); ++f) 
        for (size_t j = 0; j < c0.size(); ++j) {
          startVar.add(y[i][f][j]);
          startVal.add(y_vals[i][f][j]);
        }
    }
    cplex.addMIPStart (startVar, startVal, effort);
    //clean vals
    for (size_t i = 0; i < c0.size(); ++i) {
      x_vals[i].end();
      for (size_t f = 0; f < f0.size(); ++f) 
        y_vals[i][f].end();
      y_vals[i].end();
    }
    t_vals.end();
    e_vals.end();
    x_vals.end();
    y_vals.end();
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

