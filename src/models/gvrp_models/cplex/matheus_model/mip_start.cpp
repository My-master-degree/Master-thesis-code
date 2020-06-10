#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/cplex/matheus_model/mip_start.hpp"

#include <iostream>
#include <string>
#include <ilcplex/ilocplex.h>

ILOSTLBEGIN
using namespace std;
using namespace models;
using namespace models::gvrp_models::cplex::matheus_model;

Mip_start::Mip_start (const Gvrp_instance& instance, unsigned int time_limit, const Gvrp_solution& gvrp_solution_) : Matheus_model (instance, time_limit), gvrp_solution (gvrp_solution_) {
}

void Mip_start::extraStepsAfterModelCreation () {
  try {
    Matheus_model::extraStepsAfterModelCreation();
    //setup
    Matrix2DVal a_vals = Matrix2DVal (env, c0.size() - 1),
                u_vals = Matrix2DVal (env, c0.size()),
                v_vals = Matrix2DVal (env, c0.size() - 1),
                c_vals = Matrix2DVal (env, c0.size());
    x_vals = Matrix2DVal (env, c0.size());
    y_vals = Matrix3DVal (env, c0.size());
    //create vals
    for (size_t i = 0; i < c0.size(); ++i) {
      //x, u, v, c, e, and a vars
      x_vals[i] = IloNumArray (env, c0.size(), 0, 1, IloNumVar::Int);
      u_vals[i] = IloNumArray (env, c0.size(), 0, instance.timeLimit, IloNumVar::Float);
      c_vals[i] = IloNumArray (env, c0.size(), 0, int(instance.customers.size()) + 1, IloNumVar::Int);
      //v
      if (i > 0) {
          v_vals[i - 1] = IloNumArray (env, f0.size(), 0, instance.vehicleFuelCapacity, IloNumVar::Float);
          a_vals[i - 1] = IloNumArray (env, c0.size() - 1, 0, instance.vehicleFuelCapacity, IloNumVar::Float);
        for (size_t f = 0; f < f0.size(); ++f) 
          v_vals[i - 1][f] = 0.0;
      }
      for (size_t j = 0; j < c0.size(); ++j) {
        //c
        c_vals[i][j] = 0;
        //x
        x_vals[i][j] = 0.0;
        //u
        u_vals[i][j] = 0.0;
        //a
        if (j > 0 && i > 0) 
          a_vals[i - 1][j - 1] = 0.0;
      }
      //y var
      y_vals[i] = Matrix2DVal (env, f0.size());
      for (size_t f = 0; f < f0.size(); ++f) {
        y_vals[i][f] = IloNumArray(env, c0.size(), 0, 1, IloNumVar::Int);
        for (size_t j = 0; j < c0.size(); ++j) 
          y_vals[i][f][j] = 0.0;
      }
    }
    //get values
    double currFuel, 
           currTime;
    int nAFSs;
    for (const list<Vertex>& route : gvrp_solution.routes) {
      currFuel = instance.vehicleFuelCapacity;
      currTime = 0.0;
      nAFSs = 0;
      list<Vertex>::const_iterator curr = route.begin(), 
        prev = curr;
      for (++curr; curr != route.end(); prev = curr, ++curr) {
        auto currIndex = customersC0Indexes.find(curr->id);
        int i = customersC0Indexes[prev->id];
        //is a customer
        if (currIndex != customersC0Indexes.end()) {
          int j = currIndex->second;
          x_vals[i][j] = 1;
          u_vals[i][j] = currTime;
          c_vals[i][j] = nAFSs;
          if (i > 0 && j > 0) 
            a_vals[i - 1][j - 1] = currFuel - customersFuel(i, j);
          else if (i > 0) 
            v_vals[i - 1][0] = currFuel;
          currFuel -= customersFuel(i, j);
          currTime += time(i, j);
        } else {
          //is an afs 
          int f = afssF0Indexes[curr->id];
          ++curr;
          int j = customersC0Indexes[curr->id];
          y_vals[i][f][j] = 1;
          u_vals[i][j] = currTime;
          c_vals[i][j] = nAFSs;
          if (i > 0) 
            v_vals[i - 1][f] = currFuel;
          currTime += time(i, f, j);
          currFuel = instance.vehicleFuelCapacity - afsToCustomerFuel(f, j);
          ++nAFSs;
        }
      }
    }
    //mip start
    IloCplex::MIPStartEffort effort = IloCplex::MIPStartCheckFeas;
    IloNumVarArray startVar(env);
    IloNumArray startVal(env);
    for (size_t i = 0; i < c0.size(); ++i) {
      //v
      if (i > 0) 
        for (size_t f = 0; f < f0.size(); ++f) {
          startVar.add(v[i - 1][f]);
          startVal.add(v_vals[i - 1][f]);
        }
      for (size_t j = 0; j < c0.size(); ++j) {
        //x
        startVar.add(x[i][j]);
        startVal.add(x_vals[i][j]);
        //c
        startVar.add(c[i][j]);
        startVal.add(c_vals[i][j]);
        //u
        if (i != j) {
          startVar.add(u[i][j]);
          startVal.add(u_vals[i][j]);
        }
        //a
        if (j > 0 && i > 0) {
          startVar.add(a[i - 1][j - 1]);
          startVal.add(a_vals[i - 1][j - 1]);
        }
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
      if (i > 0) {
        a_vals[i - 1].end();
        v_vals[i - 1].end();
      }
      u_vals[i].end();
      x_vals[i].end();
      for (size_t f = 0; f < f0.size(); ++f) 
        y_vals[i][f].end();
      y_vals[i].end();
      c_vals[i].end();
    }
    x_vals.end();
    y_vals.end();
    a_vals.end();
    u_vals.end();
    v_vals.end();
    c_vals.end();
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

