#include "models/vertex.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/cplex/matheus_model/greedy_lp_heuristic.hpp"
#include "models/gvrp_models/cplex/gvrp_lp_kk_heuristic.hpp"

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <list>
#include <ilcplex/ilocplex.h>
#include <iostream>

ILOSTLBEGIN

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
  try {
    //node callback
    Depth const* const d = (Depth *) getNodeData();
    IloInt depth = d ? d->depth : 0;
    if (depth > matheus_model.nLevelsGreedyLPHeuristic) 
      return;
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
         found,
         frac_sol = false;
    double maxFirst,
           remainingFuel,
           remainingTime,
           cost = 0;
    int nextCustomer,
        nextAFS;
    matheus_model.x_vals = Matrix2DVal (env, sc0);
    matheus_model.y_vals = Matrix3DVal (env, sc0);
    //get values
    for (size_t i = 0; i < sc0; ++i) {
      matheus_model.x_vals[i] = IloNumArray (env, sc0, 0, 1, IloNumVar::Float);
      matheus_model.y_vals[i] = Matrix2DVal (env, sc0);
      getValues(matheus_model.x_vals[i], matheus_model.x[i]);
      //check y values
      if (!frac_sol)
        for (size_t j = 0; j < sc0; ++j)
          if (abs(round(matheus_model.x_vals[i][j]) - matheus_model.x_vals[i][j]) > matheus_model.INTEGRALITY_TOL) 
            frac_sol = true;
      //get y valsd
      for (size_t f = 0; f < sf0; ++f) {
        matheus_model.y_vals[i][f] = IloNumArray (env, sc0, 0, 1, IloNumVar::Float);
        getValues(matheus_model.y_vals[i][f], matheus_model.y[i][f]);
        //check y values
        if (!frac_sol)
          for (size_t j = 0; j < sc0; ++j)
            if (abs(round(matheus_model.y_vals[i][f][j]) - matheus_model.y_vals[i][f][j]) > matheus_model.INTEGRALITY_TOL) 
              frac_sol = true;
      }
    }
    if (!frac_sol) 
      return;
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
          if (matheus_model.x_vals[0][i] > maxFirst) {
            maxFirst = matheus_model.x_vals[0][i];
            nextCustomer = i;
            found = true;
          } else
            for (size_t f = 0; f < matheus_model.f0.size(); ++f)
              if (matheus_model.y_vals[0][f][i] > maxFirst) {
                maxFirst = matheus_model.y_vals[0][f][i];
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
      if (maxFirst != matheus_model.x_vals[0][nextCustomer]) {
        if (matheus_model.customerToAfsFuel(0, nextAFS) > remainingFuel || matheus_model.afsToCustomerFuel(nextAFS, nextCustomer) > remainingFuel || remainingTime - matheus_model.time(0, nextAFS, nextCustomer) < 0) {
          valid = false;
          break;
        }
        remainingFuel -= matheus_model.afsToCustomerFuel(nextAFS, nextCustomer);
        remainingTime -= matheus_model.time(0, nextAFS, nextCustomer);
        cost += matheus_model.instance.distances[route.back().id][matheus_model.f0[nextAFS]->id];
        route.push_back(Vertex(*matheus_model.f0[nextAFS]));
      } else if (matheus_model.customersFuel(0, nextCustomer) > remainingFuel || remainingTime - matheus_model.time(0, nextCustomer) < 0) {
        valid = false;
        break;
      } else {
        remainingFuel -= matheus_model.afsToCustomerFuel(0, nextCustomer);
        remainingTime -= matheus_model.time(0, nextCustomer);
      }
      cost += matheus_model.instance.distances[route.back().id][matheus_model.c0[nextCustomer]->id];
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
            if (matheus_model.x_vals[curr][i] > maxFirst) {
              maxFirst = matheus_model.x_vals[curr][i];
              nextCustomer = i;
              found = true;
            } else
              for (size_t f = 0; f < matheus_model.f0.size(); ++f)
                if (matheus_model.y_vals[curr][f][i] > 0) {
                  maxFirst = matheus_model.y_vals[curr][f][i];
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
        if (maxFirst != matheus_model.x_vals[curr][nextCustomer]) {
          if (matheus_model.customerToAfsFuel(curr, nextAFS) > remainingFuel || matheus_model.afsToCustomerFuel(nextAFS, nextCustomer) > matheus_model.instance.vehicleFuelCapacity || remainingTime - matheus_model.time(curr, nextAFS, nextCustomer) < 0) {
            valid = false;
            break;
          }
          remainingFuel = matheus_model.instance.vehicleFuelCapacity - matheus_model.afsToCustomerFuel(nextAFS, nextCustomer);
          remainingTime -= matheus_model.time(curr, nextAFS, nextCustomer);
          cost += matheus_model.instance.distances[route.back().id][matheus_model.f0[nextAFS]->id];
          route.push_back(Vertex(*matheus_model.f0[nextAFS]));
        } else if (matheus_model.customersFuel(curr, nextCustomer) > remainingFuel || remainingTime - matheus_model.time(curr, nextCustomer) < 0) {
          valid = false;
          break;
        } else {
          remainingFuel -= matheus_model.customersFuel(curr, nextCustomer);
          remainingTime -= matheus_model.time(curr, nextCustomer);
        }
        cost += matheus_model.instance.distances[route.back().id][matheus_model.c0[nextCustomer]->id];
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
    //better solution found
    if (valid && cost - getIncumbentObjValue() < -1e-6) {
      ++matheus_model.nGreedyLP;
      cout.precision(17);
      cout<<getIncumbentObjValue()<<" to "<<cost<<endl;
      //set new solution
      //reset all the values
      Matrix2DVal a_vals = Matrix2DVal (matheus_model.env, sc0 - 1),
                  u_vals = Matrix2DVal (matheus_model.env, sc0),
                  v_vals = Matrix2DVal (matheus_model.env, sc0 - 1);
      matheus_model.x_vals = Matrix2DVal (matheus_model.env, sc0);
      matheus_model.y_vals = Matrix3DVal (matheus_model.env, sc0);
      //create vals
      for (size_t i = 0; i < sc0; ++i) {
        //x, u, v, and a vars
        matheus_model.x_vals[i] = IloNumArray (matheus_model.env, sc0, 0, 1, IloNumVar::Int);
        u_vals[i] = IloNumArray (matheus_model.env, sc0, 0, matheus_model.instance.timeLimit, IloNumVar::Float);
        //v
        if (i > 0) {
          v_vals[i - 1] = IloNumArray (matheus_model.env, sf0, 0, matheus_model.instance.vehicleFuelCapacity, IloNumVar::Float);
          a_vals[i - 1] = IloNumArray (matheus_model.env, sc0 - 1, 0, matheus_model.instance.vehicleFuelCapacity, IloNumVar::Float);
          for (size_t f = 0; f < sf0; ++f) 
            v_vals[i - 1][f] = 0.0;
        }
        for (size_t j = 0; j < sc0; ++j) {
          //x
          matheus_model.x_vals[i][j] = 0.0;
          //u
          u_vals[i][j] = 0.0;
          //a
          if (j > 0 && i > 0) 
            a_vals[i - 1][j - 1] = 0.0;
        }
        //y var
        matheus_model.y_vals[i] = Matrix2DVal (env, sf0);
        for (size_t f = 0; f < sf0; ++f) {
          matheus_model.y_vals[i][f] = IloNumArray(env, sc0, 0, 1, IloNumVar::Int);
          for (size_t j = 0; j < sc0; ++j) 
            matheus_model.y_vals[i][f][j] = 0.0;
        }
      }
      //get values
      IloNumVarArray vars (matheus_model.env);
      IloNumArray vals (matheus_model.env);
      double currFuel, 
             currTime;
      for (const list<Vertex>& route : routes) {
        currFuel = matheus_model.instance.vehicleFuelCapacity;
        currTime = 0.0;
        list<Vertex>::const_iterator curr = route.begin(), 
          prev = curr;
        for (++curr; curr != route.end(); prev = curr, ++curr) {
          auto currIndex = matheus_model.customersC0Indexes.find(curr->id);
          int i = matheus_model.customersC0Indexes[prev->id];
          //is a customer
          if (currIndex != matheus_model.customersC0Indexes.end()) {
            int j = currIndex->second;
            matheus_model.x_vals[i][j] = 1;
            u_vals[i][j] = currTime;
            if (i > 0 && j > 0) 
              a_vals[i - 1][j - 1] = currFuel - matheus_model.customersFuel(i, j);
            else if (i > 0) 
              v_vals[i - 1][0] = currFuel;
            currFuel -= matheus_model.customersFuel(i, j);
            currTime += matheus_model.time(i, j);
          } else {
            //is an afs 
            int f = matheus_model.afssF0Indexes[curr->id];
            ++curr;
            int j = matheus_model.customersC0Indexes[curr->id];
            matheus_model.y_vals[i][f][j] = 1;
            u_vals[i][j] = currTime;
            if (i > 0) 
              v_vals[i - 1][f] = currFuel;
            currTime += matheus_model.time(i, f, j);
            currFuel = matheus_model.instance.vehicleFuelCapacity - matheus_model.afsToCustomerFuel(f, j);
          }
        }
      }
      //set values
      for (size_t i = 0; i < sc0; ++i) {
        //v
        if (i > 0) 
          for (size_t f = 0; f < sf0; ++f) {
            vars.add(matheus_model.v[i - 1][f]);
            vals.add(v_vals[i - 1][f]);
          }
        for (size_t j = 0; j < sc0; ++j) {
          //x
          vars.add(matheus_model.x[i][j]);
          vals.add(matheus_model.x_vals[i][j]);
          //u
          if (i != j) {
            vars.add(matheus_model.u[i][j]);
            vals.add(u_vals[i][j]);
          }
          //a
          if (j > 0 && i > 0) {
            vars.add(matheus_model.a[i - 1][j - 1]);
            vals.add(a_vals[i - 1][j - 1]);
          }
        }
        //y 
        for (size_t f = 0; f < sf0; ++f) 
          for (size_t j = 0; j < sc0; ++j) {
            vars.add(matheus_model.y[i][f][j]);
            vals.add(matheus_model.y_vals[i][f][j]);
          }
      }
      setSolution(vars, vals, cost);
      //clean vals
      for (size_t i = 0; i < sc0; ++i) {
        if (i > 0) {
          a_vals[i - 1].end();
          v_vals[i - 1].end();
        }
        u_vals[i].end();
        matheus_model.x_vals[i].end();
        for (size_t f = 0; f < sf0; ++f) 
          matheus_model.y_vals[i][f].end();
        matheus_model.y_vals[i].end();
      }
      matheus_model.x_vals.end();
      matheus_model.y_vals.end();
      a_vals.end();
      u_vals.end();
      v_vals.end();
      vars.end();
      vals.end();
    } 
  } catch (IloException& e) {
    throw e;
  } catch (string& s) {
    throw s;
  } catch (...) {
    throw string("Error in setting MIP start solution");
  }
}
