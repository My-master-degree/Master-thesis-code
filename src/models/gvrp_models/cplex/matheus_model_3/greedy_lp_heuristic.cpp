#include "models/vertex.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/greedy_lp_heuristic.hpp"
#include "models/gvrp_models/cplex/gvrp_lp_kk_heuristic.hpp"
#include "models/gvrp_models/local_searchs/fsRemove.hpp"

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
using namespace models::gvrp_models;
using namespace models::gvrp_models::local_searchs;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::matheus_model_3;

Greedy_lp_heuristic::Greedy_lp_heuristic (Matheus_model_3& matheus_model_3) : Heuristic_callback (matheus_model_3) {}

IloCplex::CallbackI* Greedy_lp_heuristic::duplicateCallback() const {
  return new(getEnv()) Greedy_lp_heuristic (*this);
}

void Greedy_lp_heuristic::main() {
  try {
    //node callback
    Depth const* const d = (Depth *) getNodeData();
    IloInt depth = d ? d->depth : 0;
    if (depth > 10 && depth%10 == 0) 
      return;
    //setup
    const int sall = matheus_model_3.all.size();
    int depot = matheus_model_3.instance.depot.id;
    IloEnv env = getEnv();
    IloExpr lhs (env);
    list<list<Vertex>> routes;
    list<Vertex> route;
    unordered_set<int> visitedNodes;
    bool valid = true,
         frac_sol = false;
    double maxVal,
           remainingFuel,
           remainingTime,
           cost = 0;
    int next,
        curr;
    matheus_model_3.x_vals = Matrix2DVal (env, sall);
    //get values
    for (const pair<int, const Vertex *>& p : matheus_model_3.all) {
      int i = p.first;
      matheus_model_3.x_vals[i] = IloNumArray (env, sall, 0, 1, IloNumVar::Float);
      getValues(matheus_model_3.x_vals[i], matheus_model_3.x[i]);
      //check y values
      if (!frac_sol)
        for (int j = 0; j < sall; ++j)
          if (abs(round(matheus_model_3.x_vals[i][j]) - matheus_model_3.x_vals[i][j]) > matheus_model_3.INTEGRALITY_TOL) 
            frac_sol = true;
    }
    if (!frac_sol) 
      return;
    //checking the depot neighboring
    for (int i = 0; i < sall; ++i) {
      if (matheus_model_3.x_vals[depot][i] > matheus_model_3.INTEGRALITY_TOL && matheus_model_3.fuel(depot, i) <= matheus_model_3.instance.vehicleFuelCapacity && matheus_model_3.instance.timeLimit - matheus_model_3.time(depot, i) - matheus_model_3.all[i]->serviceTime >= 0.0 && !visitedNodes.count(i)) {
        //setup
        remainingFuel = matheus_model_3.instance.vehicleFuelCapacity;
        remainingTime = matheus_model_3.instance.timeLimit;
        route.push_back(Vertex(*matheus_model_3.all[depot]));
        route.push_back(Vertex(*matheus_model_3.all[next]));
        curr = i;
        next = i;
        //dfs
        do {
          maxVal = -1.0;
          for (int j = 0; j < sall; ++j)
            if (matheus_model_3.x_vals[curr][j] > maxVal) {
              next = j;
              maxVal = matheus_model_3.x_vals[curr][j];
            }
          if (maxVal > 0.0 && matheus_model_3.fuel(curr, next) <= remainingFuel && remainingTime - matheus_model_3.time(curr, next) - matheus_model_3.all[curr]->serviceTime >= 0.0 && !visitedNodes.count(next)) {
            remainingFuel -= matheus_model_3.instance.fuel(curr, next);
            remainingTime -= (matheus_model_3.instance.time(curr, next) + matheus_model_3.all[curr]->serviceTime);
            route.push_back(Vertex(*matheus_model_3.all[curr]));
            curr = next;
            visitedNodes.insert(curr);
            if (matheus_model_3.afs_dummies.count(curr))
              remainingFuel = matheus_model_3.instance.vehicleFuelCapacity;
          } else {
            valid = false;
            break; 
          }
        } while (curr != depot);
        if (valid) {
          route.push_back(Vertex(*matheus_model_3.all[curr]));
          routes.push_back(route);
        }
        route = list<Vertex> ();
      }
    }
    //better solution found
    valid = true;
    for (int customer : matheus_model_3.customers)
      if (!visitedNodes.count(customer)) {
        valid = false;
        break;
      }
    if (valid) {
      Gvrp_solution gvrp_solution (routes, matheus_model_3.instance);
      if (gvrp_solution.getInfeasibilities().size() > 0)
        return;
      FsRemove fsRemove (matheus_model_3.instance, gvrp_solution);
      gvrp_solution = fsRemove.run();
      routes = gvrp_solution.routes;
      cost = gvrp_solution.calculateCost();
      if (cost - getIncumbentObjValue() < -1e-6) {
        ++matheus_model_3.nGreedyLP;
        cout.precision(17);
        cout<<getIncumbentObjValue()<<" to "<<cost<<endl;
        //set new solution
        //reset all the values
        IloNumArray t_vals = IloNumArray (matheus_model_3.env, sall, 0, 1, IloNumVar::Float);
        unordered_map<int, IloNum> e_vals;
        matheus_model_3.x_vals = Matrix2DVal (matheus_model_3.env, sall);
        //t
        for (int i = 0; i < sall; ++i) 
          t_vals[i] = 0.0;
        //x
        for (int i = 0; i < sall; ++i)
          for (int j = 0; j < sall; ++j)
            matheus_model_3.x_vals[i][j] = 0;
        //get values
        unordered_map<int, list<int>> afs_dummies = matheus_model_3.afs_dummies;
        IloNumVarArray vars (matheus_model_3.env);
        IloNumArray vals (matheus_model_3.env);
        double currFuel, 
               currTime;
        for (const list<Vertex>& route : routes) {
          currFuel = matheus_model_3.instance.vehicleFuelCapacity;
          currTime = 0.0;
          list<Vertex>::const_iterator curr = route.begin(), 
            prev = curr;
          int i = prev->id;
          for (++curr; curr != route.end(); prev = curr, ++curr) {
            int j = curr->id;
            currTime += matheus_model_3.time(i, j);
            currFuel -= matheus_model_3.fuel(i, j);
            t_vals[j] = currTime;
            //is a customer
            if (matheus_model_3.customers.count(j)) 
              e_vals[j] = currFuel;
            else {
              //is an afs
              j = afs_dummies[j].back();
              afs_dummies[j].pop_back();
              currFuel = matheus_model_3.instance.vehicleFuelCapacity;
            }
            matheus_model_3.x_vals[i][j] = 1;
            i = j;
          }
        }


        /*
           Gvrp_solution gvrp_solution (routes, matheus_model_3.instance);
           cout<<gvrp_solution<<endl;
           cout<<" ";
           for (int i = 0; i < matheus_model_3.c0.size(); ++i){
           cout<<" ";
           if (i <=9)
           cout<<" ";
           cout<<i;
           }
           cout<<endl;
           for (int i = 0; i < matheus_model_3.c0.size(); ++i){
           cout<<i<<" ";
           if (i <= 9)
           cout<<" ";
           for (int j = 0; j < matheus_model_3.c0.size(); ++j) {
           cout<<abs(matheus_model_3.x_vals[i][j])<<"  ";
           }
           cout<<endl;
           }
           for (int f = 0; f < matheus_model_3.f0.size(); ++f){
           cout<<"AFS: "<<f<<endl;
           cout<<" ";
           for (int i = 0; i < matheus_model_3.c0.size(); ++i){
           cout<<" ";
           if (i <=9)
           cout<<" ";
           cout<<i;
           }
           cout<<endl;
           for (int i = 0; i < matheus_model_3.c0.size(); ++i){
           cout<<i<<" ";
           if (i <= 9)
           cout<<" ";
           for (int j = 0; j < matheus_model_3.c0.size(); ++j)
           cout<<abs(matheus_model_3.y_vals[i][f][j])<<"  ";
           cout<<endl;
           }
           }
           for (int i = 0; i < matheus_model_3.c0.size(); ++i)
           cout<<i<<": "<<matheus_model_3.c0[i]->id<<endl;
           */

        //set values
        for (int customer : matheus_model_3.customers) {
          vars.add(matheus_model_3.e[customer]);
          vals.add(e_vals[customer]);
        }
        for (int i = 0; i < sall; ++i) {
          //t
          vars.add(matheus_model_3.t[i]);
          vals.add(t_vals[i]);
          for (int j = 0; j < sall; ++j) {
            //x
            vars.add(matheus_model_3.x[i][j]);
            vals.add(matheus_model_3.x_vals[i][j]);
          }
        }
        setSolution(vars, vals, cost);
        //clean vals
        t_vals.end();
        vars.end();
        vals.end();
      } 
    }
    //clean vals
    for (int i = 0; i < sall; ++i) 
      matheus_model_3.x_vals[i].end();
    matheus_model_3.x_vals.end();
  } catch (IloException& e) {
    throw e;
  } catch (string& s) {
    throw s;
  } catch (...) {
    throw string("Error in setting MIP start solution");
  }
}
