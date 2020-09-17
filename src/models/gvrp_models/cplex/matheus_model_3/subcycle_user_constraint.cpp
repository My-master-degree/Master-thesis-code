#include "utils/util.hpp"
#include "models/vertex.hpp"
#include "models/dsu.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/subcycle_user_constraint.hpp"

#include <unordered_set>
#include <queue>
#include <list>
#include <ilcplex/ilocplex.h>
#include <lemon/gomory_hu.h>
#include <lemon/concepts/graph.h>
#include <lemon/list_graph.h>
#include <iostream>

using namespace std;

using namespace utils;
using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models::cplex::matheus_model_3;
using namespace lemon;
using namespace lemon::concepts;

Subcycle_user_constraint::Subcycle_user_constraint (Matheus_model_3& matheus_model_3_) : User_constraint (matheus_model_3_) {}

IloCplex::CallbackI* Subcycle_user_constraint::duplicateCallback() const {
  return new(getEnv()) Subcycle_user_constraint (*this);
}

void Subcycle_user_constraint::main() {
  //node callback
  Depth const* const d = (Depth *) getNodeData();
  IloInt depth = d ? d->depth : 0;
  if (depth > 0) {
    abortCutLoop();
    return;
  }
  //setup
  const int sall = matheus_model_3.all.size();
  unordered_set<int> visited;
  IloEnv env = getEnv();
  IloExpr lhs(env);
  queue<int> q;
  //get values
  Matrix2DVal x_vals (env, sall);
  for (const pair<int, const Vertex *>& p : matheus_model_3.all) {
    x_vals[p.first] = IloNumArray (env, sall, 0, 1, IloNumVar::Float);
    getValues(x_vals[p.first], matheus_model_3.x[p.first]);
  }
  //bfs
  for (const pair<int, const Vertex *>& p : matheus_model_3.all) {
    int i = p.first;
    bool depotVisited = false;
    if (!visited.count(i) && i != matheus_model_3.instance.depot.id) {
      unordered_set<int> component;
      component.insert(i);
      visited.insert(i); 
      q.push(i);
      while (!q.empty()) {
        int curr = q.front();
        q.pop();
        if (curr == matheus_model_3.instance.depot.id) 
          depotVisited = true;
        for (const pair<int, const Vertex *>& p1 : matheus_model_3.all) {
          int j = p1.first;
          if ((x_vals[curr][j] > EPS || x_vals[j][curr]) > EPS && !visited.count(j)) {
            component.insert(j);
            visited.insert(j);
            q.push(j);
          }
        }
      }
      if (!depotVisited) {
        //get customers from component S
        list<int> customersComponent;
        for (int customer : matheus_model_3.customers)
          if (component.count(customer))
            customersComponent.push_back(customer);
        if (customersComponent.size() == 0)
          continue;
        vector<const Vertex *> vertices (customersComponent.size() + 1);
        vertices[0] = &matheus_model_3.instance.depot;
        int i = 1;
        for (int customer : customersComponent) {
          vertices[i] = matheus_model_3.all[customer];
          ++i;
        }
        //get n routes lbs
        const auto& closestsTimes = calculateClosestsGVRPCustomers(matheus_model_3.gvrpReducedGraphTimes, vertices);
        //get mst
        int improvedMSTNRoutesLB = int(ceil(calculateGvrpLBByImprovedMSTTime(vertices, closestsTimes, matheus_model_3.gvrpReducedGraphTimes)/matheus_model_3.instance.timeLimit));
        //bin packing
        int bppNRoutesLB = calculateGVRP_BPP_NRoutesLB(matheus_model_3.instance, vertices, closestsTimes, matheus_model_3.BPPTimeLimit);
        //get max
        int maxNRoutes = max(improvedMSTNRoutesLB, bppNRoutesLB);
        try {
          //in edges
          //getting lhs
          for (const pair<int, const Vertex *>& p1 : matheus_model_3.all) {
            int a = p1.first;
            if (!component.count(a))
              for (int b : component) 
                lhs += matheus_model_3.x[a][b];
          }
          //getting rhs
          lhs -= maxNRoutes;
          add(lhs >= 0.0).end();
          lhs.end();
          lhs = IloExpr (env);
          if (improvedMSTNRoutesLB == maxNRoutes) 
            ++matheus_model_3.nImprovedMSTNRoutesLB;
          if (bppNRoutesLB == maxNRoutes) 
            ++matheus_model_3.nBPPNRoutesLB;
        } catch(IloException& e) {
          cerr << "Exception while adding lazy constraint" << e.getMessage() << "\n";
          throw;
        }
      }
    }
  }
  //clean
  for (const pair<int, const Vertex *>& p : matheus_model_3.all){
    int i = p.first;
    x_vals[i].end();
  }
  x_vals.end();
}
