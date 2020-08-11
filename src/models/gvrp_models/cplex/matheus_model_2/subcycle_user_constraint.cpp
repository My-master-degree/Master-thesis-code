#include "models/vertex.hpp"
#include "models/dsu.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/gvrp_lp_kk_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/matheus_model_2.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_2/subcycle_user_constraint.hpp"
#include "models/bpp_models/bpp_instance.hpp"
#include "models/bpp_models/cplex/bpp_model.hpp"
#include "utils/util.hpp"

#include <unordered_map>
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
using namespace models::bpp_models;
using namespace models::bpp_models::cplex;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::matheus_model_2;
using namespace lemon;
using namespace lemon::concepts;

Subcycle_user_constraint::Subcycle_user_constraint (Matheus_model_2& matheus_model_2_) : User_constraint (matheus_model_2_) {}

IloCplex::CallbackI* Subcycle_user_constraint::duplicateCallback() const {
  return new(getEnv()) Subcycle_user_constraint (*this);
}

void Subcycle_user_constraint::main() {
  //node callback
  Depth const* const d = (Depth *) getNodeData();
  IloInt depth = d ? d->depth : 0;
  if (depth > matheus_model_2.levelSubcycleCallback) {
    abortCutLoop();
    return;
  }
  //setup
  const int sc0 = matheus_model_2.c0.size(),
        sf0 = matheus_model_2.f0.size();
  int bppNRoutesLB, 
      improvedMSTNRoutesLB, 
      maxNRoutes;
  DSU dsu (sc0);
  IloEnv env = getEnv();
  IloExpr lhs (env);
  vector<ListGraph::Node> nodes (sc0);
  ListGraph graph;
  unordered_multimap<int, int> subcomponents (sc0);
  unordered_set<int> component;
  vector<bool> visited (sc0, false);
  list<unordered_set<int>> components;
  queue<int> q;
  //creating nodes
  for (int i = 0; i < sc0; ++i) 
    nodes[i] = graph.addNode();
  ListGraph::EdgeMap<double> weight(graph); 
  //get values
  Matrix2DVal x_vals (env, sc0);
  Matrix3DVal y_vals (env, sc0);
  for (int i = 0; i < sc0; ++i) {
    x_vals[i] = IloNumArray (env, sc0, 0, 1, IloNumVar::Float);
    y_vals[i] = Matrix2DVal (env, sc0);
    getValues(x_vals[i], matheus_model_2.x[i]);
    for (int f = 0; f < sf0; ++f) {
      y_vals[i][f] = IloNumArray (env, sc0, 0, 1, IloNumVar::Float);
      getValues(y_vals[i][f], matheus_model_2.y[i][f]);
    }
    for (int j = 0; j < sc0; ++j) {
      double cost = 0.0;
      if (x_vals[i][j] > EPS)
        cost += x_vals[i][j];
      for (int f = 0; f < sf0; ++f) 
        if (y_vals[i][f][j] > EPS)
          cost += y_vals[i][f][j];
      weight[graph.addEdge(nodes[i], nodes[j])] = cost;
    }
  }
  //gh
  GomoryHu<ListGraph, ListGraph::EdgeMap<double> > gh (graph, weight);
  gh.run();
  //get subcycles
  for (int i = 0; i < sc0; ++i) 
    for (int j = 0; j < sc0; ++j) 
      if (gh.minCutValue(nodes[j], nodes[j]) >= 2.0 - EPS) 
        dsu.join(i, j);
  //get subcomponents
  for (int i = 0; i < sc0; ++i) 
    subcomponents.insert(make_pair(dsu.findSet(i), i));
  //store components
  unordered_multimap<int, int>::iterator it = subcomponents.begin();
  int j = it->first;
  for (; it != subcomponents.end(); it++) {
    if (it->first != j) {
      j = it->first;
      components.push_back(component);
      component.clear();
    } 
    component.insert(it->second);
  }
  components.push_back(component);
  //end of multimap 
  //inequallitites
  for (const unordered_set<int>& S : components) 
    if (!S.count(0)) {
      //\sum_{v_i \in V'\S} \sum_{v_j \in S} x_{ij} + \sum_{v_f \in F_0} y_{ifj} \geqslant 1 
      //lhs
      for (int i = 0; i < sc0; ++i) 
        if (!S.count(i)) 
          for (int j : S) {
            lhs += matheus_model_2.x[i][j];
            for (int f = 0; f < sf0; ++f)
              lhs += matheus_model_2.y[i][f][j];
          }
      //rhs
      const int sS = S.size();
      vector<const Vertex *> vertices (sS + 1);
      int j = 0;
      for (int i : S) {
        vertices[j] = matheus_model_2.c0[i];
        ++j;
      }
      vertices[j] = matheus_model_2.c0[0];
      //get n routes lbs
      const auto& closestsTimes = calculateClosestsGVRPCustomers(matheus_model_2.gvrpReducedGraphTimes, vertices);
      //get mst
      improvedMSTNRoutesLB = int(ceil(calculateGvrpLBByImprovedMSTTime(vertices, closestsTimes, matheus_model_2.gvrpReducedGraphTimes)/matheus_model_2.instance.timeLimit));
      //bin packing
      bppNRoutesLB = calculateGVRP_BPP_NRoutesLB(matheus_model_2.instance, vertices, closestsTimes, matheus_model_2.BPPTimeLimit);
      maxNRoutes = max(improvedMSTNRoutesLB, bppNRoutesLB);
      if (improvedMSTNRoutesLB == maxNRoutes) 
        ++matheus_model_2.nImprovedMSTNRoutesLB;
      if (bppNRoutesLB == maxNRoutes) 
        ++matheus_model_2.nBPPNRoutesLB;
      lhs -= maxNRoutes;
      try {
        add(lhs >= 0).end();
      } catch(IloException& e) {
        cerr << "Exception while adding user constraint" << e.getMessage() << "\n";
        throw;
      }
      lhs.end();
      lhs = IloExpr(env);
    }
  //clean
  for (int i = 0; i < sc0; ++i) {
    x_vals[i].end();
    for (int f = 0; f < sf0; ++f) 
      y_vals[i][f].end();
    y_vals[i].end();
  }
  x_vals.end();
  y_vals.end();
}
