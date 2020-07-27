#include "utils/util.hpp"
#include "models/vertex.hpp"
#include "models/dsu.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/matheus_model_3.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model_3/subcycle_user_constraint.hpp"

#include <set>
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
  const size_t sall = matheus_model_3.all.size();
  DSU dsu (sall);
  IloEnv env = getEnv();
  IloExpr lhs(env);
  vector<ListGraph::Node> nodes (sall);
  ListGraph graph;
  multimap<int, int> subcomponents;
  set<int> component;
  list<set<int>> components;
  queue<int> q;
  //get values
  Matrix2DVal x_vals (env, sall);
  for (const pair<int, const Vertex *>& p : matheus_model_3.all) {
    x_vals[p.first] = IloNumArray (env, sall, 0, 1, IloNumVar::Float);
    getValues(x_vals[p.first], matheus_model_3.x[p.first]);
  }
  //creating nodes
  for (const pair<int, const Vertex *>& p : matheus_model_3.all) 
    nodes[p.first] = graph.addNode();
  //get components
  set<int> visited;
  for (int customer : matheus_model_3.customers) {
    if (visited.count(customer))
      continue;
    ListGraph::EdgeMap<double> weight(graph); 
    //bfs
    component.insert(customer);
    q.push(customer);
    while (!q.empty()) {
      int curr = q.front();
      visited.insert(curr);
      q.pop();
      for (const pair<int, const Vertex *>& p : matheus_model_3.all) 
        if (x_vals[curr][p.first] > EPS || x_vals[p.first][curr] > EPS) {
          if (!component.count(p.first)) {
            component.insert(p.first);
            q.push(p.first);
          }
          weight[graph.addEdge(nodes[curr], nodes[p.first])] = x_vals[curr][p.first] > EPS ? x_vals[curr][p.first] : 0;
          weight[graph.addEdge(nodes[p.first], nodes[curr])] = x_vals[p.first][curr] > EPS ? x_vals[p.first][curr] : 0;
          x_vals[p.first][curr] = 0;
          x_vals[curr][p.first] = 0;
        } 
    }
    //gh
    GomoryHu<ListGraph, ListGraph::EdgeMap<double> > gh (graph, weight);
    gh.run();
    //get subcycles
    for (int i : component) 
      for (int j : component) 
        if (gh.minCutValue(nodes[j], nodes[j]) >= 2.0) 
          dsu.join(i, j);
    component.clear();
  }
  //get subcomponents
  for (const pair<int, const Vertex *>& p : matheus_model_3.all) 
    subcomponents.insert(make_pair(dsu.findSet(p.first), p.first));
  //store components
  multimap<int, int>::iterator it = subcomponents.begin();
  int j = it->first;
  for (; it != subcomponents.end(); it++) {
    if (it->first != j) {
      j = it->first;
      //check if component has at leat a customer
      for (int customer : matheus_model_3.customers)
        if (component.count(customer)) {
          components.push_back(component);
          break;
        }
      component.clear();
    } 
    component.insert(it->second);
  }
  //check if component has at leat a customer
  for (int customer : matheus_model_3.customers)
    if (component.count(customer)) {
      components.push_back(component);
      break;
    }
  //end of multimap 
  //inequallitites
  for (const set<int>& S : components) 
    //\sum_{v_i \in V'\S} \sum_{v_j \in S} x_{ij} \geqslant N_ROUTES_LB(S) 
    if (!S.count(matheus_model_3.instance.depot.id)) {
      //get customers from component S
      vector<const Vertex *> vertices;
      vertices.push_back(&matheus_model_3.instance.depot);
      for (int customer : matheus_model_3.customers)
        if (S.count(customer)) 
          vertices.push_back(matheus_model_3.all[customer]);
      if (vertices.size() == 1)
        continue;
      //get n routes lbs
      const auto& closestsTimes = calculateClosestsGVRPCustomers(matheus_model_3.gvrpReducedGraphTimes, vertices);
      //get mst
      int improvedMSTNRoutesLB = int(ceil(calculateGvrpLBByImprovedMSTTime(vertices, closestsTimes, matheus_model_3.gvrpReducedGraphTimes)/matheus_model_3.instance.timeLimit));
      //bin packing
      int bppNRoutesLB = calculateGVRP_BPP_NRoutesLB(matheus_model_3.instance, vertices, closestsTimes, matheus_model_3.BPPTimeLimit);
      int maxNRoutes = max(improvedMSTNRoutesLB, bppNRoutesLB);
      if (improvedMSTNRoutesLB == maxNRoutes) 
        ++matheus_model_3.nImprovedMSTNRoutesLB;
      if (bppNRoutesLB == maxNRoutes) 
        ++matheus_model_3.nBPPNRoutesLB;
      //lhs
      for (const pair<int, const Vertex *>& p : matheus_model_3.all) 
        if (!S.count(p.first))
          for (int j : S) 
            lhs += matheus_model_3.x[p.first][j];
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
  for (const pair<int, const Vertex *>& p : matheus_model_3.all){
    int i = p.first;
    x_vals[i].end();
  }
  x_vals.end();
}
