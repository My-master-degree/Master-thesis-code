#include "models/vertex.hpp"
#include "models/dsu.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/kk_model/kk_model.hpp"
#include "models/gvrp_models/cplex/kk_model/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/kk_model/subcycle_user_constraint.hpp"

#include <set>
#include <queue>
#include <list>
#include <ilcplex/ilocplex.h>
#include <lemon/gomory_hu.h>
#include <lemon/concepts/graph.h>
#include <lemon/list_graph.h>
#include <iostream>

using namespace std;
using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models::cplex::kk_model;
using namespace lemon;
using namespace lemon::concepts;

Subcycle_user_constraint::Subcycle_user_constraint (KK_model& kk_model_) : User_constraint (kk_model_) {}

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
  const size_t sc0 = kk_model.c0.size(),
        sf0 = kk_model.f0.size();
  DSU dsu (sc0);
  IloEnv env = getEnv();
  IloExpr lhs(env);
  vector<ListGraph::Node> nodes (sc0);
  ListGraph graph;
  multimap<int, int> subcomponents;
  set<int> component;
  set<int> visited;
  list<set<int>> components;
  queue<int> q;
  //get values
  Matrix2DVal x_vals (env, sc0);
  Matrix3DVal y_vals (env, sc0);
  for (size_t i = 0; i < sc0; ++i) {
    x_vals[i] = IloNumArray (env, sc0, 0, 1, IloNumVar::Float);
    y_vals[i] = Matrix2DVal (env, sc0);
    getValues(x_vals[i], kk_model.x[i]);
    for (size_t f = 0; f < sf0; ++f) {
      y_vals[i][f] = IloNumArray (env, sc0, 0, 1, IloNumVar::Float);
      getValues(y_vals[i][f], kk_model.y[i][f]);
    }
  }
  //creating nodes
  for (size_t i = 0; i < sc0; ++i) 
    nodes[i] = graph.addNode();
  //get components
  for (int i = 1; i < sc0; ++i) {
    if (visited.count(i))
      continue;
    ListGraph::EdgeMap<double> weight(graph); 
    //bfs
    component.insert(i);
    q.push(i);
    while (!q.empty()) {
      int curr = q.front();
      visited.insert(curr);
      q.pop();
      for (int j = 0; j < sc0; ++j) {
        double currToJFlow = 0.0, 
               jToCurrFlow = 0.0;
        bool flowExists = false;
        if (x_vals[curr][j] > EPS) {
          currToJFlow = x_vals[curr][j];
          flowExists = true;
          x_vals[curr][j] = 0;
        }
        if (x_vals[j][curr] > EPS) {
          jToCurrFlow = x_vals[j][curr];
          flowExists = true;
          x_vals[j][curr] = 0;
        }
        for (size_t f = 0; f < sf0; ++f) {
          if (y_vals[curr][f][j] > EPS) {
            currToJFlow += y_vals[curr][f][j];
            flowExists = true;
            y_vals[curr][f][j] = 0;
          }
          if (y_vals[j][f][curr] > EPS) {
            jToCurrFlow += y_vals[j][f][curr];
            flowExists = true;
            y_vals[j][f][curr] = 0;
          }
        }
        if (flowExists) {
          if (!visited.count(j)) {
            component.insert(j);
            q.push(j);
          }
          nodes[curr];
          nodes[j];
          auto e = graph.addEdge(nodes[curr], nodes[j]);
          weight[e] = currToJFlow;
          e = graph.addEdge(nodes[j], nodes[curr]);
          weight[e] = jToCurrFlow;
        }
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
        else
          cout<<"ooooopssss "<<i<<" and "<<j<<endl;
    component.clear();
  }
  //get subcomponents
  for (int i = 0; i < sc0; ++i) 
    subcomponents.insert(make_pair(dsu.findSet(i), i));
  //store components
  multimap<int, int>::iterator it = subcomponents.begin();
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
  for (const set<int>& S : components) 
    if (!S.count(0)) {
      //\sum_{v_i \in V'\S} \sum_{v_j \in S} x_{ij} + \sum_{v_f \in F_0} y_{ifj} \geqslant 1 
      //lhs
      for (int i = 0; i < sc0; ++i) 
        if (!S.count(i))
          for (int j : S) {
            lhs += kk_model.x[i][j];
            for (size_t f = 0; f < sf0; ++f)
              lhs += kk_model.y[i][f][j];
          }
      //rhs
      lhs -= 1;
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
  for (size_t i = 0; i < sc0; ++i) {
    x_vals[i].end();
    for (size_t f = 0; f < sf0; ++f) 
      y_vals[i][f].end();
    y_vals[i].end();
  }
  x_vals.end();
  y_vals.end();
}