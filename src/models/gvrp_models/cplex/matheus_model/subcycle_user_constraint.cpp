#include "models/vertex.hpp"
#include "models/dsu.hpp"
#include "models/cplex/mip_depth.hpp"
#include "models/gvrp_models/cplex/gvrp_lp_kk_heuristic.hpp"
#include "models/gvrp_models/cplex/matheus_model/matheus_model.hpp"
#include "models/gvrp_models/cplex/matheus_model/lazy_constraint.hpp"
#include "models/gvrp_models/cplex/matheus_model/subcycle_user_constraint.hpp"
#include "models/bpp_models/bpp_instance.hpp"
#include "models/bpp_models/cplex/bpp_model.hpp"

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
using namespace models;
using namespace models::cplex;
using namespace models::bpp_models;
using namespace models::bpp_models::cplex;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::matheus_model;
using namespace lemon;
using namespace lemon::concepts;

Subcycle_user_constraint::Subcycle_user_constraint (Matheus_model& matheus_model_) : User_constraint (matheus_model_) {}

IloCplex::CallbackI* Subcycle_user_constraint::duplicateCallback() const {
  return new(getEnv()) Subcycle_user_constraint (*this);
}

void Subcycle_user_constraint::main() {
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
  unsigned int bpp_time_limit = 10000000;
  int bppNRoutesLB, 
      mstNRoutesLB, 
      kkGreedyNRoutesLB,
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
  //creating nodes
  for (size_t i = 0; i < sc0; ++i) 
    nodes[i] = graph.addNode();
  //get components
  for (int i = 1; i < sc0; ++i) {
    if (visited[i])
      continue;
    ListGraph::EdgeMap<double> weight(graph); 
    //bfs
    component.insert(i);
    q.push(i);
    while (!q.empty()) {
      int curr = q.front();
      visited[curr] = true;
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
          if (!visited[j]) {
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
      for (size_t i = 0; i < sc0; ++i) 
        if (!S.count(i)) 
          for (int j : S) {
            lhs += matheus_model.x[i][j];
            for (size_t f = 0; f < sf0; ++f)
              lhs += matheus_model.y[i][f][j];
          }
      //rhs
        //get mst
      DSU mstDsu (sc0);
      double mstCost = 0;
      auto comp = [](const tuple<int, int, double> & edge1, const tuple<int, int, double> & edge2)
      {
        return get<2>(edge1) > get<2>(edge2);
      };
      priority_queue <tuple<int, int, double>, vector<tuple<int, int, double>>, decltype(comp)> pq (comp); 
      for (int i : S) {
        for (int j : S)
          if (i != j) 
            pq.push (make_tuple(i, j, matheus_model.instance.time(matheus_model.c0[i]->id, matheus_model.c0[j]->id)));
        pq.push (make_tuple(i, 0, matheus_model.instance.time(matheus_model.c0[i]->id, matheus_model.c0[0]->id)));
        pq.push (make_tuple(0, i, matheus_model.instance.time(matheus_model.c0[0]->id, matheus_model.c0[i]->id)));
      }
      while (!pq.empty()) {
        tuple<int, int, double> edge = pq.top();
        pq.pop();
        if (dsu.findSet(get<0>(edge)) != dsu.findSet(get<1>(edge))) {
          dsu.join(get<0>(edge), get<1>(edge));
          mstCost += get<2> (edge);
        }
      }
      mstNRoutesLB = ceil(mstCost/matheus_model.instance.timeLimit);
        //calculate ditances
      vector<double> closestS0Time (sc0);
      vector<double> secondClosestS0Time (sc0);
      for (int i : S) {
        secondClosestS0Time[i] = closestS0Time[i] = matheus_model.instance.time(matheus_model.c0[i]->id, matheus_model.c0[0]->id);
        for (int j : S) 
          if (i != j) {
            double time = matheus_model.instance.time(matheus_model.c0[i]->id, matheus_model.c0[j]->id);
            if (time < closestS0Time[i]) {
              secondClosestS0Time[i] = closestS0Time[i];
              closestS0Time[i] = time;
            } else if (time < secondClosestS0Time[i]) 
              secondClosestS0Time[i] = time;
          }
      }
      for (size_t i = 0; i < sc0; ++i)
        if (S.count(i))
          cout<<i<<" "<<closestS0Time[i]<<secondClosestS0Time[i]<<endl;
        //bin packing
      vector<double> items (S.size() + 1); 
      items[0] = matheus_model.c0[0]->serviceTime + (closestS0Time[0] + secondClosestS0Time[0])/2;
      for (int i : S) {
        items[j] = matheus_model.c0[i]->serviceTime + (closestS0Time[i] + secondClosestS0Time[i])/2;
        ++j;
      }
      BPP_instance bpp_instance (items, matheus_model.instance.timeLimit);
      BPP_model bpp_model (bpp_instance, bpp_time_limit);
      bppNRoutesLB = bpp_model.run().first.size();
        //greedy
      kkGreedyNRoutesLB = matheus_model.kk_greedy_nRoutes_lb(S);
      if (mstNRoutesLB >= bppNRoutesLB && mstNRoutesLB >= kkGreedyNRoutesLB) {
        cout<<"MST with "<<mstNRoutesLB;
        ++matheus_model.nMSTNRoutesLB;
        maxNRoutes = mstNRoutesLB;
      } else if (bppNRoutesLB >= mstNRoutesLB && bppNRoutesLB >= kkGreedyNRoutesLB) {
        cout<<"BPP with "<<bppNRoutesLB;
        ++matheus_model.nBPPNRoutesLB;
        maxNRoutes = bppNRoutesLB;
      } else  {
        cout<<"KK Greedy with "<<bppNRoutesLB;
        ++matheus_model.nKKGreedyNRoutesLB;
        maxNRoutes = kkGreedyNRoutesLB;
      }
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
  for (size_t i = 0; i < sc0; ++i) {
    x_vals[i].end();
    for (size_t f = 0; f < sf0; ++f) 
      y_vals[i][f].end();
    y_vals[i].end();
  }
  x_vals.end();
  y_vals.end();
}
