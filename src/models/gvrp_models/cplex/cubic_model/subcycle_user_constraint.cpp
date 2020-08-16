#include "utils/util.hpp"
#include "models/dsu.hpp"
#include "models/vertex.hpp"
#include "models/gvrp_models/cplex/cubic_model/cubic_model.hpp"
#include "models/gvrp_models/cplex/cubic_model/user_constraint.hpp"
#include "models/gvrp_models/cplex/cubic_model/subcycle_user_constraint.hpp"
#include "models/cplex/mip_depth.hpp"

#include <set>
#include <queue>
#include <list>
#include <ilcplex/ilocplex.h>
#include <lemon/gomory_hu.h>
#include <lemon/concepts/graph.h>
#include <lemon/list_graph.h>

using namespace std;
using namespace utils;
using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models::cplex::cubic_model;
using namespace lemon;
using namespace lemon::concepts;

Subcycle_user_constraint::Subcycle_user_constraint (Cubic_model& cubic_model_) : User_constraint (cubic_model_) {}

IloCplex::CallbackI* Subcycle_user_constraint::duplicateCallback() const {
  return new(getEnv()) Subcycle_user_constraint (*this);
}

void Subcycle_user_constraint::main() {
  Depth const* const d = (Depth *)getNodeData();
  IloInt depth = d ? d->depth : 0;
  if (depth > cubic_model.levelSubcycleCallback) {
    abortCutLoop();
    return;
  }
  const int sall = cubic_model.instance.distances.size();
  DSU dsu (sall);
  ListGraph graph;
  ListGraph::EdgeMap<double>  weight(graph); 
  IloEnv env = getEnv();
  IloExpr lhs(env);
  unordered_set<int> component;
  list<unordered_set<int>> components;
  unordered_multimap<int, int> subcomponents (sall);
  vector<ListGraph::Node> nodes (sall);
  int depot = cubic_model.instance.depot.id;
  int k;
  //get values
  Matrix3DVal x_vals (env, cubic_model.instance.maxRoutes);
  for (k = 0; k < cubic_model.instance.maxRoutes; k++) {
    x_vals[k] = IloArray<IloNumArray> (env, sall);
    for (const pair<int, const Vertex *>& p : cubic_model.all) {
      int i = p.first;
      x_vals[k][i] = IloNumArray (env, sall, 0, 1, IloNumVar::Float);
      getValues(x_vals[k][i], cubic_model.x[k][i]);
    }
  }
  //build graph
  //creating nodes
  for (const pair<int, const Vertex *>& p : cubic_model.all) 
    nodes[p.first] = graph.addNode();
  ListGraph::NodeMap<int> nodeId(graph);
  for (const pair<int, const Vertex *>& p : cubic_model.all) 
    nodeId[nodes[p.first]] = p.first;
  //creating edges
  for (const pair<int, const Vertex *>& p : cubic_model.all) {
    int i = p.first;
    for (const pair<int, const Vertex *>& p1 : cubic_model.all) {
      int j = p1.first;
      double cost = 0.0;
      for (k = 0; k < cubic_model.instance.maxRoutes; k++) 
        if (x_vals[k][i][j] > EPS)
          cost += x_vals[k][i][j];
      weight[graph.addEdge(nodes[i], nodes[j])] = cost;
    }
  }
  for (auto it = cubic_model.all.begin(); it != cubic_model.all.end(); ++it) {
    int i = it->first;
    for (auto it1 = it; it1 != cubic_model.all.end(); ++it1) {
      int j = it1->first;
      double cost = 0.0;
      for (k = 0; k < cubic_model.instance.maxRoutes; k++) {
        if (x_vals[k][i][j] > EPS)
          cost += x_vals[k][i][j];
        if (x_vals[k][j][i] > EPS)
          cost += x_vals[k][j][i];
      }
//      if (cost > 0.0)
//        cout<<i<<", "<<j<<": "<<cost<<endl;
    }
  }
  //gomory hu
  GomoryHu<ListGraph, ListGraph::EdgeMap<double>> gh (graph, weight);
  gh.run();
  //get subcycles
  for (const pair<int, const Vertex *>& p : cubic_model.all) {
    int i = p.first;
    if (gh.predNode(nodes[i]) != INVALID && gh.predValue(nodes[i]) >= 2.0 - EPS) 
      dsu.join(i, nodeId[gh.predNode(nodes[i])]);
  }
  /*
  for (const pair<int, const Vertex *>& p : cubic_model.all) {
    int i = p.first;
    for (const pair<int, const Vertex *>& p1 : cubic_model.all) {
      int j = p1.first;
      if (gh.minCutValue(nodes[i], nodes[j]) >= 2.0 - EPS) 
        dsu.join(i, j);
    }
  }
  */
  //get subcycles
  //get subcomponents
  for (int i = 0; i < sall; ++i) 
    subcomponents.insert(make_pair(dsu.findSet(i), i));
  //store components
  unordered_multimap<int, int>::iterator it = subcomponents.begin();
  int j = it->first;
  for (; it != subcomponents.end(); ++it) {
    if (it->first != j) {
      j = it->first;
      components.push_back(component);
      component.clear();
    } 
    component.insert(it->second);
  }
  components.push_back(component);
  //end of multimap 
  //inequalitites
  for (unordered_set<int>& S : components) 
    if (!S.count(depot)) {
      //get customers from component S
      list<int> customersComponent;
      for (int customer : cubic_model.customers)
        if (S.count(customer))
          customersComponent.push_back(customer);
      if (customersComponent.size() == 0)
        continue;
      int i = 1;
      vector<const Vertex *> vertices (customersComponent.size() + 1);
      vertices[0] = &cubic_model.instance.depot;
      for (int customer : customersComponent) {
        vertices[i] = cubic_model.all[customer];
        ++i;
      }
      //get n routes lbs
      const auto& closestsTimes = calculateClosestsGVRPCustomers(cubic_model.gvrpReducedGraphTimes, vertices);
      //get mst
      int improvedMSTNRoutesLB = int(ceil(calculateGvrpLBByImprovedMSTTime(vertices, closestsTimes, cubic_model.gvrpReducedGraphTimes)/cubic_model.instance.timeLimit));
      //bin packing
      int bppNRoutesLB = calculateGVRP_BPP_NRoutesLB(cubic_model.instance, vertices, closestsTimes, cubic_model.BPPTimeLimit);
      int maxNRoutes = max(improvedMSTNRoutesLB, bppNRoutesLB);
      if (improvedMSTNRoutesLB == maxNRoutes) 
        ++cubic_model.nImprovedMSTNRoutesLB;
      if (bppNRoutesLB == maxNRoutes) 
        ++cubic_model.nBPPNRoutesLB;
      try {
        //in edges
        //getting lhs
        for (k = 0; k < cubic_model.instance.maxRoutes; ++k) {
          for (const pair<int, const Vertex *>& p2 : cubic_model.all) {
            int a = p2.first;
            if (!S.count(a))
              for (int b : S) 
                lhs += cubic_model.x[k][a][b];
          }
        }
        //getting rhs
        lhs -= maxNRoutes;
        add(lhs >= 0.0).end();
        //out edges
        lhs.end();
        lhs = IloExpr(env);
        //getting lhs
        for (k = 0; k < cubic_model.instance.maxRoutes; ++k) {
          for (const pair<int, const Vertex *>& p2 : cubic_model.all) {
            int a = p2.first;
            if (!S.count(a))
              for (int b : S) 
                lhs += cubic_model.x[k][b][a];
          }
        }
        //getting rhs
        lhs -= maxNRoutes;
        add(lhs >= 0.0).end();
        lhs.end();
        lhs = IloExpr(env);
        //bfs to get all afss out and connected to this component
        queue<int> q; 
        for (int node : S)
          q.push(node);
        while (!q.empty()) {
          int curr = q.front();
          q.pop();
          for (int afs : cubic_model.afss) {
            double weight = 0.0;
            for (k = 0; k < cubic_model.instance.maxRoutes; ++k) {
              if (x_vals[k][curr][afs] > EPS)
                weight += x_vals[k][curr][afs];
              if (x_vals[k][j][curr] > EPS)
                weight += x_vals[k][afs][curr];
            }
            if (!S.count(afs) && weight > EPS) {
//              cout<<"from "<<curr<<" to "<<afs<<endl;
              q.push(afs);
              S.insert(afs);
              //in edges
              //getting lhs
              for (k = 0; k < cubic_model.instance.maxRoutes; ++k) {
                for (const pair<int, const Vertex *>& p2 : cubic_model.all) {
                  int a = p2.first;
                  if (!S.count(a))
                    for (int b : S) 
                      lhs += cubic_model.x[k][a][b];
                }
              }
              //getting rhs
              lhs -= maxNRoutes;
              add(lhs >= 0.0).end();
              //out edges
              lhs.end();
              lhs = IloExpr(env);
              //getting lhs
              for (k = 0; k < cubic_model.instance.maxRoutes; ++k) {
                for (const pair<int, const Vertex *>& p2 : cubic_model.all) {
                  int a = p2.first;
                  if (!S.count(a))
                    for (int b : S) 
                      lhs += cubic_model.x[k][b][a];
                }
              }
              //getting rhs
              lhs -= maxNRoutes;
              add(lhs >= 0.0).end();
              lhs.end();
              lhs = IloExpr(env);
              for (int node : S) 
                cout<<node<<", "; 
              cout<<endl;
            }
          }
        }
//        cout<<"("<<maxNRoutes<<")"<<endl;
      } catch(IloException& e) {
        cerr << "Exception while adding lazy constraint" << e.getMessage() << "\n";
        throw;
      }
      lhs.end();
      lhs = IloExpr(env);
    }
  //end vars
  for (k = 0; k < cubic_model.instance.maxRoutes; ++k)
    for (const pair<int, const Vertex *>& p : cubic_model.all){
      int i = p.first;
      x_vals[k][i].end();
    }
  x_vals.end();
}
