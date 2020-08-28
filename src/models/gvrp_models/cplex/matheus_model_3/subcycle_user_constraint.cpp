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
  const int sall = matheus_model_3.all.size();
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
  ListGraph::NodeMap<int> nodeId(graph);
  for (const pair<int, const Vertex *>& p : matheus_model_3.all) 
    nodeId[nodes[p.first]] = p.first;
  ListGraph::EdgeMap<double> weight(graph); 
  for (const pair<int, const Vertex *>& p : matheus_model_3.all) 
    for (const pair<int, const Vertex *>& p1 : matheus_model_3.all) 
      if (x_vals[p.first][p1.first] > EPS)
        weight[graph.addEdge(nodes[p.first], nodes[p1.first])] = x_vals[p.first][p1.first];
  //gh
  GomoryHu<ListGraph, ListGraph::EdgeMap<double> > gh (graph, weight);
  gh.run();
  //get subcycles
  for (const pair<int, const Vertex *>& p : matheus_model_3.all) {
    int i = p.first;
    if (gh.predNode(nodes[i]) != INVALID && gh.predValue(nodes[i]) >= 2.0 - EPS) 
      dsu.join(i, nodeId[gh.predNode(nodes[i])]);
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
  for (set<int>& S : components) 
    //\sum_{v_i \in V'\S} \sum_{v_j \in S} x_{ij} \geqslant N_ROUTES_LB(S) 
    if (!S.count(matheus_model_3.instance.depot.id)) {
      //get customers from component S
      for (int node : S)
        cout<<node<<", ";
      cout<<endl;
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
      try {
        //lhs
        //in edges
        for (const pair<int, const Vertex *>& p : matheus_model_3.all) 
          if (!S.count(p.first))
            for (int j : S) 
              lhs += matheus_model_3.x[p.first][j];
        lhs -= maxNRoutes;
        add(lhs >= 0).end();
        lhs.end();
        lhs = IloExpr(env);
        //out edges
        for (const pair<int, const Vertex *>& p : matheus_model_3.all) 
          if (!S.count(p.first))
            for (int j : S) 
              lhs += matheus_model_3.x[j][p.first];
        lhs -= maxNRoutes;
        add(lhs >= 0).end();
        lhs.end();
        lhs = IloExpr(env);
        //bfs to get all afss out and connected to this component
        queue<int> q; 
        for (int node : S)
          q.push(node);
        while (!q.empty()) {
          int curr = q.front();
          q.pop();
          for (const pair<int, const Vertex *> p : matheus_model_3.dummies) {
            int afs = p.first;
            double weight = 0.0;
            if (x_vals[curr][afs] > EPS)
              weight += x_vals[curr][afs];
            if (x_vals[afs][curr] > EPS)
              weight += x_vals[afs][curr];
            if (!S.count(afs) && weight > EPS) {
//              cout<<"from "<<curr<<" to "<<afs<<endl;
              q.push(afs);
              S.insert(afs);
              //in edges
              //getting lhs
              for (const pair<int, const Vertex *>& p1 : matheus_model_3.all) {
                int a = p1.first;
                if (!S.count(a))
                  for (int b : S) 
                    lhs += matheus_model_3.x[a][b];
              }
              //getting rhs
              lhs -= maxNRoutes;
              add(lhs >= 0.0).end();
              //out edges
              lhs.end();
              lhs = IloExpr(env);
              //getting lhs
              for (const pair<int, const Vertex *>& p1 : matheus_model_3.all) {
                int a = p1.first;
                if (!S.count(a))
                  for (int b : S) 
                    lhs += matheus_model_3.x[b][a];
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
