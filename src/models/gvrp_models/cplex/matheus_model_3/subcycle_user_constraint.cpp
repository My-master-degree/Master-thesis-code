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
  DSU dsu (sall);
  IloEnv env = getEnv();
  IloExpr lhs(env);
  vector<ListGraph::Node> nodes (sall);
  ListGraph graph;
  multimap<int, int> subcomponents;
  unordered_set<int> component;
  list<unordered_set<int>> components;
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
  vector<vector<double>> costs (matheus_model_3.all.size(), vector<double> (matheus_model_3.all.size(), 0.0));
  for (const pair<int, const Vertex *>& p : matheus_model_3.all) 
    for (const pair<int, const Vertex *>& p1 : matheus_model_3.all) 
      if (x_vals[p.first][p1.first] > EPS) {
        weight[graph.addEdge(nodes[p.first], nodes[p1.first])] = x_vals[p.first][p1.first];
        costs[min(p.first, p1.first)][max(p.first, p1.first)] += x_vals[p.first][p1.first];
      }
  for (const pair<int, const Vertex *>& p : matheus_model_3.all) 
    for (const pair<int, const Vertex *>& p1 : matheus_model_3.all) 
      if (costs[p.first][p1.first] > EPS) {
        cout<<p.first<<", "<<p1.first<<": "<<costs[p.first][p1.first]<<endl;
      }
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
  list<unordered_set<int>> previousSubcycles = matheus_model_3.previousSubsets;
  matheus_model_3.previousSubsets.clear();
  //end of multimap 
  //inequallitites
  for (unordered_set<int>& S : components) 
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
      matheus_model_3.previousSubsets.push_back(S);
      for (int a : S)
        cout<<a<<", ";
      cout<<endl;
      //get n routes lbs
      const auto& closestsTimes = calculateClosestsGVRPCustomers(matheus_model_3.gvrpReducedGraphTimes, vertices);
      //get mst
      int improvedMSTNRoutesLB = int(ceil(calculateGvrpLBByImprovedMSTTime(vertices, closestsTimes, matheus_model_3.gvrpReducedGraphTimes)/matheus_model_3.instance.timeLimit));
      //bin packing
      int bppNRoutesLB = calculateGVRP_BPP_NRoutesLB(matheus_model_3.instance, vertices, closestsTimes, matheus_model_3.BPPTimeLimit);
      maxNRoutes = max(improvedMSTNRoutesLB, bppNRoutesLB);
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
      } catch(IloException& e) {
        cerr << "Exception while adding user constraint" << e.getMessage() << "\n";
        throw;
      }
      //bfs to get all afss out and connected to this component
      unordered_set<int> afss;
      queue<int> q; 
      for (int node : S)
        q.push(node);
      while (!q.empty()) {
        size_t queueSize = q.empty();
        for (size_t remainingNodes = 0; remainingNodes < queueSize; --remainingNodes) {
          int curr = q.front();
          q.pop();
          for (const pair<int, const Vertex *> p : matheus_model_3.dummies) {
            int afs = p.first;
            if (!afss.count(afs) && !S.count(afs) && x_vals[curr][afs] + x_vals[afs][curr] > EPS) {
              q.push(afs);
              afss.insert(afs);
            }
          }
        }
        break;
      }
      for(int a : afss) {
          cout<<a<<", ";
      }
      cout<<endl;
      //lhs
      //in edges
      for (const pair<int, const Vertex *>& p : matheus_model_3.all) 
        if (!S.count(p.first) && !afss.count(p.first)) {
          for (int j : S) 
            lhs += matheus_model_3.x[p.first][j];
          for (int j : afss) 
            lhs += matheus_model_3.x[p.first][j];
        }
      lhs -= maxNRoutes;
      add(lhs >= 0).end();
      lhs.end();
      lhs = IloExpr(env);
      bool repeated = true;
      for (const unordered_set<int> set : previousSubcycles) {
        bool found = false;
        for (unordered_set<int>& S : components) 
          if (set == S) {
            found = true;
            break;
          }
        if (!found) {
          repeated = false;
          break;
        }
      }
//      if (repeated) {
      if (true) {
        //add all possible ineq
        vector<int> afssVector (afss.begin(), afss.end());
        const size_t vectorSize = afssVector.size();
        //size of the power set of a set with set size n is (2**n -1)
        size_t pow_set_size = 1<<vectorSize; 
        //Run from counter 000..0 to 111..1
//        cout<<pow_set_size<<endl;
        for(int counter = 0; counter < pow_set_size; counter++) { 
          unordered_set<int> set;
          for(int j = 0; j < vectorSize; j++)  
            //Check if jth bit in the counter is set
            if(counter & (1 << j)) 
              set.insert(afssVector[j]);
          double cut = 0.0;
          for (const pair<int, const Vertex *>& p : matheus_model_3.all) {
            int a = p.first;
            if (!S.count(a) && !set.count(a)) {
              for (int b : S) {
                if (x_vals[a][b] > EPS)
                  cut += x_vals[a][b];
                if (x_vals[b][a] > EPS)
                  cut += x_vals[b][a];
              }
              for (int b : set) {
                if (x_vals[a][b] > EPS)
                  cut += x_vals[a][b];
                if (x_vals[b][a] > EPS)
                  cut += x_vals[b][a];
              }
            }
          }
          /*
          if (cut < 2 - EPS) {
            cout<<"Set:";
            for (int b : S) 
              cout<<b<<",";
            for (int b : set) 
              cout<<b<<",";
            cout<<endl;
          }
          */
          //add ineq
          try {
            //in edges
            //getting lhs
            for (const pair<int, const Vertex *>& p : matheus_model_3.all) {
              int a = p.first;
              if (!S.count(a) && !set.count(a)) {
                for (int b : S) 
                  lhs += matheus_model_3.x[a][b];
                for (int b : set) 
                  lhs += matheus_model_3.x[a][b];
              }
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
        /*
        //out edges
        for (const pair<int, const Vertex *>& p : matheus_model_3.all) 
        if (!S.count(p.first))
        for (int j : S) 
        lhs += matheus_model_3.x[j][p.first];
        lhs -= maxNRoutes;
        add(lhs >= 0).end();
        lhs.end();
        lhs = IloExpr(env);
        */
        lhs.end();
        lhs = IloExpr(env);
      }
    }
  //clean
  for (const pair<int, const Vertex *>& p : matheus_model_3.all){
    int i = p.first;
    x_vals[i].end();
  }
  x_vals.end();
}
