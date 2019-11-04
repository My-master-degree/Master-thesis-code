#include <map>
#include <set>
#include <queue>
#include <ilcplex/ilocplex.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <sstream>
#include <list>
#include <vector>
#include "models/vertex.hpp"
#include "models/gvrp_instance.hpp"
#include "utils/cplex/quadratic_model_subcycle_constraint_callback.hpp"
#include "SampleConfig.h"
ILOSTLBEGIN

using namespace std;

int main (int argc, char **argv)
{ 
  int id = 0,
      customers_size,
      afss_size,
      total_size,
      NUM_MAX_VISITS_AFS;  
  double vehicleFuelCapacity;
  string buff, type, x, y;
  string::size_type sz = 0;
  string line;
  ifstream inFile;
  list<vertex::Vertex> afss, customers;
  stringstream ss;
  vertex::Vertex depot;
  //read file
  inFile.open(PROJECT_PATH + string("S4_S2_4i8s.txt"));
  if (!inFile) {
    cerr << "Unable to open file";
    exit(1);   // call system to stop
  }  
  //ignore header
  getline(inFile, line);
  //get depot
  getline(inFile, line);
  ss.str(line);
  //ignore id
  ss>>buff;
  ss>>type;
  ss>>x;
  ss>>y;
  depot = vertex::Vertex(id++, stod(x, NULL), stod(y, NULL));
  //get afs's
  getline(inFile, line);
  ss.clear();
  ss.str(line);
  //ignore id
  ss>>buff;
  //get type
  ss>>type;
  ss>>x;
  ss>>y;
  while (type == "f"){         
    afss.push_back(vertex::Vertex(id++, stod(x, NULL), stod(y, NULL)));
    getline(inFile, line);
    ss.clear();
    ss.str(line);
    //ignore id
    ss>>buff;
    //get type
    ss>>type;
    ss>>x;
    ss>>y;
  }
  //get customers
  while (type == "c"){         
    customers.push_back(vertex::Vertex(id++, stod(x, NULL), stod(y, NULL)));
    getline(inFile, line);
    if (line.empty())
      break;
    ss.clear();
    ss.str(line);
    //ignore id
    ss>>buff;
    //get type
    ss>>type;
    ss>>x;
    ss>>y;
  }
  //get beta
  getline(inFile, line);
  ss.clear();
  ss.str(line);  
  while (!ss.eof())
    ss>>buff;  
  vehicleFuelCapacity = stod(buff.substr(1), NULL);
  inFile.close();
  //save data
  customers_size = customers.size();
  afss_size = afss.size();
  total_size = customers_size + afss_size + 1;
  NUM_MAX_VISITS_AFS = customers_size * 2;
  //calculate distances
  vector<vertex::Vertex> vertexes (total_size);
  int j =0; 
  vertexes[j++] = depot;
  for (list<vertex::Vertex>::iterator i = customers.begin(); i != customers.end(); i++)
    vertexes[j++] = *i;
  for (list<vertex::Vertex>::iterator i = afss.begin(); i != afss.end(); i++)
    vertexes[j++] = *i;
  vector<vector<double> > distances(total_size);
  for (int i = 0; i < total_size; i++){
    distances[i] = vector<double> (total_size);
    for (int j = 0; j < total_size; j++)
      distances[i][j] = hypot(vertexes[i].x - vertexes[j].x, vertexes[i].y - vertexes[j].y);
  }
  //print data
  cout<<"Depot: "<<endl;
  cout<<depot.id<<" "<<depot.x<<" "<<depot.y<<endl;
  cout<<"AFSs: "<<endl;
  for (list<vertex::Vertex>::iterator i = afss.begin(); i != afss.end(); i++)
    cout<<i->id<<" "<<i->x<<" "<<i->y<<endl;
  cout<<"Customers: "<<endl;
  for (list<vertex::Vertex>::iterator i = customers.begin(); i != customers.end(); i++)
    cout<<i->id<<" "<<i->x<<" "<<i->y<<endl;

  //model
  IloEnv env;
  try {
    IloModel model(env);
    IloArray<IloNumVarArray> x_vars (env, total_size),
                            y_vars(env, customers_size + 1),
                            w_vars(env, customers_size);
    IloNumVarArray e_vars(env, total_size, 0, vehicleFuelCapacity, IloNumVar::Float);
    IloArray<IloNumArray> x_vals (env, total_size),
                            y_vals(env, customers_size + 1),
                            w_vals(env, customers_size);
    IloNumArray e_vals(env, total_size, 0, vehicleFuelCapacity, IloNumVar::Float);
    //x var
    for (int i = 0; i < total_size; i++){
      x_vars[i] = IloNumVarArray(env, total_size, 0, 1, IloNumVar::Int);
      x_vals[i] = IloNumArray(env, total_size, 0, 1, IloNumVar::Int);
    }
    //y and w var
    for (int i = 0; i < customers_size; i++){
      y_vars[i] = IloNumVarArray(env, customers_size + 1, 0, 1, IloNumVar::Int);
      y_vals[i] = IloNumArray(env, total_size, 0, 1, IloNumVar::Int);
      w_vars[i] = IloNumVarArray(env, afss_size, 0, NUM_MAX_VISITS_AFS, IloNumVar::Int);
      w_vals[i] = IloNumArray(env, afss_size, 0, NUM_MAX_VISITS_AFS, IloNumVar::Int);
    }
    //objective function
    IloExpr fo (env);
    for (int i = 0; i < total_size; i++)
      for (int j = 0; j < total_size; j++)
        fo +=  distances[i][j] * x_vars[i][j];
    model.add(IloMinimize(env, fo));
    //constriaints
    IloExpr expr(env),
            expr1(env),
            expr2(env);
    //y[k][0] = 1, \forall k \in M
    for (int k = 0; k < customers_size; k++)
      model.add(y_vars[k][0] == 1);
    //\sum_{k \in M} y_{i}^{k} = 1, \forall v_i \in C
    for (int i = 1; i <= customers_size; i++){
      for (int k = 0; k < customers_size; k++)
        expr += y_vars[k][i];
      model.add(expr == 1);
      expr.end();
      expr = IloExpr(env);
    }
    //\sum_{j \in V} x_{ij} = \sum_{j \in V} x_{ji} = \sum_{k \in  M} y_{i}^k, \forall v_i \in C \cup \{v_0\}
    for (int i = 0; i <= customers_size; i++){
      for (int j = 0; j <= customers_size; j++){
        expr += x_vars[i][j];     
        expr1 += x_vars[j][i];    
      }
      for (int k = 0; k < customers_size; k++)
        expr2 += y_vars[k][i];
      model.add(expr == expr1);
      model.add(expr1 == expr2);
      expr.end();
      expr1.end();
      expr2.end();
      expr = IloExpr(env);
      expr1 = IloExpr(env);
      expr2 = IloExpr(env);
    }
    //w_f^k = \sum_{j \in V} x_{fj} = \sum_{j \in V} x_{jf}, \forall v_j \in F, k \in M
    for (int f = 0; f < afss_size; f++){
      for (int k = 0; k < customers_size; k++){
        for (int j = 0; j < total_size; j++){
          expr += x_vars[customers_size + 1 + f][j];
          expr1 += x_vars[j][customers_size + 1 + f];
        }
        model.add(w_vars[k][f] == expr);
        model.add(expr == expr1);
        expr.end();
        expr1.end();
        expr = IloExpr(env);
        expr1 = IloExpr(env);
      }
    }
    //y_i^k \geq x_{ij} + y_j^k - 1, \forall v_i, v_j \in C, k \in M
    //y_j^k \geq x_{ij} + y_i^k - 1, \forall v_i, v_j \in C, k \in M
    for (int i = 1; i <= customers_size; i++)
      for (int j = 1; j <= customers_size; j++)
        for (int k = 0; k < customers_size; k++){
          expr = x_vars[i][j] + y_vars[k][j] - 1;
          model.add(y_vars[k][i] >= expr);
          expr.end();
          expr = IloExpr(env);
          expr = x_vars[i][j] + y_vars[k][i] - 1;
          model.add(y_vars[k][j] >= expr);
          expr.end();
          expr = IloExpr(env);
        }
    //e_0 = \beta
    model.add(e_vars[0] == vehicleFuelCapacity);
    //e_f = \beta, \forall v_f \in F
    for (int f = 0; f < afss_size; f++)
      model.add(e_vars[customers_size + 1 + f] == vehicleFuelCapacity);
    //e_j \leq e_i - c_{ij} x_{ij} + \beta (1 - x_{ij}), \forall v_j \in C, v_i \in V
    for (int j = 1; j <= customers_size; j++)
      for (int i = 0; i < total_size; i++){
        expr = e_vars[i] - distances[i][j] * x_vars[i][j] + vehicleFuelCapacity * (1 - x_vars[i][j]);
        model.add(e_vars[j] <= expr);
        expr.end();
        expr = IloExpr (env);
      }
    //e_i \geq c_{ij} x_{ij}, \forall v_i, v_j \in V
    for (int i = 0; i < total_size; i++)
      for (int j = 0; j < total_size; j++){
        expr = distances[i][j] * x_vars[i][j];
        model.add(e_vars[i] >= expr);
        expr.end();
        expr = IloExpr(env);
      }
    //e_i \geq x_{i0} c_{i0}
    for (int i = 0; i < total_size; i++){
      expr = x_vars[i][0] * distances[i][0];
      model.add(e_vars[i] >= expr);
      expr.end();
      expr = IloExpr(env);
    }
    //run model
    IloCplex cplex(model);
    if ( !cplex.solve() ) {
      env.error() << "Failed to optimize LP." << endl;
      throw(-1);
    }

//    IloNumArray vals(env);
//    env.out() << "Solution status = " << cplex.getStatus() << endl;
//    env.out() << "Solution value = " << cplex.getObjValue() << endl;
//    for (int i = 0; i < total_size; i++)
//      cplex.getValues(x_vals[i], x_vars[i]);
//    for (int i = 0; i < customers_size; i++){
//      cplex.getValues(y_vals[i], y_vars[i]);
//      cplex.getValues(w_vals[i], w_vars[i]);
//    }
//    cplex.getValues(e_vals, e_vars);
    
    //separation algorithm
    for (int k = 0; k < customers_size; k++){
      list<int> vertexes;
      //get customers
      for (int i = 0; i <= customers_size; i++)
        if (y_vals[k][i] > 0)
          vertexes.push_back(i);
      //get afss
      for (int f = 0; f < afss_size; f++)
        if (w_vals[k][f] > 0)
          vertexes.push_back(f + customers_size + 1);
      //get edges
      map<int, list<int>> adj;
      for (list<int>::iterator it1 = vertexes.begin(); it1 != vertexes.end(); it1++)
        for (list<int>::iterator it2 = vertexes.begin(); it2 != vertexes.end(); it2++)
          if (x_vals[*it1][*it2] > 0)
            adj[*it1].push_back(*it2);
      //get connected components
      map<int, bool> visited;
      for (map<int, list<int> >::iterator it = adj.begin(); it != adj.end(); it++){
        if (!visited[it->first]){
          set<int> connectedCompVertexes;
          queue<int> q;
          q.push(it->first);
          visited[it->first] = true;
          while (!q.empty()){
            int vertex = q.front();
            connectedCompVertexes.insert(vertex);
            q.pop();
            list<int> neighbors = adj[vertex];
            for (list<int>::iterator it1 = neighbors.begin(); it1 != neighbors.end(); it1++)
              if (!visited[*it1]){
                q.push(*it1);
                visited[*it1] = true;
              }
          }
          //add constraints
          if (connectedCompVertexes.find(0) == connectedCompVertexes.end()){            
            int i_start = -1;
            //\sum_{v_i \in V\S} \sum_{v_j \in S} x_{ij}
            for (int i = 0; i < total_size; i++)
              if (connectedCompVertexes.find(i) == connectedCompVertexes.end()) 
                for (set<int>::iterator it1 = connectedCompVertexes.begin(); it1 != connectedCompVertexes.end(); it1++)
                    expr += x_vars[i][*it1];                      
            //\sum_{v_p \in V} x_{pi*}
            for (set<int>::iterator it1 = connectedCompVertexes.begin(); it1 != connectedCompVertexes.end(); it1++)
              if (1 <= *it1 && *it1 <= customers_size)
                i_start = *it1;
            if (i_start == -1)
              break;           
            for (int p = 0; p < total_size; p++)
              expr -= x_vars[p][i_start];
            //\sum_{v_i \in V\S} \sum_{v_j \in S} x_{ij} - \sum_{v_p \in V} x_{pi*}>= 0
            IloExprArray lhs (env, 1);
            IloNumArray rhs(env, 1);
            lhs[0] = expr;
            rhs[0] = 0; 
            cplex.use((IloCplex::Callback(new(env) quadratic_model_subcycle_constraint_callback::Quadratic_model_subcycle_constraint_callback(env, lhs, rhs, cplex.getParam(IloCplex::EpRHS)))));
            expr.end();
            expr = IloExpr(env);
          }
        }
      }
    }
//    env.out() << "Values = " << x_vals << endl;
  }catch (IloException& e) {
    cerr << "Concert exception caught: " << e << endl;
  }
  catch (...) {
    cerr << "Unknown exception caught" << endl;
  }
  env.end();

  return 0;
}
