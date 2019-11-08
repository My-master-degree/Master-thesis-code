#include "utils/cplex/quadratic_model.hpp"
#include "models/vertex.hpp"
#include "models/gvrp_instance.hpp"

#include <list>
#include <map>
#include <set>
#include <queue>
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

using namespace std;
using namespace models;
using namespace utils::cplex;

//Quadratic_model_subcycle_constraint_callback ::Quadratic_model_subcycle_constraint_callback(IloEnv env, IloExprArray xx1, IloNumArray xx2, IloNum xx3) : 
//  IloCplex::LazyConstraintCallbackI(env), lhs(xx1), rhs(xx2), eps(xx3) 
//  {
//  }

//IloCplex::CallbackI* Quadratic_model_subcycle_constraint_callback::duplicateCallback() const{
//  return (new(getEnv()) Quadratic_model_subcycle_constraint_callback(*this));
//} 
//void Quadratic_model_subcycle_constraint_callback::main(void){
//}

//separation algorithm
ILOLAZYCONSTRAINTCALLBACK5(Quadratic_subcycle_constraint, IloArray<IloNumVarArray> &, x, IloArray<IloNumVarArray> &, y, IloArray<IloNumVarArray> &, w, IloNumVarArray &, e, Gvrp_instance &, gvrp_instance) {
  int customers_size = gvrp_instance.customers.size(),
      afss_size = gvrp_instance.afss.size(),
      total_size = customers_size + afss_size + 1,
      NUM_MAX_VISITS_AFS = 1 + customers_size;
  IloEnv env = getEnv();
  IloExpr expr(env);
  IloArray<IloNumArray> x_vals (env, total_size),
    y_vals(env, customers_size + 1),
    w_vals(env, customers_size);
  IloNumArray e_vals(env, total_size, 0, gvrp_instance.vehicleFuelCapacity, IloNumVar::Float);
  for (int i = 0; i < total_size; i++){
    x_vals[i] = IloNumArray(env, total_size, 0, 1, IloNumVar::Int);
    getValues(x_vals[i], x[i]);
  }
  for (int i = 0; i < customers_size; i++){
    y_vals[i] = IloNumArray(env, total_size, 0, 1, IloNumVar::Int);
    getValues(y_vals[i], y[i]);
    w_vals[i] = IloNumArray(env, afss_size, 0, NUM_MAX_VISITS_AFS, IloNumVar::Int);
    getValues(w_vals[i], w[i]);
  }
  getValues(e_vals, e);
  //for each route
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
                expr += x[i][*it1];                      
          //\sum_{v_p \in V} x_{pi*}
          for (set<int>::iterator it1 = connectedCompVertexes.begin(); it1 != connectedCompVertexes.end(); it1++)
            if (1 <= *it1 && *it1 <= customers_size)
              i_start = *it1;
          if (i_start == -1)
            break;           
          for (int p = 0; p < total_size; p++)
            expr -= x[p][i_start];
          //\sum_{v_i \in V\S} \sum_{v_j \in S} x_{ij} - \sum_{v_p \in V} x_{pi*}>= 0
          add(expr >= 0);
          expr.end();
          expr = IloExpr(env);
        }
      }
    }
  }
}

Quadratic_model::Quadratic_model(Gvrp_instance _gvrp_instance): 
  gvrp_instance(_gvrp_instance) {
  }

list<list<Vertex> > Quadratic_model::run(){
  IloEnv env;
  IloCplex cplex(env);
  list<list<Vertex> > routes;
  int customers_size = gvrp_instance.customers.size(),
      afss_size = gvrp_instance.afss.size(),
      total_size = customers_size + afss_size + 1,
      NUM_MAX_VISITS_AFS = customers_size + 1;
  try {
    IloModel model(env);
    IloArray<IloNumVarArray> x (env, total_size),
      y(env, customers_size + 1),
      w(env, customers_size);
    IloNumVarArray e(env, total_size, 0, gvrp_instance.vehicleFuelCapacity, IloNumVar::Float);
    //x var
    for (int i = 0; i < total_size; i++)
      x[i] = IloNumVarArray(env, total_size, 0, 1, IloNumVar::Int);
    //y and w var
    for (int i = 0; i < customers_size; i++){
      y[i] = IloNumVarArray(env, customers_size + 1, 0, 1, IloNumVar::Int);
      w[i] = IloNumVarArray(env, afss_size, 0, NUM_MAX_VISITS_AFS, IloNumVar::Int);
    }
    //objective function
    IloExpr fo (env);
    for (int i = 0; i < total_size; i++)
      for (int j = 0; j < total_size; j++)
        fo +=  gvrp_instance.distances[i][j] * x[i][j];
    model.add(IloMinimize(env, fo));
    //constriaints
    IloExpr expr(env),
            expr1(env),
            expr2(env);
    //y[k][0] = 1, \forall k \in M
    for (int k = 0; k < customers_size; k++)
      model.add(y[k][0] == 1);
    //\sum_{k \in M} y_{i}^{k} = 1, \forall v_i \in C
    for (int i = 1; i <= customers_size; i++){
      for (int k = 0; k < customers_size; k++)
        expr += y[k][i];
      model.add(expr == 1);
      expr.end();
      expr = IloExpr(env);
    }
    //\sum_{j \in V} x_{ij} = \sum_{j \in V} x_{ji} = \sum_{k \in  M} y_{i}^k, \forall v_i \in C \cup \{v_0\}
    for (int i = 0; i <= customers_size; i++){
      for (int j = 0; j <= customers_size; j++){
        expr += x[i][j];     
        expr1 += x[j][i];    
      }
      for (int k = 0; k < customers_size; k++)
        expr2 += y[k][i];
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
          expr += x[customers_size + 1 + f][j];
          expr1 += x[j][customers_size + 1 + f];
        }
        model.add(w[k][f] == expr);
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
          expr = x[i][j] + y[k][j] - 1;
          model.add(y[k][i] >= expr);
          expr.end();
          expr = IloExpr(env);
          expr = x[i][j] + y[k][i] - 1;
          model.add(y[k][j] >= expr);
          expr.end();
          expr = IloExpr(env);
        }
    //e_0 = \beta
    model.add(e[0] == gvrp_instance.vehicleFuelCapacity);
    //e_f = \beta, \forall v_f \in F
    for (int f = 0; f < afss_size; f++)
      model.add(e[customers_size + 1 + f] == gvrp_instance.vehicleFuelCapacity);
    //e_j \leq e_i - c_{ij} x_{ij} + \beta (1 - x_{ij}), \forall v_j \in C, v_i \in V
    for (int j = 1; j <= customers_size; j++)
      for (int i = 0; i < total_size; i++){
        expr = e[i] - gvrp_instance.distances[i][j] * x[i][j] + gvrp_instance.vehicleFuelCapacity * (1 - x[i][j]);
        model.add(e[j] <= expr);
        expr.end();
        expr = IloExpr (env);
      }
    //e_i \geq c_{ij} x_{ij}, \forall v_i, v_j \in V
    for (int i = 0; i < total_size; i++)
      for (int j = 0; j < total_size; j++){
        expr = gvrp_instance.distances[i][j] * x[i][j];
        model.add(e[i] >= expr);
        expr.end();
        expr = IloExpr(env);
      }
    //e_i \geq x_{i0} c_{i0}
    for (int i = 0; i < total_size; i++){
      expr = x[i][0] * gvrp_instance.distances[i][0];
      model.add(e[i] >= expr);
      expr.end();
      expr = IloExpr(env);
    }
    //lazy
    cplex.use(Quadratic_subcycle_constraint(env, x, y, w, e, gvrp_instance)); 
    //run model
    IloCplex cplex(model);
    if ( !cplex.solve() ) {
      env.error() << "Failed to optimize LP." << endl;
      throw(-1);
    }
    env.out() << "Solution status = " << cplex.getStatus() << endl;
    env.out() << "Solution value = " << cplex.getObjValue() << endl;
  }catch (IloException& e) {
    cerr << "Concert exception caught: " << e << endl;
  }
  catch (...) {
    cerr << "Unknown exception caught" << endl;
  }
  env.end();
  return routes;
}
