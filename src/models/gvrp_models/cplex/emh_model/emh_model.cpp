#include "models/vertex.hpp"
#include "models/distances_enum.hpp"
#include "models/cplex/mip_solution_info.hpp"
#include "models/cplex/depth_node_callback.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/cplex/gvrp_model.hpp"
#include "models/gvrp_models/cplex/emh_model/emh_model.hpp"
#include "models/gvrp_models/cplex/emh_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/emh_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/emh_model/user_constraint.hpp"
#include "models/gvrp_models/cplex/emh_model/lazy_constraint.hpp"

#include <sstream>
#include <list>
#include <time.h> 
#include <string> 

using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::emh_model;

using namespace std;

EMH_model::EMH_model(const Gvrp_instance& instance, unsigned int time_limit) : Gvrp_model(instance, time_limit) {
  if (instance.distances_enum != METRIC)
    throw string("Error: The compact model requires a G-VRP instance with metric distances");
  //gvrp afs tree
  gvrp_afs_tree = new Gvrp_afs_tree(instance);
  //populating all map and customers set
  all[instance.depot.id] = &instance.depot;
  for (const Vertex& customer: instance.customers) {
    all[customer.id] = &customer;
    customers.insert(customer.id);
  }
  //creating dummies
  for (const Vertex& afs: instance.afss) {
    afs_dummies[afs.id].push_back(afs.id);
    all[afs.id] = &afs;
    dummies[afs.id] = &afs;
  }
  int dummy_id = all.rbegin()->second->id;
  //afs dummies
  for (const Vertex& afs: instance.afss)
    for (size_t i = 0; i < instance.customers.size(); ++i) {
      afs_dummies[afs.id].push_back(++dummy_id);
      all[dummy_id] = &afs;
      dummies[dummy_id] = &afs;
    }
  //depot dummies
  for (size_t i = 0; i < instance.customers.size() + 1; ++i) {
    afs_dummies[instance.depot.id].push_back(++dummy_id);
    all[dummy_id] = &instance.depot;
    dummies[dummy_id] = &instance.depot;
  }
} 

pair<Gvrp_solution, Mip_solution_info> EMH_model::run(){
  //setup
  stringstream output_exception;
  Mip_solution_info mipSolInfo;
  try {
    cout<<"Creating variables"<<endl;
    createVariables();
    cout<<"Creating objective function"<<endl;
    createObjectiveFunction();
    cout<<"Creating model"<<endl;
    createModel();
    cout<<"Setting parameters"<<endl;
    setCustomParameters();
    cout<<"Solving model"<<endl;
    time_t start = clock();
    if ( !cplex.solve() ) {
//      env.error() << "Failed to optimize LP." << endl;
      mipSolInfo = Mip_solution_info(-1, cplex.getStatus(), -1, -1);
      endVars();
      env.end();
      throw mipSolInfo;
    }
    double total_time =  (double) (clock() - start) / (double) CLOCKS_PER_SEC;
//    cplex.exportModel("cplexcpp.lp");
//    env.out() << "Solution value = " << cplex.getObjValue() << endl;
//    cout<<"Getting x values"<<endl;
    fillX_vals();
//    cout<<"Creating GVRP solution"<<endl;
    createGvrp_solution();
    mipSolInfo = Mip_solution_info(cplex.getMIPRelativeGap(), cplex.getStatus(), total_time, cplex.getObjValue());
    endVars();
    env.end();
    return make_pair(*solution, mipSolInfo);
  } catch (IloException& e) {
    output_exception<<"Concert exception caught: " << e<<endl;
    throw output_exception.str();
  } catch (string s) {
    throw s;
  }
}

void EMH_model::createVariables(){
  x = Matrix2DVar (env, all.size());
  e = IloNumVarArray (env, all.size(), 0, instance.vehicleFuelCapacity, IloNumVar::Float);
  t = IloNumVarArray (env, all.size(), 0, instance.timeLimit, IloNumVar::Float);
  try {
    //setting names
    stringstream nameStream;
    for (const pair<int, const Vertex *>& p : all){
      int i = p.first;
      //e var
      nameStream<<"e["<<i<<"]";
      e[i].setName(nameStream.str().c_str());
      nameStream.clear();
      nameStream.str("");
      //t var
      nameStream<<"t["<<i<<"]";
      t[i].setName(nameStream.str().c_str());
      nameStream.clear();
      nameStream.str("");
      //x var
      x[i] = IloNumVarArray(env, all.size(), 0, 1, IloNumVar::Int);
      for (const pair<int, const Vertex *>& p1 : all){
        int j = p1.first;
        nameStream<<"x["<<i<<"]["<<j<<"]";
        x[i][j].setName(nameStream.str().c_str());
        nameStream.clear();
        nameStream.str("");
      }
    }
  }catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating variables");
  }
}

void EMH_model::createObjectiveFunction() {
  //objective function
  try{
    IloExpr fo (env);
    for (const pair<int, const Vertex *>& p : all){
      int i = p.first;
      for (const pair<int, const Vertex *>& p1 : all){
        int j = p1.first;
        fo +=  instance.distances[p.second->id][p1.second->id] * x[i][j];
      }
    }  
    model = IloModel (env);
    model.add(IloMinimize(env, fo));
  } catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating the objective function");
  }
}

void EMH_model::createModel() {
  try {
    //preprocessing conditions
    for (Preprocessing* preprocessing : preprocessings)
      preprocessing->add();
    //setup
    int depot = instance.depot.id;
    double beta = instance.vehicleFuelCapacity;
    double T = instance.timeLimit;
    //constraints
    IloExpr expr(env),
            expr1(env);    
    IloConstraint c;
    stringstream constraintName;
    //x_{ii} = 0, \forall v_i \in V'
    for (const pair<int, const Vertex *>& p : all) {
      int i = p.first;
      model.add(x[i][i] == 0);
    }
    //\sum_{v_j \in V' : v_i \neq v_j} x_{ij} = 1, \forall v_i \in C
    for (const Vertex& customer : instance.customers){
      int i = customer.id;
      for (const pair<int, const Vertex *>& p1 : all) {
        int j = p1.first;
        if (i != j)
          expr += x[i][j];
      }
      c = IloConstraint (expr == 1);
      constraintName<<"customer "<<i<<" must be visited exactly once";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in V' : v_f \neq v_j} x_{fj} \leqslant 1, \forall v_f inF_0 
    for (const pair<int, const Vertex *>& p : dummies) {
      int f = p.first;
      for (const pair<int, const Vertex *>& p1 : all) {
        int j = p1.first;
        if (f != j)
          expr += x[f][j];
      }
      c = IloConstraint (expr <= 1);
      constraintName<<"dummy "<<f<<" must be used at most once";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in V' : v_j \neq v_i} x_{ij} = \sum_{v_j \in V' : v_j \neq v_i} x_{ji}, \forall v_i \in V'
    for (const pair<int, const Vertex *>& p : all){
      int i = p.first;
      for (const pair<int, const Vertex *>& p1 : all){
        int j = p1.first;
        if (i != j) {
          expr += x[i][j];
          expr1 += x[j][i]; 
        }
      }
      c = IloConstraint (expr == expr1);
      constraintName<<i<<" # entering edges == "<<i<<" # exiting edges";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr1.end();
      expr = IloExpr(env);
      expr1 = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }    
    //\sum_{v_j \in V' : v_j \neq v_0} x_{0j} \leqslant m
    for (const pair<int, const Vertex *>& p : all)
      expr += x[depot][p.first];
    c = IloConstraint (expr <= instance.nRoutes);
    c.setName(string(" # routes limit == ").c_str());
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    //t_j \geq t_i + (t_{ij}  + p_j) * x_{ij} - T (1 - x_{ij}), \forall v_j \in V\backslash\{v_0\},\forall v_i \in V
    for (const pair<int, const Vertex *>& p1 : all) {
      int j = p1.first;
      if (j != depot) {
        for (const pair<int, const Vertex *>& p : all) {
          int i = p.first;
          if (i != j) {
            expr = t[i] + x[i][j] * ((instance.distances[p.second->id][p1.second->id] / instance.vehicleAverageSpeed) + p1.second->serviceTime) - T * (1 -  x[i][j]);
            c = IloConstraint (t[j] >= expr);
            c.setName("updating time");
            model.add(c);
            expr.end();
            expr = IloExpr (env);
          }
        }
      }
    }
    //t_{0j} \leq t_j \leq T - t_{j0}_ , \forall v_j \in V\backslash\{v_0\}
    for (const pair<int, const Vertex *>& p : all) {
      int j = p.first;
      if (j != depot) {
        c = IloConstraint (t[j] >= instance.distances[depot][p.second->id] / instance.vehicleAverageSpeed);
        c.setName("time variable lb");
        model.add(c);
        c = IloConstraint (t[j] <= T - instance.distances[p.second->id][depot] / instance.vehicleAverageSpeed);
        c.setName("time variable ub");
        model.add(c);
      }
    }
    //e_f = \beta, \forall v_f \in F_0
    for (const pair<int, const Vertex *>& p : dummies) {
      c = IloConstraint (e[p.second->id] == beta);
      c.setName("e_f = beta");
      model.add(c);
    }
    c = IloConstraint (e[depot] == beta);
    c.setName("e_depot = beta");
    model.add(c);
    //e_j \leq e_i - c_{ij} x_{ij} + \beta (1 - x_{ij}), \forall v_j \in C,\forall v_i \in V'
    for (const Vertex& customer : instance.customers) {
      int j = customer.id;
      for (const pair<int, const Vertex *>& p : all) {
        int i =  p.first;
        if (i != j) {
          expr = e[i] - x[i][j] * instance.distances[p.second->id][j] * instance.vehicleFuelConsumptionRate + beta * (1 -  x[i][j]);
          c = IloConstraint (e[j] <= expr);
          c.setName("updating fuel level");
          model.add(c);
          expr.end();
          expr = IloExpr (env);
        }
      }
    }
    //e_i \geq c_{ij} x_{ij}, \forall v_i, \forall v_j \in V'
    for (const pair<int, const Vertex *>& p : all) {
      int i = p.first;
      for (const pair<int, const Vertex *>& p1 : all) {
        int j = p1.first;
        expr = instance.distances[p.second->id][p1.second->id] * x[i][j] * instance.vehicleFuelConsumptionRate;
        c = IloConstraint (e[i] >= expr);
        c.setName("disabling infeasible edges");
        model.add(c);
        expr.end();
        expr = IloExpr(env);
      }
    }
    //extra constraints
    for (Extra_constraint* extra_constraint : extra_constraints) 
      extra_constraint->add();
    //init
    cplex = IloCplex(model);
    //lazy cuts
    for (User_constraint* user_constraint : user_constraints)
      cplex.use(user_constraint);
    //user cuts
    for (User_constraint* user_constraint : user_constraints)
      cplex.use(user_constraint);
    //extra steps
    extraStepsAfterModelCreation();
    //depth node callback
    cplex.use(new Depth_node_callback(env));
  } catch (IloException& e) {
    throw e;
  } catch (string s) {
    throw s;
  }
}

void EMH_model::extraStepsAfterModelCreation() {
  //
}

void EMH_model::setCustomParameters(){
  try{
    setParameters();
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in setting parameters");
  }
}

void EMH_model::fillX_vals(){
  //getresult
  try{
    x_vals = Matrix2DVal (env, all.size());
    for (const pair<int, const Vertex *>& p : all){
      int i = p.first;
      x_vals[i] = IloNumArray (env, all.size(), 0, 1, IloNumVar::Int);
      cplex.getValues(x_vals[i], x[i]);
    }
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting solution");
  }
}

void EMH_model::createGvrp_solution(){
  try{
    list<list<Vertex> > routes;
    list<Vertex> route;
    int depot = instance.depot.id;
    int curr,
        next;    
    //checking the depot neighboring
    for (const pair<int, const Vertex *>& p : all){
      //dfs
      curr = depot;
      next = p.first; 
      if (x_vals[curr][next] > 0 && curr != next) {
        route.push_back(Vertex(*all[curr]));
        for (; next != depot; route.push_back(Vertex(*all[curr]))) {
          curr = next;
          for (const pair<int, const Vertex *>& p1 : all) {
            int j = p1.first;
            if (x_vals[curr][j] > 0) {
              next = j;
              break;
            }
          }
        }
        route.push_back(Vertex(*all[next]));
        routes.push_back(route);
        route = list<Vertex> ();
      }
    }
    solution = new Gvrp_solution(routes, instance);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting routes");
  }
}

void EMH_model::endVars(){
  for (const pair<int, const Vertex *>& p : all)
    x[p.first].end();
  x.end();
  e.end(); 
  t.end(); 
}
