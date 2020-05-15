#include "models/vertex.hpp"
#include "models/distances_enum.hpp"
#include "models/cplex/mip_solution_info.hpp"
#include "models/cplex/depth_node_callback.hpp"
#include "models/gvrp_models/gvrp_solution.hpp"
#include "models/gvrp_models/gvrp_instance.hpp"
#include "models/gvrp_models/cplex/gvrp_model.hpp"
#include "models/gvrp_models/cplex/lh_model/lh_model.hpp"
#include "models/gvrp_models/cplex/lh_model/preprocessing.hpp"
#include "models/gvrp_models/cplex/lh_model/extra_constraint.hpp"
#include "models/gvrp_models/cplex/lh_model/user_constraint.hpp"
#include "models/gvrp_models/cplex/lh_model/lazy_constraint.hpp"

#include <sstream>
#include <list>
#include <time.h> 
#include <string> 

using namespace models;
using namespace models::cplex;
using namespace models::gvrp_models;
using namespace models::gvrp_models::cplex;
using namespace models::gvrp_models::cplex::lh_model;

using namespace std;

LH_model::LH_model(const Gvrp_instance& instance, unsigned int time_limit) : Gvrp_model(instance, time_limit) {
  if (instance.distances_enum != METRIC)
    throw string("Error: The compact model requires a G-VRP instance with metric distances");
  //gvrp afs tree
  gvrp_afs_tree = new Gvrp_afs_tree(instance);
  //c_0
  c0 = vector<const Vertex *> (instance.customers.size() + 1);
  c0[0] = &instance.depot;
  int i = 0;
  for (const Vertex& customer : instance.customers) {
    c0[i + 1] = &customer;
    customersC0Indexes[customer.id] = i + 1;
    ++i;
  }
  //f_0
  f0 = vector<const Vertex *> (instance.afss.size() + 1);
  f0[0] = &instance.depot;
  afssF0Indexes[instance.depot.id] = 0;
  int f = 0;
  for (const Vertex& afs : instance.afss) {
    f0[f + 1] = &afs;
    afssF0Indexes[afs.id] = f + 1;
    ++f;
  }
} 

double LH_model::time (int i, int f, int j) {
  return instance.time(c0[i]->id, f0[f]->id) + instance.time(f0[f]->id, c0[j]->id);
}

double LH_model::time(int i, int j) {
  return instance.time(c0[i]->id, c0[j]->id);
}

double LH_model::fuel(int i, int j) {
  return instance.fuel(c0[i]->id, c0[j]->id);
}

double LH_model::M1(int i, int f, int j) {
  return instance.timeLimit + time(i, j) + time(i, f, j) - time(i, 0) - time(0, j);
}

double LH_model::M2(int i, int j) {
  double closestIAFS = instance.fuel(f0[0]->id, i),
         closestJAFS = instance.fuel(f0[0]->id, j);
  for (int f = 1; f < f0.size(); ++f) {
    closestIAFS = min(closestIAFS, instance.fuel(f0[f]->id, i));
    closestJAFS = min(closestIAFS, instance.fuel(f0[f]->id, j));
  }
  return instance.vehicleFuelCapacity + fuel(i, j) - closestIAFS - closestJAFS;
}

pair<Gvrp_solution, Mip_solution_info> LH_model::run(){
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
    cout<<"Setting parameter"<<endl;
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
    fillVals();
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

void LH_model::createVariables(){
  y = Matrix3DVar (env, c0.size());
  x = Matrix2DVar (env, c0.size());
  e = IloNumVarArray (env, instance.customers.size(), 0, instance.vehicleFuelCapacity, IloNumVar::Float);
  t = IloNumVarArray (env, instance.customers.size(), 0, instance.timeLimit, IloNumVar::Float);
  try {
    //setting names
    stringstream nameStream;
    for (size_t i = 1; i < c0.size(); ++i) {
      //e var
      nameStream<<"e["<<i - 1<<"]=energy of customer "<<c0[i]->id;
      e[i - 1].setName(nameStream.str().c_str());
      nameStream.clear();
      nameStream.str("");
      //t var
      nameStream<<"t["<<i - 1<<"]=time of customer "<<c0[i]->id;
      t[i - 1].setName(nameStream.str().c_str());
      nameStream.clear();
      nameStream.str("");
    }
    for (size_t i = 0; i < c0.size(); ++i) {
      //x var
      x[i] = IloNumVarArray (env, c0.size(), 0, 1, IloNumVar::Int);
      for (size_t j = 0; j < c0.size(); ++j) {
        nameStream<<"x["<<i<<"]["<<j<<"]=edge("<<c0[i]->id<<","<<c0[j]->id<<")";
        x[i][j].setName(nameStream.str().c_str());
        nameStream.clear();
        nameStream.str("");
      }
      //y var
      y[i] = Matrix2DVar (env, f0.size());
      for (size_t f = 0; f < f0.size(); ++f) {
        y[i][f] = IloNumVarArray(env, c0.size(), 0, 1, IloNumVar::Int);
        for (size_t j = 0; j < c0.size(); ++j) {
          nameStream<<"y["<<i<<"]["<<f<<"]["<<j<<"]=path("<<c0[i]->id<<","<<f0[f]->id<<","<<c0[j]->id<<")";
          y[i][f][j].setName(nameStream.str().c_str());
          nameStream.clear();
          nameStream.str("");
        }
      }
    }
  }catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating variables");
  }
}

void LH_model::createObjectiveFunction() {
  //objective function
  try{
    IloExpr fo (env);
    for (size_t i = 0; i < c0.size(); ++i) 
      for (size_t j = 0; j < c0.size(); ++j) {
        fo +=  instance.distances[c0[i]->id][c0[j]->id] * x[i][j];
        for (size_t f = 0; f < f0.size(); ++f)
          fo += (instance.distances[c0[i]->id][f0[f]->id] + instance.distances[f0[f]->id][c0[j]->id]) * y[i][f][j];
      }
    model = IloModel (env);
    model.add(IloMinimize(env, fo));
  } catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating the objective function");
  }
}

void LH_model::createModel() {
  try {
    //preprocessing conditions
    for (Preprocessing* preprocessing : preprocessings)
      preprocessing->add();
    //constraints
    IloExpr expr(env),
            expr1(env);    
    IloConstraint c;
    stringstream constraintName;
    //x_{ii} = 0, \forall v_i \in C_0
    for (size_t i = 0; i < c0.size(); ++i) 
      model.add(x[i][i] == 0);
    //y_{ifi} = 0, \forall v_i \in C_0, \forall v_f \in F
    for (size_t i = 0; i < c0.size(); ++i) 
      for (size_t f = 0; f < f0.size(); ++f)
        model.add(y[i][f][i] == 0);
    //\sum_{v_j \in C_0} (x_{ij} + \sum_{v_f \in F_0} y_{ifj}) = 1, \forall v_i \in C
    for (size_t i = 0; i < c0.size(); ++i) {
      for (size_t j = 0; j < c0.size(); ++j) {
        expr += x[i][j];
        for (size_t f = 0; f < f0.size(); ++f)
          expr += y[i][f][j];
      }
      c = IloConstraint (expr == 1);
      constraintName<<"customer "<<c0[i]->id<<" must be visited exactly once";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in C_0} ((x_{ij} - x_{ji}) + \sum_{v_f \in F_0} (y_{ifj} - y_{jfi})) = 0, \forall v_i \in C_0 
    for (size_t i = 0; i < c0.size(); ++i) {
      for (size_t j = 0; j < c0.size(); ++j) {
        expr += x[i][j] - x[j][i];
        for (size_t f = 0; f < f0.size(); ++f)
          expr += y[i][f][j] - y[j][f][i];
      }
      c = IloConstraint (expr == 0);
      constraintName<<c0[i]->id<<" flow conservation ";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    //\sum_{v_j \in C_0} (x_{0j} + \sum_{v_f \in F_0} y_{0fj}) \leqslant m 
    for (size_t j = 0; j < c0.size(); ++j) {
      expr += x[0][j];
      for (size_t f = 0; f < f0.size(); ++f)
        expr += y[0][f][j];
    }
    c = IloConstraint (expr <= instance.maxRoutes);
    constraintName<<instance.maxRoutes<<" routes must be used";
    c.setName(constraintName.str().c_str());
    model.add(c);
    expr.end();
    expr = IloExpr(env);
    constraintName.clear();
    constraintName.str("");
    // \tau_i - \tau_j + (M1_{ifj} - t_{ifj}) * x_{ij}
    //                + (M1_{ifj} - t_{ifj} - t_{ij} - t_{ji}) * x_{ji}
    //                + (M1_{ifj} - t_{ij}) * y_{ifj}
    //                + (M1_{ifj} - t_{ij} - t_{ifj} - t_{jfi}) * y_{jfi}
    //                \leqslant M1_{ifj} - t_{ij} - t_{ifj} 
    for (size_t i = 1; i < c0.size(); ++i)
      for (size_t j = 1; j < c0.size(); ++j) 
        for (size_t f = 0; f < f0.size(); ++f) {
          expr = t[i - 1] - t[j - 1] + (M1(i, f, j) - time(i, f, j)) * x[i][j] 
                              + (M1(i, f, j) - time(i, f, j) - time(i, j) - time(j, i)) * x[j][i]
                             + (M1(i, f, j) - time(i, j)) * y[i][f][j] 
                              + (M1(i, f, j) - time(i, j) - time(i, f, j) - time(j, f, i)) * y[j][f][i]
                              - (M1(i, f, j) - time(i, j) - time(i, f, j));
          c = IloConstraint (expr <= 0);
          constraintName<<"time path ("<<c0[i]->id<<", "<<f0[f]->id<<", "<<c0[j]->id<<")";
          c.setName(constraintName.str().c_str());
          model.add(c);
          expr.end();
          expr = IloExpr(env);
          constraintName.clear();
          constraintName.str("");
        }
    // \tau_j \geqslant t_{0j}) * x_{0j} + \sum_{v_f \in F_0} y_{0fj} t_{0fj} \forall v_j \in C
    for (size_t j = 1; j < c0.size(); ++j) {
      expr = time(0, j) * x[0][j];
      for (size_t f = 0; f < f0.size(); ++f) 
        expr += time(0, f, j) * y[0][f][j];
      c = IloConstraint (t[j - 1] >= expr);
      constraintName<<"customer "<<c0[j]->id<<" time lb";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    // \tau_j \leqslant T_{max} - (T_{max}^- t_{0j}) * x_{0j} - \sum_{v_f \in F_0} y_{0fj} * (T_{max} - t_{0fj}) \forall v_j \in C
    for (size_t j = 1; j < c0.size(); ++j) {
      expr = instance.timeLimit - (instance.timeLimit - time(0, j)) * x[0][j];
      for (size_t f = 0; f < f0.size(); ++f) 
        expr -= (instance.timeLimit - time(0, f, j)) * y[0][f][j];
      c = IloConstraint (t[j - 1] <= expr);
      constraintName<<"customer "<<c0[j]->id<<" time ub";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    // \tau_j \leqslant T_{max} - t_{j0} * x_{j0} - \sum_{v_f \in F_0} - t_{jf0} * y_{jf0} \forall v_j \in C
    for (size_t j = 1; j < c0.size(); ++j) {
      expr = instance.timeLimit - time(j, 0) * x[j][0];
      for (size_t f = 0; f < f0.size(); ++f) 
        expr -= time(0, f, j) * y[0][f][j];
      c = IloConstraint (t[j - 1] <= expr);
      constraintName<<"customer "<<c0[j]->id<<" time ub 2";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    // e_j - e_i + M2_{ij} * x_{ij} + (M2_{ij} - e_{ij} - e_{ji}) * x_{ji} \leqslant M2_{ij} - e_{ij} \forall v_i, v_j \in C
    for (size_t i = 1; i < c0.size(); ++i) 
      for (size_t j = 1; j < c0.size(); ++j) {
        expr = e[j - 1] - e[i - 1] + M2(i, j) * x[i][j] + (M2(i, j) - fuel(i, j) - fuel(j, i)) * x[j][i];
        c = IloConstraint (expr <= M2(i, j) - fuel(i, j));
        constraintName<<"edge ("<<c0[i]->id<<", "<<c0[j]->id<<") energy";
        c.setName(constraintName.str().c_str());
        model.add(c);
        expr.end();
        expr = IloExpr(env);
        constraintName.clear();
        constraintName.str("");
      }
    // e_j \leqslant \beta - e_{0j} * x_{0j} - \sum_{v_i \in C_0} \sum_{v_f \in F_0} e_{fj} * y_{ifj} \forall v_j \in C
    for (size_t j = 1; j < c0.size(); ++j) {
      expr = instance.vehicleFuelCapacity - fuel(0, j) * x[0][j];
      for (size_t i = 0; i < c0.size(); ++i) 
        for (size_t f = 0; f < f0.size(); ++f) 
          expr -= fuel(f, j) * y[i][f][j];
      c = IloConstraint (e[j - 1] <= expr);
      constraintName<<"customer "<<c0[j]->id<<" energy ub";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
    }
    // e_j \geqslant e_{j0} * x_{j0} + \sum_{v_i \in C_0} \sum_{v_f \in F_0} e_{jf} * y_{jfi} \forall v_j \in C
    for (size_t j = 1; j < c0.size(); ++j) {
      expr = fuel(j, 0) * x[j][0];
      for (size_t i = 0; i < c0.size(); ++i) 
        for (size_t f = 0; f < f0.size(); ++f) 
          expr += fuel(j, f) * y[j][f][i];
      c = IloConstraint (e[j - 1] >= expr);
      constraintName<<"customer "<<c0[j]->id<<" energy ub2";
      c.setName(constraintName.str().c_str());
      model.add(c);
      expr.end();
      expr = IloExpr(env);
      constraintName.clear();
      constraintName.str("");
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

void LH_model::extraStepsAfterModelCreation() {
  //
}

void LH_model::setCustomParameters(){
  try{
    setParameters();
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in setting parameters");
  }
}

void LH_model::fillVals(){
  //getresult
  try{
    x_vals = Matrix2DVal (env, c0.size());
    y_vals = Matrix3DVal (env, c0.size());
    for (size_t i = 0; i < c0.size(); ++i){
      x_vals[i] = IloNumArray (env, c0.size(), 0, 1, IloNumVar::Int);
      y_vals[i] = Matrix2DVal (env, f0.size());
      cplex.getValues(x_vals[i], x[i]);
      for (size_t f = 0; f < f0.size(); ++f){
        y_vals[i][f] = IloNumArray(env, c0.size(), 0, 1, IloNumVar::Int);
        cplex.getValues(y_vals[i][f], y[i][f]);
      }
    }
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting solution");
  }
}

void LH_model::createGvrp_solution(){
  try{
    list<list<Vertex>> routes;
    list<Vertex> route;
    int curr = 0;    
    route.push_back(Vertex(*c0[curr]));
    //checking the depot neighboring
    for (size_t i = 1; i < c0.size(); ) {
      //dfs
      if (x_vals[curr][i] > 0) {
        route.push_back(Vertex(*c0[i]));
        x_vals[curr][i] = 0;
        curr = i;
      } else
        for (size_t f = 0; f < f0.size(); ++f)
          if (y_vals[curr][f][i] > 0) {
            route.push_back(Vertex(*f0[f]));
            route.push_back(Vertex(*c0[i]));
            y_vals[curr][f][i] = 0;
            curr = i;
            break;
          }
      if (curr == i) {
        //route ended
        if (i == 0) {
          route.push_back(Vertex(*c0[0]));
          routes.push_back(route);
          route = list<Vertex> ();
        }
        i = 0;
      } else
        ++i;
    }
    solution = new Gvrp_solution(routes, instance);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting routes");
  }
}

void LH_model::endVars(){
  for (size_t i = 0; i < c0.size(); ++i) {
    x[i].end();
    for (size_t f = 0; f < f0.size(); ++f) {
      for (size_t j = 0; j < c0.size(); ++j) 
        y[i][f][j].end();
      y[i][f].end();
    }
    y[i].end();
  }
  x.end();
  y.end();
  e.end(); 
  t.end(); 
}
