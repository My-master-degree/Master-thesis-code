#include "models/mip_solution_info.hpp"
#include "models/cplex_model.hpp"
#include "models/vrp_instance.hpp"
#include "models/gvrp_instance.hpp"
#include "models/instance_generation_model/instance_generation_model.hpp"
#include "models/instance_generation_model/lazy_constraint.hpp"
#include "models/instance_generation_model/subcycle_lazy_constraint.hpp"
#include "models/instance_generation_model/user_constraint.hpp"
#include "models/instance_generation_model/subcycle_user_constraint.hpp"

#include <list>
#include <stdlib.h>
#include <exception>
#include <sstream>
#include <time.h>
#include <ilcplex/ilocplex.h>

using namespace std;
using namespace models;
using namespace models::instance_generation_model;

Instance_generation_model::Instance_generation_model(Vrp_instance& vrp_instance, unsigned int time_limit): Cplex_model(vrp_instance, time_limit) {
  if (instance.distances_enum != SYMMETRIC && instance.distances_enum != METRIC)
    throw string("Error: The compact model requires a G-VRP instance with symmetric or metric distances");
  sNodes = instance.customers.size() + 1;
}

pair<Gvrp_instance, Mip_solution_info> Instance_generation_model::run () {
  if (instance.distances_enum != SYMMETRIC && instance.distances_enum != METRIC)
    throw string("Error: The instance generation model requires a G-VRP instance with symmetric or metric distances");
  //setup
  stringstream output_exception;
  Mip_solution_info mipSolInfo;
  try {
//    cout<<"Creating variables"<<endl;
    createVariables();
//    cout<<"Creating objective function"<<endl;
    createObjectiveFunction();
//    cout<<"Creating model"<<endl;
    createModel();
//    cout<<"Setting parameters"<<endl;
    setCustomParameters();
//    cout<<"Solving model"<<endl;
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

void Instance_generation_model::createVariables(){
  try {
    stringstream nameStream;
    x = Matrix2DVar (env, sNodes);
    z = IloNumVarArray (env, sNodes, 0, 1, IloNumVar::Int);
    //x var
    for (size_t i = 0; i < sNodes; i++){
      x[i] = IloNumVarArray(env, sNodes, 0, 1, IloNumVar::Int);
      nameStream<<"z["<<i<<"]";
      z[i].setName(nameStream.str().c_str());
      nameStream.clear();
      nameStream.str("");
      //setting names
      for (size_t j = 0; j < sNodes; j++){
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

void Instance_generation_model::createObjectiveFunction() {
//objective function
  try{
    IloExpr fo (env);
    for (size_t i = 0; i < sNodes; i++)
      //setting names
      for (size_t j = 0; j < sNodes; j++)
        fo +=  instance.distances[i][j] * x[i][j];
    model = IloModel (env);
    model.add(IloMinimize(env, fo));
  } catch (IloException& e) {
    throw e;
  } catch(...){
    throw string("Error in creating the objective function");
  }
}

void Instance_generation_model::createModel() {
  try {
    //setup
    int depot = instance.depot.id;
    //constraints
    IloExpr expr(env),
            expr1(env);    
    IloConstraint c;
    stringstream constraintName;
    //\sum_{v_j \in V : v_i \neq v_j} x_{ij} = 1, \forall v_i \in V 
    for (size_t i = 0; i < sNodes; i++) {
      //setting names
      for (size_t j = 0; j < sNodes; j++)
        if (i != j)
          expr += x[i][j];
      c = IloConstraint (expr == 1);
      c.setName("customer i allocated exactly to one facility");
      model.add(c);
      expr.end();
      expr = IloExpr(env);
    }
    //x_{ij} - z_j \leqslant 0, \forall v_i, v_j \in V : v_ \neq v_j
    for (size_t i = 0; i < sNodes; i++) 
      //setting names
      for (size_t j = 0; j < sNodes; j++)
        if (i != j) {
          expr = x[i][j] - z[j];
          c = IloConstraint (expr <= 0);
          c.setName("a customer can only be allocated to a facility");
          model.add(c);
          expr.end();
          expr = IloExpr(env);
        }
    //z_0 = 1
    c = IloConstraint (z[depot] == 1);
    c.setName("depot is a facility");
    model.add(c);
    //init
    cplex = IloCplex(model);
    cplex.use(separation_algorithm()); 
    //user cuts
    for (User_constraint* user_constraint : user_constraints)
      cplex.use(user_constraint);
  } catch (IloException& e) {
    throw e;
  } catch (string s) {
    throw s;
  }
}

Lazy_constraint* Instance_generation_model::separation_algorithm(){
  return new Subcycle_lazy_constraint(*this);
}

void Instance_generation_model::setCustomParameters(){
  try{
    setParameters();
    //DOUBTS:
    // Turn off the presolve reductions and set the CPLEX optimizer
    // to solve the worker LP with primal simplex method.
    cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 1);
    cplex.setParam(IloCplex::Param::Preprocessing::Symmetry, 0);
    cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Primal); 
    //preprocesing setting
    cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse); 
    // Turn on traditional search for use with control callbacks
//    cplex.setParam(IloCplex::Param::MIP::Strategy::Search, IloCplex::Traditional);
    //:DOUBTS
    //LAZY CONSTRAINTS
    //thread safe setting
    cplex.setParam(IloCplex::Param::Threads, 1);
    // Tweak some CPLEX parameters so that CPLEX has a harder time to
    // solve the model and our cut separators can actually kick in.
    cplex.setParam(IloCplex::Param::MIP::Strategy::HeuristicFreq, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::MIRCut, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::Implied, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::Gomory, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::FlowCovers, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::PathCut, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::LiftProj, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::ZeroHalfCut, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::Cliques, -1);
    cplex.setParam(IloCplex::Param::MIP::Cuts::Covers, -1);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in setting parameters");
  }
}

void Instance_generation_model::fillVals(){
  //getresult
  try{
    x_vals = Matrix2DVal (env, sNodes);
    z_vals = IloNumArray (env, sNodes);
    cplex.getValues(z_vals, z);
    for (size_t i = 0; i < sNodes; i++) {
      x_vals[i] = IloNumArray (env, sNodes, 0, 1, IloNumVar::Int);
      cplex.getValues(x_vals[i], x[i]);
    }
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting solution");
  }
}

void Instance_generation_model::createGvrp_solution(){
  try{
    list<Vertex> customers,
                 afss;
    double vehicleFuelCapacity, 
           timeLimit, 
           vehicleFuelConsumptionRate, 
           vehicleAverageSpeed;
    for (const Vertex& v : instance.customers)
      if (z_vals[v.id] > 0)
        afss.push_back(v);
      else
        customers.push_back(v);
    //get \beta
    vehicleFuelCapacity = 60;
    vehicleFuelConsumptionRate = 0.2;
    //get T
    timeLimit = 11; 
    vehicleAverageSpeed = 40;
    solution = new Gvrp_instance(afss, customers, instance.depot, vehicleFuelCapacity, instance.distances, instance.distances_enum, customers.size(), timeLimit, vehicleFuelConsumptionRate, vehicleAverageSpeed);
  } catch (IloException& e) {
    throw e;
  } catch (...) {
    throw string("Error in getting routes");
  }
}

void Instance_generation_model::endVars(){
  for (size_t i = 0; i < sNodes; i++)
    x[i].end();
  x.end();
  z.end(); 
}
