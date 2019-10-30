#include <ilcplex/ilocplex.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <sstream>
#include "SampleConfig.h"
#include "vertex.hpp"
#include <list>
#include <vector>
ILOSTLBEGIN

using namespace std;

void version()
{
    cout << "Version : " << SAMPLE_VERSION_MAJOR <<"." << SAMPLE_VERSION_MINOR <<"." << SAMPLE_VERSION_PATCH << endl;
}

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
  for (list<vertex::Vertex>::iterator i = afss.begin(); i != afss.end(); i++)
    vertexes[j++] = *i;
  for (list<vertex::Vertex>::iterator i = customers.begin(); i != customers.end(); i++)
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
                            y_vars(env, customers_size),
                            w_vars(env, customers_size);
    IloNumVarArray e_vars(env, total_size, 0, vehicleFuelCapacity, IloNumVar::Int);
    //x var
    for (int i = 0; i < total_size; i++)
      x_vars[i] = IloNumVarArray(env, total_size, 0, 1, IloNumVar::Int);
    //y and w var
    for (int i = 0; i < customers_size; i++){
      y_vars[i] = IloNumVarArray(env, customers_size, 0, 1, IloNumVar::Int);
      w_vars[i] = IloNumVarArray(env, afss_size, 0, NUM_MAX_VISITS_AFS, IloNumVar::Int);
    }
    //objective function
    IloExpr fo (env);
    for (int i = 0; i < total_size; i++)
      for (int j = 0; j < total_size; j++)
        fo +=  distances[i][j] * x_vars[i][j];
    model.add(IloMaximize(env, fo));
    //constrins
    //...
//    model.add(IloMaximize(env, vars[0] + 2 * vars[1] + 3 * vars[2]));
//    model.add( - vars[0] +     vars[1] + vars[2] <= 20);
//    model.add(   vars[0] - 3 * vars[1] + vars[2] <= 30);
//
//    IloCplex cplex(model);
//    if ( !cplex.solve() ) {
//      env.error() << "Failed to optimize LP." << endl;
//      throw(-1);
//    }
//
//    IloNumArray vals(env);
//    env.out() << "Solution status = " << cplex.getStatus() << endl;
//    env.out() << "Solution value = " << cplex.getObjValue() << endl;
//    cplex.getValues(vals, vars);
//    env.out() << "Values = " << vals << endl;

    //variables
//    IloNumVarArray x()
  }catch (IloException& e) {
    cerr << "Concert exception caught: " << e << endl;
  }
  catch (...) {
    cerr << "Unknown exception caught" << endl;
  }
  env.end();

  return 0;
}
