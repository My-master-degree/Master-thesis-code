#include "models/gvrp_instance.hpp"
#include "models/vertex.hpp"
#include <vector>
#include <list>
#include <string>

using namespace models;
using namespace std;

Gvrp_instance::Gvrp_instance(list<Vertex> _afss, list<Vertex> _customers, Vertex _depot, double _vehicleFuelCapacity, vector<vector<double> > _distances) :
  afss(_afss), customers(_customers), depot(_depot), vehicleFuelCapacity(_vehicleFuelCapacity), distances(_distances){
}

//std::ostream& operator<<(std::ostream &strm, const Gvrp_instance &a) {
//    return strm << "A(" << a.i << ")";
//  cout<<"Depot: "<<endl;
//  cout<<depot.id<<" "<<depot.x<<" "<<depot.y<<endl;
//  cout<<"AFSs: "<<endl;
//  for (list<vertex::Vertex>::iterator i = afss.begin(); i != afss.end(); i++)
//    cout<<i->id<<" "<<i->x<<" "<<i->y<<endl;
//  cout<<"Customers: "<<endl;
//  for (list<vertex::Vertex>::iterator i = customers.begin(); i != customers.end(); i++)
//    cout<<i->id<<" "<<i->x<<" "<<i->y<<endl;
//
//}

