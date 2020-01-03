#ifndef VERTEX_HPP_
#define VERTEX_HPP_

#include <string>
#include <sstream>

using namespace std;

namespace models {

  class Vertex {
    public:
      explicit Vertex(int id, double x, double y);
      explicit Vertex(int id, double x, double y, double serviceTime);
      Vertex();
      friend ostream& operator<<(ostream& strm, const Vertex& vertex){
        stringstream output;
        output<<"Vertex(ID:"<<vertex.id<<", X:"<<vertex.x<<", Y:"<<vertex.y<<", Time:"<<vertex.serviceTime<<");";
        return strm << output.str();
      }
      int id;
      double x;
      double y;
      double serviceTime;
  };

} 
#endif
