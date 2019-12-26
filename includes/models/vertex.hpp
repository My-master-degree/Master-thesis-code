#ifndef VERTEX_HPP_
#define VERTEX_HPP_

#include <string>
#include <sstream>

using namespace std;

namespace models {

  class Vertex {
    public:
      explicit Vertex(int id, double x, double y);
      friend ostream& operator<<(ostream& strm, const Vertex& vertex){
        stringstream output;
        output<<"Vertex(ID:"<<vertex.id<<", X:"<<vertex.x<<", Y:"<<vertex.y<<");";
        return strm << output.str();
      }
      Vertex();
      int id;
      double x;
      double y;
  };

} 
#endif
