#ifndef VERTEX_HPP_
#define VERTEX_HPP_

using namespace std;

namespace models {

  class Vertex {
    public:
      explicit Vertex(int id, double x, double y);
      Vertex();
      int id;
      double x;
      double y;
  };

} 
#endif
