#include "models/vertex.hpp"

using vertex::Vertex;

Vertex::Vertex(void):
  id(), x(), y() {}
Vertex::Vertex(int _id, double _x, double _y) :
  id(_id), x(_x), y(_y) {}

