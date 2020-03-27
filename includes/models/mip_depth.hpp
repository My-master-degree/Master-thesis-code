#ifndef MIP_DEPTH_HPP_
#define MIP_DEPTH_HPP_

#include <ilcplex/ilocplex.h>

namespace models {
  class Depth : public IloCplex::MIPCallbackI::NodeData {
    public:
      explicit Depth(IloInt d); 
      IloInt const depth;
  };
}

#endif
