#ifndef DEPTH_NODE_CALLBACK_HPP_
#define DEPTH_NODE_CALLBACK_HPP_

#include <ilcplex/ilocplex.h>

namespace models {
  class Depth_node_callback : public IloCplex::NodeCallbackI {
    public:
      explicit Depth_node_callback (IloEnv env);
      void main() override;
      [[nodiscard]] IloCplex::CallbackI* duplicateCallback() const override;
  };
}

#endif
