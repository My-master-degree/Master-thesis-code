#ifndef PREPROCESSING_COMPACT_MODEL_HPP_
#define PREPROCESSING_COMPACT_MODEL_HPP_

namespace utils {
  namespace cplex {
    class Compact_model;
    class Preprocessing_compact_model {
      protected:
        Compact_model& compact_model;
      public:
        explicit Preprocessing_compact_model (Compact_model& compact_model);
        virtual void add () = 0;
    };
  }
}

#endif
