#ifndef DSU_HPP_
#define DSU_HPP_

namespace utils {

  class DSU {
    public:
      int * pred, * rank;
      size_t n;
      DSU (size_t n_); 
      void join (size_t i, size_t j);
      size_t findSet (size_t i);
    private:
      void swap (size_t &a, size_t &b);
  };

}

#endif
