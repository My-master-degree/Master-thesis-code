#include <stdlib.h>

#include "utils/dsu.hpp"

using namespace utils;
using namespace std;

DSU::DSU (size_t n_) : n(n_) {
  size_t m = n * sizeof(int);
  pred = (int*) malloc (m);
  rank = (int*) malloc (m);
  for (size_t i = 0; i < n; pred[i] = i, rank[i++] = 0);
}

void DSU::join (size_t i, size_t j) {
  if (i != j && i < n && j < n) {
    size_t a = findSet(i);
    size_t b = findSet(j);
    if (rank[a] < rank[b]) 
      swap(a, b);
    pred[b] = a;
    if (rank[a] == rank[b])
      rank[a]++;
  }
}

void DSU::swap (size_t &a, size_t &b) {
  a ^= b;
  b ^= a;
  a ^= b;
}

size_t DSU::findSet (size_t i) {
  size_t j = i;
  if (pred[i] == i)
    return i;
  return pred[i] = pred[findSet(pred[i])];
}
