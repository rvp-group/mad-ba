#pragma once

#include <vector>
//gg:
//Old stuff I wrte when in beloved Freiburg.
// This is  a thing to sample with removal from a set of weights, in log time

namespace srrg2_core {

  class PermutationSampler{
  public:
    PermutationSampler(const std::vector<double>& v);
    ~PermutationSampler();
    int sampleWithRemoval(double d);
    int sample(double d);
    void recoverWeights();

    inline double getSum() const {return _sum;}

  private:
    struct PermutationSamplerNode{
      double left_sum;
      int info;
      PermutationSamplerNode *left, *right, *parent;
      inline bool isLeaf(){
        return left==0 && right==0;
      }
      inline bool isRoot(){
        return parent==0;
      }
    };

    PermutationSamplerNode* _root;
    double _sum;
    double _total_sum;

    PermutationSamplerNode* binarySearch(double d);

    void remove (PermutationSamplerNode* n);

    double recoverWeights(PermutationSamplerNode* n);

    PermutationSamplerNode *  initializeTree(int level, PermutationSamplerNode* parent, 
                                             const std::vector<double>& v, int& index);


    PermutationSamplerNode * initializeTree(int level, const std::vector<double>& v, int& index);

    void destroyTree(PermutationSamplerNode * n);
  };


} //namespace srrg2_core
