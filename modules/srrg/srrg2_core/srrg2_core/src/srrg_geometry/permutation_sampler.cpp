#include <cmath>
#include <assert.h>
#include "permutation_sampler.h"
#include <iostream>

using namespace std;
namespace srrg2_core {

  PermutationSampler::PermutationSampler(const std::vector<double>& v){
    int level=(int)ceil(log2(v.size()))+1;
    int index=0;
    _root=initializeTree(level, 0, v, index);
    _total_sum=0;
    for (int i=0; i<(int)v.size(); i++){
      _total_sum+=v[i];
    }
    recoverWeights(_root);
    _sum=_total_sum;
  }

  PermutationSampler::~PermutationSampler(){
    destroyTree(_root);
    _sum=0;
    _root=0;
  }

  int PermutationSampler::sampleWithRemoval(double d){
    PermutationSamplerNode * n=binarySearch(d);
    if (n){
      remove(n);
      return n->info;
    }
    return -1;
  }

  int PermutationSampler::sample(double d){
    PermutationSamplerNode * n=binarySearch(d);
    if (n){
      return n->info;
    }
    return -1;
  }

  void PermutationSampler::recoverWeights(){
    recoverWeights(_root);
    _sum=_total_sum;
  }


  PermutationSampler::PermutationSamplerNode* PermutationSampler::binarySearch(double d) {
    PermutationSamplerNode* aux=_root;
    while (aux && ! aux->isLeaf()){
      if (d<=aux->left_sum)
	aux=aux->left;
      else{
	d-=aux->left_sum;
	aux=aux->right;
      }
    }
    return aux;
  }

  void PermutationSampler::remove(PermutationSampler::PermutationSamplerNode* n){
    // climb up and remove the value of the removed node from all
    // nodes for which the subtree is on the left
    assert (n->isLeaf());
    double v=n->left_sum;
    PermutationSamplerNode* np=n;
    while (n!=_root){
      n=n->parent;
      if (n->left==np){
	n->left_sum-=v;
      }
      np=n;
    }
    _sum-=v;
  }

  double PermutationSampler::recoverWeights(PermutationSampler::PermutationSamplerNode* n) {
    if (n->isLeaf()){
      return n->left_sum;
    }
    if (!n)
      return 0;
    double left_sum  = recoverWeights(n->left);
    double right_sum = recoverWeights(n->right);
    n->left_sum=left_sum;
    return left_sum+right_sum;
  }

  PermutationSampler::PermutationSamplerNode * 
      PermutationSampler::initializeTree(int level, PermutationSampler::PermutationSamplerNode* parent, 
					 const std::vector<double>& v, int& index){
    if (level==0)
      return 0;

    PermutationSamplerNode* n=new PermutationSamplerNode;
    n->parent=parent;
    if (level==1){
      n->left=0;
      n->right=0;
      n->info=index;
      n->left_sum = (index<(int)v.size()) ? v[index++] : 0;
    } else {
      n->left  = initializeTree (level-1, n, v, index);
      n->right = initializeTree (level-1, n, v, index);
    }
    return n;
  }

  void PermutationSampler::destroyTree(PermutationSampler::PermutationSamplerNode * n){
    if (!n)
      return;
    destroyTree(n->left);
    destroyTree(n->right);
    delete n;
  }


}
