#include "data_association.h"
#include <Eigen/src/Geometry/OrthoMethods.h>    
#include <ext/alloc_traits.h>                   
#include <math.h>                               
#include <stdint.h>                             
#include <iostream>                             
#include "srrg_system_utils/chrono.h"           
#include "structure_refinement/kdtree.hpp"      
#include "structure_refinement/surfel_matches.h"
#include "structure_refinement/surfelv2.h"      

namespace structure_refinement {

uint64_t Surfelv2::counter_ = 0;

void DataAssociation::associateDataKernelCPU(int numOfLeafs, int kdTreeAIdx, int kdTreeBIdx, TreeNodeTypePtr *kdTreeLeafesPtr, TreeNodeTypePtr *kdTreesPtr, SurfelMatches *matchPtr) {
#pragma omp parallel for
  for (int i = 0; i < numOfLeafs; i += 1) {
    matchPtr[i].matched = false;
    matchPtr[i].surfelA = kdTreeLeafesPtr[i];
    matchPtr[i].surfelB = kdTreesPtr[kdTreeBIdx]->bestMatchingLeafFast(kdTreeLeafesPtr[i]->mean_);
    TreeNodeTypePtr surfelTmp = kdTreesPtr[kdTreeAIdx]->bestMatchingLeafFast(matchPtr[i].surfelB->mean_);
    // If surfelA <-> is closest to surfelB and vice verse
    if (matchPtr[i].surfelA == surfelTmp) {
      float maxD = 1 * 0.3;               // 1.5
      float maxDNorm = 1 * 0.9;           // 3.0
      float maxAngle = 5 * M_PI / 180.0;  // Smaller value converges faster
      float d = (matchPtr[i].surfelA->mean_ - matchPtr[i].surfelB->mean_).norm();
      float dNorm = abs((matchPtr[i].surfelB->mean_ - matchPtr[i].surfelA->mean_).dot(matchPtr[i].surfelA->eigenvectors_.col(0)));
      if (d < maxD || dNorm < maxDNorm) {
        Eigen::Vector3f a = matchPtr[i].surfelA->eigenvectors_.col(0).cast<float>();
        Eigen::Vector3f b = matchPtr[i].surfelB->eigenvectors_.col(0).cast<float>();
        float angle = atan2(a.cross(b).norm(), a.dot(b));
        if (angle < maxAngle)
          matchPtr[i].matched = true;
      }
    }
  }
}

void DataAssociation::prepareDataCPU(std::vector<std::unique_ptr<TreeNodeType>> &kdTrees, std::vector<std::vector<TreeNodeTypePtr>> &kdTreeLeafes) {
  std::cout << "PreparaceDataCPU" << std::endl;

  srrg2_core::Chrono::ChronoMap _timings;
  srrg2_core::Chrono chGP2("ProcessingMatches", &_timings, false);

  std::vector<TreeNodeTypePtr> kdTreePtrs;
  kdTreePtrs.reserve(kdTrees.size());

  for (int i = 0; i < kdTrees.size(); i++)
    kdTreePtrs.push_back(kdTrees[i].get());

  for (int i = 0; i < kdTrees.size() - 1; i++) {
    // Copy the container for surfelMatches to GPU (output)
    std::vector<SurfelMatches> kdTreeMatches(kdTreeLeafes.at(i).size());
    if (i % 10 == 0)
      std::cout << "KdTree matching " << i << std::endl;
    for (int j = i + 1; j < kdTrees.size(); j++) {
      {
        srrg2_core::Chrono chGP2("Associating data", &_timings, false);
        associateDataKernelCPU(kdTreeLeafes.at(i).size(), i, j, kdTreeLeafes.at(i).data(), kdTreePtrs.data(), kdTreeMatches.data());
      }
      {
        srrg2_core::Chrono chGP2("ProcessingMatches", &_timings, false);
        processTheSurfelMatches(kdTreeMatches);
      }
    }
    int maxNum = 0;
    int idx = 0;
    for (int k = 0; k < surfels_.size(); k++) {
      int num = surfels_.at(k).leafs_.size();
      if (num > maxNum) {
        maxNum = num;
        idx = k;
      }
    }
    std::cout << "Num of surfels " << surfels_.size() << " Max surfels:  " << maxNum << std::endl;
  }
  std::cout << "Ended CPU computations " << std::endl;
  srrg2_core::Chrono::printReport(_timings);
}

// Potentially this can be parallelized, as matches contain only pairs of surfels
void DataAssociation::processTheSurfelMatches(std::vector<SurfelMatches> &matches) {
  for (int i = 0; i < matches.size(); i++) {
    if (matches[i].matched == false)
      continue;
    int idSurfelA = matches[i].surfelA->surfel_id_;
    int idSurfelB = matches[i].surfelB->surfel_id_;
    if ((idSurfelA != -1) && (idSurfelB != -1)) {
    } else if (idSurfelA != -1) {
      // Check if given surfel Already has leaf from that pose
      int pointCloudBIdx = matches[i].surfelB->pointcloud_id_;
      if (surfels_.at(idSurfelA).hasLeafFromPointCloud(pointCloudBIdx) == false)
        surfels_.at(idSurfelA).addLeaf(matches[i].surfelB);
    } else if (idSurfelB != -1) {
      int pointCloudAIdx = matches[i].surfelA->pointcloud_id_;
      if (surfels_.at(idSurfelB).hasLeafFromPointCloud(pointCloudAIdx) == false)
        surfels_.at(idSurfelB).addLeaf(matches[i].surfelA);

    } else {
      Surfelv2 newSurfel;
      newSurfel.addLeaf(matches.at(i).surfelA);
      newSurfel.addLeaf(matches.at(i).surfelB);
      surfels_.push_back(newSurfel);
    }
  }
}

void DataAssociation::filterSurfels() {
  surfels_.erase(std::remove_if(surfels_.begin(), surfels_.end(),
                                [](const Surfelv2 &s) { return (s.getMaxRadius() == 0); }),
                 surfels_.end());
}

void DataAssociation::resetSurfels() {
  surfels_.clear();
  Surfelv2::resetTheSurfelsCounter();  // Surfel's id is its position in a surfels_ vector, so it must be reset
}

void DataAssociation::decimateSurfels(int decimation) {
  int position = 0;
  surfels_.erase(std::remove_if(surfels_.begin(), surfels_.end(),
                                [&position, decimation](const Surfelv2 &) { return position++ % decimation != 0; }),
                 surfels_.end());
}

}  // namespace structure_refinement
