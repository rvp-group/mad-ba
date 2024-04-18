#include "data_association.cuh"
#include <stdio.h>
#include <srrg_system_utils/chrono.h>


namespace structure_refinement {

uint64_t Surfelv2::counter_ = 0;

__global__ void associateDataKernel(int numOfLeafs, int kdTreeAIdx, int kdTreeBIdx, TreeNodeTypePtr *kdTreeLeafesPtr, TreeNodeTypePtr *kdTreesPtr, SurfelMatches *matchPtr) {
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  for (int i = index; i < numOfLeafs; i += stride) {
    // printf("I:  %d ,IndexA: %d, IndexB: %d\n",i, kdTreeAIdx, kdTreeBIdx);
    matchPtr[i].matched = false;
    matchPtr[i].surfelA = kdTreeLeafesPtr[i];
    matchPtr[i].surfelB = kdTreesPtr[kdTreeBIdx]->bestMatchingLeafFast(kdTreeLeafesPtr[i]->mean_);
    TreeNodeTypePtr surfelTmp = kdTreesPtr[kdTreeAIdx]->bestMatchingLeafFast(matchPtr[i].surfelB->mean_);
    // If surfelA <-> is closest to surfelB and vice verse
    if (matchPtr[i].surfelA == surfelTmp) {
      float maxD = 0.3;
      float maxDNorm = 0.9;
      float maxAngle = 5 * M_PI / 180.0;
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

void associateDataKernelCPU(int numOfLeafs, int kdTreeAIdx, int kdTreeBIdx, TreeNodeTypePtr *kdTreeLeafesPtr, TreeNodeTypePtr *kdTreesPtr, SurfelMatches *matchPtr) {
  for (int i = 0; i < numOfLeafs; i += 1) {
    // printf("I:  %d ,IndexA: %d, IndexB: %d\n",i, kdTreeAIdx, kdTreeBIdx);
    matchPtr[i].matched = false;
    matchPtr[i].surfelA = kdTreeLeafesPtr[i];
    matchPtr[i].surfelB = kdTreesPtr[kdTreeBIdx]->bestMatchingLeafFast(kdTreeLeafesPtr[i]->mean_);
    TreeNodeTypePtr surfelTmp = kdTreesPtr[kdTreeAIdx]->bestMatchingLeafFast(matchPtr[i].surfelB->mean_);
    // If surfelA <-> is closest to surfelB and vice verse
    if (matchPtr[i].surfelA == surfelTmp) {
      float maxD = 1 * 1.5;  // 1.5
      float maxDNorm = 1 * 3.0; // 3.0
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

__host__ void DataAssociation::prepareData(std::vector<std::shared_ptr<TreeNodeType>> &kdTrees, std::vector<std::vector<TreeNodeTypePtr>> &kdTreeLeafes) {
  srrg2_core::Chrono::ChronoMap _timings;
  {
    // Converting shared_ptr to ptr
    std::vector<TreeNodeTypePtr> kdTreePtrs;
    kdTreePtrs.reserve(kdTrees.size());
    TreeNodeTypePtr *devKdTreesPtr;
    srrg2_core::Chrono chGP2wew1("ProcessingMatches", &_timings, false);

    for (int i = 0; i < kdTrees.size(); i++)
      kdTreePtrs.push_back(kdTrees[i].get());

    int numSMs;
    cudaDeviceGetAttribute(&numSMs, cudaDevAttrMultiProcessorCount, 0);

    // Copy the pointers of kdTrees to GPU (2nd input param)
    size_t dataSize = kdTreePtrs.size() * sizeof(TreeNodeTypePtr);
    {
      srrg2_core::Chrono chGP2("Malloc devKdTreesPtr", &_timings, false);
      cudaMalloc(&devKdTreesPtr, dataSize);
      cudaMemcpy(devKdTreesPtr, kdTreePtrs.data(), dataSize, cudaMemcpyHostToDevice);
    }

    for (int i = 0; i < kdTrees.size() - 1; i++) {

      // Copy the leafs for given kdTree (1st input param)
      TreeNodeTypePtr *devKdTreeLeafesPtr;
      size_t dataSizeLeafs = kdTreeLeafes.at(i).size() * sizeof(TreeNodeTypePtr);
      {
        srrg2_core::Chrono chGP2("Malloc devKdTreeLeafesPtr", &_timings, false);
        cudaMalloc(&devKdTreeLeafesPtr, dataSizeLeafs);
        cudaMemcpy(devKdTreeLeafesPtr, kdTreeLeafes.at(i).data(), dataSizeLeafs, cudaMemcpyHostToDevice);
      }

      // Copy the container for surfelMatches to GPU (output)
      std::vector<SurfelMatches> kdTreeMatches(kdTreeLeafes.at(i).size());
      SurfelMatches *devSurfelMatchesPtr;

      size_t dataSizeMatches = kdTreeMatches.size() * sizeof(SurfelMatches);
      {
        srrg2_core::Chrono chGP2("Malloc devSurfelMatchesPtr", &_timings, false);
        cudaMalloc(&devSurfelMatchesPtr, dataSizeMatches);
      }

      // Copy the results back
      // std::vector<SurfelMatches> kdTreeMatches(kdTreeLeafes.at(i).size());
      // // kdTreeMatches.reserve(kdTreeLeafes.at(0).size());
      // SurfelMatches *devSurfelMatchesPtr;

      // size_t dataSizeMatches = kdTreeMatches.size() * sizeof(SurfelMatches);
      // cudaMalloc(&devSurfelMatchesPtr, dataSizeMatches);
      for (int j = i + 1; j < kdTrees.size(); j++) {
      // Copy the results back
      // std::vector<SurfelMatches> kdTreeMatches(kdTreeLeafes.at(i).size());
      // // kdTreeMatches.reserve(kdTreeLeafes.at(0).size());
      // SurfelMatches *devSurfelMatchesPtr;

      // size_t dataSizeMatches = kdTreeMatches.size() * sizeof(SurfelMatches);
      // cudaMalloc(&devSurfelMatchesPtr, dataSizeMatches);
        {
          srrg2_core::Chrono chGP2("Calling GPU kernel", &_timings, false);
          associateDataKernel<<<32 * numSMs, 256>>>(kdTreeLeafes.at(i).size(), i, j, devKdTreeLeafesPtr, devKdTreesPtr, devSurfelMatchesPtr);
        }
        {
          srrg2_core::Chrono chGP2("Memcopy matches", &_timings, false);
          cudaMemcpy(kdTreeMatches.data(), devSurfelMatchesPtr, dataSizeMatches, cudaMemcpyDeviceToHost);
        }
        {
          srrg2_core::Chrono chGP2("Processing surfel matches", &_timings, false);
          processTheSurfelMatches(kdTreeMatches);
        }
        std::cout << "KdTree matching " << i << " with " << j << std::endl;
      }
      cudaFree(devSurfelMatchesPtr);
      cudaFree(devKdTreeLeafesPtr);
    }

  cudaDeviceSynchronize();

    // // Free memory
    cudaFree(devKdTreesPtr);
  // cudaFree(devOutPtr);
  // cudaFree(devSurfelMatchesPtr);

  std::cout << "Ended GPU computations " << std::endl;
  // surfels_.clear();
  // Surfelv2::counter_ = 0;
  }
  srrg2_core::Chrono::printReport(_timings);
}

__host__ void DataAssociation::prepareDataCPU(std::vector<std::shared_ptr<TreeNodeType>> & kdTrees, std::vector<std::vector<TreeNodeTypePtr>> & kdTreeLeafes) {


  srrg2_core::Chrono::ChronoMap _timings;
  srrg2_core::Chrono chGP2("ProcessingMatches", &_timings, false);

  std::vector<TreeNodeTypePtr> kdTreePtrs;
  kdTreePtrs.reserve(kdTrees.size());

  for (int i=0; i < kdTrees.size(); i++)
    kdTreePtrs.push_back(kdTrees[i].get());

  for (int i = 0; i < kdTrees.size() - 1; i++) {

    // Copy the container for surfelMatches to GPU (output)
    std::vector<SurfelMatches> kdTreeMatches(kdTreeLeafes.at(i).size());
    for (int j = i + 1; j < kdTrees.size(); j++) {
      std::cout << "KdTree matching " << i << " with " << j << std::endl;
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
__host__ void DataAssociation::processTheSurfelMatches(std::vector<SurfelMatches> &matches) {

  for (int i = 0; i < matches.size(); i++) {
    if (matches[i].matched == false)
      continue;
    int idSurfelA = matches[i].surfelA->surfel_id_;
    int idSurfelB = matches[i].surfelB->surfel_id_;
    if ((idSurfelA != -1) && (idSurfelB != -1)){
    }
    else if (idSurfelA != -1){
      // std::cout << "SurfelA found" << std::endl;
      // Check if given surfel Already has leaf from that pose
      int pointCloudBIdx =  matches[i].surfelB->pointcloud_id_;
      if (surfels_.at(idSurfelA).hasLeafFromPointCloud(pointCloudBIdx) == false)
        surfels_.at(idSurfelA).addLeaf(matches[i].surfelB);
      // std::cout << "Matched1 "  << matches[i].surfelA->pointcloud_id_ << " with "  << matches[i].surfelB->pointcloud_id_ << std::endl;
    }
    else if (idSurfelB != -1){
      // std::cout << "SurfelB found" << std::endl;
      int pointCloudAIdx = matches[i].surfelA->pointcloud_id_;
      if (surfels_.at(idSurfelB).hasLeafFromPointCloud(pointCloudAIdx) == false)
        surfels_.at(idSurfelB).addLeaf(matches[i].surfelA);
      // std::cout << "Matched2 " << matches[i].surfelA->pointcloud_id_ << " with " << matches[i].surfelB->pointcloud_id_ << std::endl;

    }
    else {
      Surfelv2 newSurfel;
      // std::cout << "Surfel ID " << newSurfel.id_ << std::endl;
      newSurfel.addLeaf(matches.at(i).surfelA);
      newSurfel.addLeaf(matches.at(i).surfelB);
      surfels_.push_back(newSurfel);
      // std::cout << "Matched3 " << matches[i].surfelA->pointcloud_id_ << " with " << matches[i].surfelB->pointcloud_id_ << std::endl;
    }
  }
}


__host__ void DataAssociation::prepareDataExample() {
  // int N = 1 << 20;
  // float *x, *y;

  // // Allocate Unified Memory – accessible from CPU or GPU
  // cudaMallocManaged(&x, N * sizeof(float));
  // cudaMallocManaged(&y, N * sizeof(float));

  // // initialize x and y arrays on the host
  // for (int i = 0; i < N; i++) {
  //   x[i] = 1.0f;
  //   y[i] = 2.0f;
  // }

  // int numSMs;
  // cudaDeviceGetAttribute(&numSMs, cudaDevAttrMultiProcessorCount, 0);
  // std::cout << "numSMs: " << numSMs << std::endl;
  // // Run kernel on 1M elements on the GPU
  // associateDataKernel<<<32 * numSMs, 256>>>(N, x, y);
  // // Copy data back
  // // cudaMemcpy(y, d_y, N*sizeof(float), cudaMemcpyDeviceToHost);

  // // Wait for GPU to finish before accessing on host
  // cudaDeviceSynchronize();

  // // Check for errors (all values should be 3.0f)
  // float maxError = 0.0f;
  // for (int i = 0; i < N; i++)
  //   maxError = fmax(maxError, fabs(y[i] - 3.0f));
  // std::cout << "Max error: " << maxError << std::endl;

  // // Free memory
  // cudaFree(x);
  // cudaFree(y);
}



}  // namespace structure_refinement
