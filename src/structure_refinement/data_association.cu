#include "data_association.cuh"
#include <stdio.h>

namespace structure_refinement {

__global__ void associateDataKernel(int n, TreeNodeTypePtr *x) {
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  for (int i = index; i < n; i += stride) {
    Eigen::Vector3d querryPoint(5, 5, 5);
    printf("Index  %d\n", i);
    auto tmp = x[i]->bestMatchingLeafFast(querryPoint);
    printf("Index  %d, point: (%f,%f,%f)\n", i, tmp->mean_[0], tmp->mean_[1], tmp->mean_[2]);
  }
}

__host__ void DataAssociation::prepareData(std::vector<std::shared_ptr<TreeNodeType>> kdTrees) {

  // For test create vector without shared_ptr
  std::vector<TreeNodeTypePtr> kdTreePtrs;
  kdTreePtrs.reserve(kdTrees.size());
  TreeNodeTypePtr * devInPtr;

  for (int i=0; i < kdTrees.size(); i++)
  {
    // kdTreeObjVec[i] = *kdTrees.at(i);
    kdTreePtrs.push_back(kdTrees[i].get());
    printf("Adress: %p\n", kdTreePtrs.data()[i]);
  }

  // Copy the pointers to GPU
  size_t dataSize = kdTrees.size() * sizeof(TreeNodeTypePtr);
  cudaMalloc(&devInPtr, dataSize);
  cudaMemcpy(devInPtr, kdTreePtrs.data(), dataSize, cudaMemcpyHostToDevice);

  Eigen::Vector3d querryPoint(5, 5, 5);
  for (int i = 0; i < kdTreePtrs.size(); i++) {
    auto tmp = kdTreePtrs.at(i)->bestMatchingLeafFast(querryPoint);
    std::cout << "CPU point:  " << tmp->mean_[0] << " " << tmp->mean_[1] << " " << tmp->mean_[2] << std::endl;
  }
  // Call the kernel
  int numSMs;
  cudaDeviceGetAttribute(&numSMs, cudaDevAttrMultiProcessorCount, 0);
  associateDataKernel<<<1, 256>>>(kdTreePtrs.size(), devInPtr);


  // Alocate space for result
  std::vector<TreeNodeType> kdTreeMatch;
  kdTreeMatch.reserve(kdTrees.size());
  TreeNodeType *devOutPtr;

  // Malloc data on GPU for the kdTree
  // size_t dataSize = kdTrees.size() * sizeof(TreeNodeType);
  // kdTreeObj[0]->doNothing(5);
  // cudaMallocManaged((void **)&devInPtr, dataSize);
  // devInPtr->build()

  // cudaMallocManaged((void **)&devOutPtr, dataSize);

  // Copy memory
  // cudaMemcpy(devInPtr, kdTreeObj.data(), dataSize, cudaMemcpyHostToDevice);
  

  // Copy kdTrees leaf by leaf
  // Iterate through all leafs
  // for (int i =0; )
  //cudaMemcpy(d_y, y, N*sizeof(float), cudaMemcpyHostToDevice);

  // cudaMallocManaged(&devInPtr, size);
  // cudaMallocManaged(&devOutPtr, size);



  // Copy the results back
  // cudaMemcpy(kdTreeMatch.data(), devOutPtr ,dataSize, cudaMemcpyHostToDevice);

  cudaDeviceSynchronize();

    // // Free memory
  cudaFree(devInPtr);
  cudaFree(devOutPtr);

  std::cout << "Ended GPU computations " << std::endl;
  // Allocate memory on GPU for input
  // cudaMalloc(&dx, N *sizeof(float));
  // Allocate memory on GPU for output
  // cudaMalloc(&dy, N * sizeof(float));

  // x = (float *)malloc(N * sizeof(float));
  // y = (float *)malloc(N * sizeof(float));

  // For each point in point cloud find the closest one

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
