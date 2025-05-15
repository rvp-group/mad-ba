#pragma once
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <memory>

namespace srrg2_solver {
  using Isometry3fVector = std::vector <Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> >;
  using Isometry3f = Eigen::Isometry3f;
  using Matrix3f   = Eigen::Matrix3f;
  using Vector3f   = Eigen::Vector3f;

  struct TrajectoryGeneratorBase {
    TrajectoryGeneratorBase(Isometry3fVector& path_):
      poses(path_){}
  
    virtual void compute()=0;
    virtual ~TrajectoryGeneratorBase(){};
    virtual void parseArgs(const float* argv) = 0;
    Isometry3fVector& poses;
  };

  using TrajectoryGeneratorPtr = std::shared_ptr<TrajectoryGeneratorBase>;

  struct TrajectoryGeneratorSphere : public TrajectoryGeneratorBase{
    TrajectoryGeneratorSphere(Isometry3fVector& path_);
    void parseArgs(const float* argv) override;
    void compute() override;
    float radius=10.f;
    int azimuth_steps=20;
    int elevation_steps=20;
  };

  struct TrajectoryGeneratorSphereUniform : public TrajectoryGeneratorBase{
    TrajectoryGeneratorSphereUniform(Isometry3fVector& path_);
    void parseArgs(const float* argv) override;
    void compute() override;
    float radius=10.f;
    int steps=20;
  };

  struct TrajectoryGeneratorTorus : public TrajectoryGeneratorBase{
    TrajectoryGeneratorTorus(Isometry3fVector& path_);

    void parseArgs(const float* argv) override;
    void compute();
    float outer_radius=30.f;
    int outer_steps=20;
    float inner_radius=5.f;
    int inner_steps=20;
    int laps=1;
  };

  struct TrajectoryGeneratorRandomManhattan : public TrajectoryGeneratorBase{
    TrajectoryGeneratorRandomManhattan(Isometry3fVector& path_);
    int grid_size=30;
    float prob_turn=0.5;
    int r,  c; // cell coordinates
    int heading; // 0-3 right, top, left, bottom
    int steps=1000;
    int step_size=5;
    
    // true if agent can move forward, if true, it performs a motion
    bool canMove();
    void move();
    void addPose();
    void parseArgs(const float* argv) override;
    void compute() override ;
  };


    
}
