#pragma once
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <memory>
#include <srrg_config/configurable.h>

namespace srrg2_solver {
  using namespace srrg2_core;
  
  using Isometry3fVector = std::vector <Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> >;
  using Isometry3f = Eigen::Isometry3f;
  using Matrix3f   = Eigen::Matrix3f;
  using Vector3f   = Eigen::Vector3f;

  
  struct TrajectoryGeneratorBase: public Configurable {
    virtual void compute()=0;
    virtual void reset() {poses.clear();}
    virtual ~TrajectoryGeneratorBase(){};
    inline Isometry3fVector& path() {return poses;}
  protected:
    Isometry3fVector poses;
  };

  using TrajectoryGeneratorPtr = std::shared_ptr<TrajectoryGeneratorBase>;

  struct TrajectoryGeneratorSphere : public TrajectoryGeneratorBase{
    void compute() override;
    PARAM(PropertyFloat,
          radius,
          "radius of the thing",
          10.f,
          0);
    PARAM(PropertyInt,
          azimuth_steps,
          "steps along the horizontal axis",
          20,
          0);
    PARAM(PropertyInt,
          elevation_steps,
          "steps along the vertical axis",
          20,
          0);
  };

  struct TrajectoryGeneratorSphereUniform : public TrajectoryGeneratorBase{
    void compute() override;
    PARAM(PropertyFloat,
          radius,
          "radius of the thing",
          10.f,
          0);
    PARAM(PropertyInt,
          steps,
          "steps of the trajectory along the horizon (max)",
          20,
          0);
  };

  struct TrajectoryGeneratorTorus : public TrajectoryGeneratorBase{
    void compute();
    PARAM(PropertyFloat,
          outer_radius,
          "radius of ring",
          10.f,
          0);
    PARAM(PropertyInt,
          outer_steps,
          "steps of the trajectory along the ring",
          20,
          0);
    PARAM(PropertyFloat,
          inner_radius,
          "radius of ring",
          10.f,
          0);
    PARAM(PropertyInt,
          inner_steps,
          "steps of the trajectory along the ring",
          20,
          0);
    PARAM(PropertyInt,
          laps,
          "how many rounds on the torus",
          1,
          0);
    int laps=1;
  };

  struct TrajectoryGeneratorRandomManhattan : public TrajectoryGeneratorBase{
    PARAM(PropertyInt,
          grid_size,
          "number of 'squares' along one dimension",
          30,
          0);
    PARAM(PropertyInt,
          steps,
          "how long keep simulating",
          1000,
          0);
    PARAM(PropertyInt,
          step_size,
          "how many measurements along the edge of a square",
          5,
          0);
    PARAM(PropertyFloat,
          prob_turn,
          "how likely to turn?",
          0.5f,
          0);

    
    // true if agent can move forward, if true, it performs a motion
    bool canMove();
    void move();
    void addPose();
    void compute() override ;
    // status
    int r,  c; // cell coordinates
    int heading; // 0-3 right, top, left, bottom

  };


    
}
