#include <Eigen/Dense>
#include <iostream>

#define MAX_TIMESTAMPS_INITIALIZATION 100
#define SIGMA_Z 1e-3
#define NOISE 1e-4


class TimestampFilter{
    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        enum Status {
            Error        = 0x0,
            Initializing = 0x1, 
            Ready        = 0x2  
        };

        TimestampFilter(){
            _A << 1, 1, 0, 1;
            _C << 1, 0;
            reset();
        }

        inline void reset(){
            _mu = Eigen::Vector2d::Zero();
            _sigma = Eigen::Matrix2d::Identity();
            _filter_stamp = 0.0;
            _prev_stamp = 0.0;
            _delta_sum = 0.0;
            _counter_init_stamps = 0; // reset from outside
            _status = Status::Error;
        }

    
        inline void setMeasurement(const double& current_stamp_){
            _initialize(current_stamp_);
            if(_status == Status::Ready){
                _filter_stamp = _filterStamp(current_stamp_);
            }
        }

        inline const double stamp() const{
            return _filter_stamp;
        }

        inline const Status status() const{
            return _status;
        }

    
    protected:
        Eigen::Vector2d _mu = Eigen::Vector2d::Zero();
        Eigen::Matrix2d _sigma = Eigen::Matrix2d::Identity();
        Eigen::Matrix2d _A = Eigen::Matrix2d::Identity();
        Eigen::Vector2d _C = Eigen::Vector2d::Zero();  
        double _current_stamp = 0.0;
        double _filter_stamp = 0.0;
        double _prev_stamp = 0.0;
        double _delta_sum = 0.0;
        int _counter_init_stamps = 0; 
        Status _status = Status::Error;

        inline double _filterStamp(const double& current_stamp_){
            assert(_mu(0) > 0);
            assert(_mu(1) > 0);
            _predictKF(_mu, _sigma);
            _updateKF(_mu, _sigma, current_stamp_);
            return _mu(0);
        }


        inline void _initialize(const double& current_stamp_){
            // ldg if we wrote on inital state we ready fuckers
            if(_mu(0) > 0 && _mu(1) > 0){
                _status = Status::Ready;
                return;
            }
            
            if(_counter_init_stamps){
                _delta_sum += current_stamp_-_prev_stamp;
            }
        
            _counter_init_stamps++;
            _prev_stamp = current_stamp_;
            
            // ldg if we reach max value we're still not ready, 
            // ldg need to wait for ldg next timestamp
            if(_counter_init_stamps >= MAX_TIMESTAMPS_INITIALIZATION){
                _mu(0) = current_stamp_;
                _mu(1) = _delta_sum / (_counter_init_stamps - 1);
            }
            
            _status = Status::Initializing;
            return; 
        }


        
        inline void _predictKF(Eigen::Vector2d& mu_, Eigen::Matrix2d& sigma_){
            // ldg adding delta to curr timestamp
            mu_(0) += mu_(1);
            sigma_ = _A*sigma_*_A.transpose() + Eigen::Matrix2d::Identity()*NOISE;
        }
        

        inline void _updateKF(Eigen::Vector2d& mu_, Eigen::Matrix2d& sigma_, const double current_stamp_){
            const Eigen::Vector2d K = sigma_* _C * (1. / (_C.transpose()*sigma_*_C + SIGMA_Z));
            const double err = current_stamp_ - mu_(0);
            // std::cerr << "err: " << std::abs(err) << std::endl;
            // ldg check if we need to correct
            if(std::abs(err) > (3*mu_(1))){
                return;
            }
            const Eigen::Vector2d correction = K*err;
            mu_ += correction;
            sigma_ = (Eigen::Matrix2d::Identity() - K*_C.transpose())*sigma_;            
        }
        
};

