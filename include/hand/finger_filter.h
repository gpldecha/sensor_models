#ifndef FILTER_HAND_MODEL_
#define FILTER_HAND_MODEL_

// ROS

#include <tf/LinearMath/Quaternion.h>

// STL

#include <vector>

// Hand

#include "hand/types.h"


// Statistics

#include <statistics/distributions/gmm.h>


namespace hm{


template<typename T> class Emvg{

public:

    Emvg(float alpha=0.1):alpha(alpha){
        bFirst=true;
    }

    T& update(T& x){
        if(bFirst){
            s = x;
            return x;
            bFirst=false;
        }else{
            s = alpha * x + (1.0 - alpha) * s;
            return s;
        }
    }

private:

    float alpha;
    bool bFirst;
    T s;

};

class Finger_filter{


public:

    Finger_filter();

    void update(const arma::mat& points,std::array<avec3,NUM_FINGERS>& finger_posistions);


private:

    void get_responsibility_factor(const arma::mat& points,const std::array<avec3,NUM_FINGERS>& finger_posistions);

    /*inline float square_distance_origin(const vec3& A) const{
        return (A.x - P(0)) * (A.x - P(0)) +
               (A.y - P(1)) * (A.y - P(1)) +
               (A.z - P(2)) * (A.z - P(2));
    }*/

    inline float square_distance(const vec3& A,const vec3& B) const{
        return (A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y) + (A.z - B.z) * (A.z - B.z);
    }

    inline float normal(float x, float mu=0.0,float std=1.0) const {
        return 1.0/(std * 2 * M_PI) * exp(-(x - mu)*(x-mu)/(2*std*std));
    }

private:


    std::array<avec3,NUM_FINGERS>                 fp, h_fp, dh_fp;

    std::array<arma::vec,NUM_FINGERS>             r; // each bounded point has a responsibility to belong to a finger

    Emvg<avec3>                                   emvg;

    std::array<GMM,NUM_FINGERS>                   gmm;



};

}



#endif
