#ifndef HAND_MODEL_TYPES
#define HAND_MODEL_TYPES

#include <armadillo>
#include <map>

#include <tf/LinearMath/Vector3.h>

namespace hm {

struct vec3{

    vec3(){}
    vec3(float x,float y, float z):x(x),y(y),z(z){}

    float x, y, z;
};

struct joint_info{
    std::string name;
    float       value;
};


typedef arma::colvec::fixed<3> avec3;
typedef arma::mat::fixed<3,3>  mat3;


typedef enum{INDEX=0,MIDDLE=1,RING=2,PINKY=3,THUMB=4,Last=5} FINGERS;

enum{NUM_FINGERS=5};

const static std::map<std::string,FINGERS> f_str2enum =  {
                                             {"index",  INDEX},
                                             {"middle", MIDDLE},
                                             {"ring",   RING},
                                             {"pinky",  PINKY},
                                             {"thumb",  THUMB}
                                            };
const static std::map<FINGERS,std::string> f_enum2str =  {
                                             {INDEX,"index"},
                                             {MIDDLE,"middle"},
                                             {RING,"ring"},
                                             {PINKY,"pinky"},
                                             {THUMB,"thumb"}
                                            };

//const static std::array<std::size_t,NUM_FINGERS>   j_index_begin = {{0,6,10,14,18}};
//const static std::array<std::size_t,NUM_FINGERS>   j_index_end   = {{5,9,13,17,21}};






class Print{
public:

    static void print(const std::vector<vec3>&  points){
        for(std::size_t i = 0; i < points.size();i++){
            std::cout<< points[i].x << "\t" << points[i].y << "\t" << points[i].z << std::endl;
        }
    }

    static void print(const tf::Vector3& vec){
            std::cout<< vec.x() << "\t" << vec.y() << "\t" << vec.z() << std::endl;
    }
};

}

#endif
