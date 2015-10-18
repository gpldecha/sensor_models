#ifndef POINT_MODEL_H_
#define POINT_MODEL_H_

#include "mrff/likelihood/likelihood.h"

class Point_likelihood: public mrff::BaseLikelihood2{

public:

    Point_likelihood();

    float likelihood();

private:


};

///
/// \brief The Point_model class
///   Represents a point mass range sensor, it is represented by an
///   3D cartesian origin

class Point_model{

public:





};

#endif
