#include "hand/finger_filter.h"

namespace hm{

Finger_filter::Finger_filter()
{

    // Load a gmm for each finger. The Gmm represents the workspace (rechable area) of the extent of a finger

    std::string path_to_load_parameters = "/home/guillaume/MatlabWorkSpace/joint_blabeling/parameters/";
    std::string path_to_gmm;
    FINGERS f;
    for(int i = INDEX; i != Last;i++){
        f = static_cast<FINGERS>(i);
        path_to_gmm = path_to_load_parameters + f_enum2str.at(f) + "/";
        gmm[f] = GMM(path_to_gmm,f_enum2str.at(f) );
    }

}


void Finger_filter::update(const arma::mat& points,std::array<avec3, NUM_FINGERS> &finger_posistions){


    // wigh the points
    get_responsibility_factor(points,finger_posistions);


    r[0].st().print("r_f1");
    r[1].st().print("r_f2");
    r[2].st().print("r_f3");
    r[3].st().print("r_f4");
    r[4].st().print("r_f5");

    FINGERS f;
    std::size_t index;
    for(int i = INDEX; i != Last;i++){
        f = static_cast<FINGERS>(i);
        index = arma::max(r[f]);
        finger_posistions[f] = points[index];
    }

}


/*void Finger_filter::get_points(){
    apoints.set_size(points.size()+1,3);
    for(std::size_t p = 0; p < points.size();p++){
            apoints(p,0) = points[p].x;
            apoints(p,1) = points[p].y;
            apoints(p,2) = points[p].z;
    }
}*/

void Finger_filter::get_responsibility_factor(const arma::mat& points,const std::array<avec3,NUM_FINGERS>& finger_posistions){
    FINGERS f;
    for(int i = INDEX; i != Last;i++){
        f = static_cast<FINGERS>(i);
        r[f].resize(points.n_rows+1);
        gmm[f].likelihood(points,r[f]);
    }

    double c;
    for(int i = INDEX; i != Last;i++){
        f = static_cast<FINGERS>(i);
        //c = gmm[f].likelihood(finger_posistions[f]);
        r[f] = r[f]/c;
        r[f] = arma::normalise(r[f]);
    }



}


}
