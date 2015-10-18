#include "hand/marker_listener.h"

namespace hm{

Point_cloud_listener::Point_cloud_listener(ros::NodeHandle& node,const std::string topic_name){

     marker_subscriber = node.subscribe(topic_name, 1, &Point_cloud_listener::callback,this);

     R.setRPY(M_PI,0,0);
     R2.setEulerYPR(0,M_PI,0);
}


void Point_cloud_listener::callback(const visualization_msgs::MarkerConstPtr& marray){

    points.resize(marray->points.size(),3);
    avec3 dir;
    for(i = 0; i < marray->points.size();i++){
        vec.setX(marray->points[i].x);
        vec.setY(marray->points[i].y);
        vec.setZ(marray->points[i].z);

        tf.setIdentity();
        tf.setOrigin(vec);

        //vec2.setZero();
        //dir = vec2 - vec;

       // vec = vec + dir;



        vec =  R * vec;
        vec.setZ(-vec.z());

        //tf.setIdentity();
        //tf.setOrigin(vec);



        points(i,0) =  vec.getX();
        points(i,1) =  vec.getY();
        points(i,2) =  vec.getZ();
    }
}


Point_cloud_broadcaster::Point_cloud_broadcaster(ros::NodeHandle& node, const std::string topic_name){
    point_cloud_publisher = node.advertise<visualization_msgs::Marker>(topic_name, 10);

    mCloudPoints.header.frame_id  = "world";
    mCloudPoints.type             =  visualization_msgs::Marker::SPHERE_LIST;


    mCloudPoints.color.a          = 1.0;
    mCloudPoints.color.g          = 1.0;
    mCloudPoints.scale.x          = 0.02;
    mCloudPoints.scale.y          = 0.02;
    mCloudPoints.scale.z          = 0.02;

}

void Point_cloud_broadcaster::update(const arma::mat& points){
    mCloudPoints.points.resize(points.n_rows);
    for(std::size_t i = 0; i < points.n_rows;i++){
        mCloudPoints.points[i].x = points(i,0);
        mCloudPoints.points[i].y = points(i,1);
        mCloudPoints.points[i].z = points(i,2);
       // mCloudPoints.id = i;
       // mCloudPoints.ns = "point_cloud";
    }

    point_cloud_publisher.publish(mCloudPoints);
}



}
