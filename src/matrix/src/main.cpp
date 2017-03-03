#include "matrixNode.hpp"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "MatrixNode", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);	
    
	MatrixNode matrixNode;

    ros::Duration d(1.0);
    while (ros::ok()){
        d.sleep();
    }

    return 0;
}
