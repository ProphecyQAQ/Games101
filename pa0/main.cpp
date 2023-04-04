#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>
using namespace std;

int main(){

    Eigen::Vector3f p(2.f, 1.f, 1.f);
    float pi = std::acos(-1)/4;
    Eigen::Matrix3f trans;
    trans << cos(pi), -sin(pi), 1,
             sin(pi), cos(pi), 2,
             0, 0, 1;

    cout << trans * p << endl;
        
    return 0;
}