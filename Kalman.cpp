//
// Created by yunze on 2024/8/19.
//

#include "Kalman.h"
#include "iostream"

using namespace std;

Kalman::Kalman(int stateSize_, int measureSize_, int controlSize_) : stateSize(stateSize_), measureSize(measureSize_),
                                                                     controlSiz(controlSize_) {
    if (stateSize == 0 || measureSize == 0) {
        cout << "error" << endl;
    }
    x.resize(stateSize);
    x.setZero();

    z.resize(measureSize);
    z.setZero();

    u.resize(controlSiz);
    u.transpose();
    u.setZero();

    A.resize(stateSize, stateSize);
    A.setIdentity();

    B.resize(stateSize, controlSiz);
    B.setZero();

    P.resize(stateSize, stateSize);
    P.setIdentity();

    H.resize(measureSize, stateSize);
    H.setZero();

    Q.resize(stateSize, stateSize);
    Q.setZero();

    R.resize(measureSize, measureSize);
    R.setZero();
}

void Kalman::init(Eigen::VectorXd &x_, Eigen::MatrixXd &P_, Eigen::MatrixXd &R_, Eigen::MatrixXd &Q_) {
    x = x_;
    P = P_;
    R = R_;
    Q = Q_;
}

Eigen::VectorXd Kalman::predict(Eigen::MatrixXd &A_, Eigen::MatrixXd &B_, Eigen::VectorXd &u_) {
    A = A_;
    B = B_;
    u = u_;
    x = A * x + B * u;
    Eigen::MatrixXd A_T = A.transpose();
    P = A * P * A_T + Q;
    return x;
}

void Kalman::update(Eigen::MatrixXd &H_, Eigen::VectorXd z_meas) {
    H = H_;
    Eigen::MatrixXd temp1, temp2, Ht;
    Ht = H.transpose();
    temp1 = H * P * Ht + R;
    temp2 = temp1.inverse();
    Eigen::MatrixXd K = P * Ht * temp2;
    z = H * x;
    x = x + K * (z_meas - z);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateSize, stateSize);
    P = (I - K * H) * P;

}