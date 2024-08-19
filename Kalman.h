//
// Created by yunze on 2024/8/19.
//

#ifndef KALMAN_FILTER_KALMAN_H
#define KALMAN_FILTER_KALMAN_H

#include "Eigen/Dense"

class Kalman {
private:
    int stateSize;
    int measureSize;
    int controlSiz;
    Eigen::VectorXd x;
    Eigen::VectorXd z;
    Eigen::VectorXd u;

    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd P;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;
    Eigen::MatrixXd Q;

public:
    Kalman(int stateSize_, int measureSize_, int controlSize_);

    ~Kalman() {}

    void init(Eigen::VectorXd &x_, Eigen::MatrixXd &P_, Eigen::MatrixXd &R_, Eigen::MatrixXd &Q_);

    Eigen::VectorXd predict(Eigen::MatrixXd &A_);

    Eigen::VectorXd predict(Eigen::MatrixXd &A_, Eigen::MatrixXd &B_, Eigen::VectorXd &u_);

    void update(Eigen::MatrixXd &H_, Eigen::VectorXd z_meas);
};

#endif //KALMAN_FILTER_KALMAN_H
