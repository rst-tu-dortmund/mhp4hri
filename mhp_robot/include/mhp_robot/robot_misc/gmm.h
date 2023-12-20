// Training of a task-parameterized Gaussian mixture model (GMM) based on candidate frames of reference.
// The proposed task-parameterized GMM approach relies on the linear transformation and product properties of
// Gaussian distributions to derive an expectation-maximization (EM) algorithm to train the model.
// The proposed approach is contrasted with an implementation of the approach proposed by Wilson and Bobick in
// 1999, with an implementation applied to GMM (that we will call PGMM) and following the model described in
// "Parametric Hidden Markov Models for Gesture Recognition", IEEE Trans. on Pattern Analysis and Machine
// Intelligence.
// In contrast to the standard PGMM approach, the new approach that we propose allows the parameterization of
// both the centers and covariance matrices of the Gaussians. It has been designed for targeting problems in
// which the task parameters can be represented in the form of coordinate systems, which is for example the
// case in robot manipulation problems.
//
// Authors:	Tohid Alizadeh and Sylvain Calinon, 2012
//         	http://programming-by-demonstration.org/
//
// This source code is given for free! In exchange, we would be grateful if you cite
// the following reference in any academic publication that uses this code or part of it:
//
// @inproceedings{Calinon12Hum,
//   author="Calinon, S. and Li, Z. and Alizadeh, T. and Tsagarakis, N. G. and Caldwell, D. G.",
//   title="Statistical dynamical systems for skills acquisition in humanoids",
//   booktitle="Proc. {IEEE} Intl Conf. on Humanoid Robots ({H}umanoids)",
//   year="2012",
//   address="Osaka, Japan"
// }
//
// Modification: Heiko Renz, 2023

#ifndef GMM_H
#define GMM_H

#include <ros/ros.h>
#include <string.h>
#include <sys/time.h>

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>

#ifdef DKM
#include <dkm.hpp>
#endif  // DKM

#define REALMIN 2.2251e-200
#define REALMAX 1.7977e200
using namespace std;

#ifdef ARMADILLO
#include <armadillo>

using namespace arma;

struct Model
{
    vec priors;
    mat Mu;
    vector<mat> Sigma;
};

class GMM
{
 public:
    using Ptr  = shared_ptr<GMM>;
    using UPtr = unique_ptr<GMM>;

    GMM();
    GMM(int nbStates, int nbObs, int nbDimensions, int nbExtrapolationSteps, bool weighting);

    // Move and Copy operators
    GMM(const GMM&)            = delete;
    GMM(GMM&&)                 = default;
    GMM& operator=(const GMM&) = delete;
    GMM& operator=(GMM&&)      = default;
    ~GMM()                     = default;

    void init_GMM_model();
    void update_GMM(mat* error_gt_extrap, int joint);
    void get_variance();

    // Accesible results after GMR for y and sigma
    vec _y;
    vec _sigma_y;

 private:
    int _nbStates, _nbObs, _nbDim, _nbExtrapSteps;
    mat _buffer;
    Model _model;

    // weighting based on approach from "Gaussian Mixture Estimation from Weighted Samples" (Hanebeck, Frisch)
    vec _weights;
    bool _weighting = true;

    std::vector<std::array<double, 2>> _data_vec;
    std::vector<bool> _kmeans_initialized = {
        false, false, false, false, false, false, false, false, false, false, false,
    };

    int weightedEM(mat* data);
    vec gaussPDF(mat M, vec Mu, mat Sigma);
    bool init_k_means(mat* data);
};

#else   // ARMADILLO
class GMM
{
 public:
    using Ptr  = shared_ptr<GMM>;
    using UPtr = unique_ptr<GMM>;

    GMM() { ROS_ERROR("GMM: Armadillo not available!"); };
    GMM(int nbStates, int nbObs, int nbDimensions, int nbExtrapolationSteps, bool weighting) { ROS_ERROR("GMM: Armadillo not available!"); };
};
#endif  // ARMADILLO

#endif  // GMM_H
