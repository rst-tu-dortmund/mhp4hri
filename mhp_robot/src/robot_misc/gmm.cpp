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
// Modification: Heiko Renz, 2023
#ifdef ARMADILLO
#include <mhp_robot/robot_misc/gmm.h>
using namespace arma;
using namespace std;

GMM::GMM() { GMM(4, 60, 2, 30, false); }

GMM::GMM(int nbStates, int nbObs, int nbDimensions, int nbExtrapolationSteps, bool weighting)
{
    _nbStates      = nbStates;      // Number of GMM Components
    _nbObs         = nbObs;         // Number of Observations
    _nbDim         = nbDimensions;  // Number of Dimensions for the Datasamples
    _nbExtrapSteps = nbExtrapolationSteps;
    _weighting     = weighting;
    _y.resize(nbExtrapolationSteps);
    _sigma_y.resize(nbExtrapolationSteps);

    init_GMM_model();
}

void GMM::init_GMM_model()
{
    _model.priors.resize(_nbStates);
    _model.Mu.resize(_nbDim, _nbStates);
    _model.Sigma.resize(_nbDim);
    std::fill(_model.Sigma.begin(), _model.Sigma.end(), mat(_nbDim, _nbStates, arma::fill::zeros));

    _buffer = mat(_nbObs * _nbExtrapSteps, _nbDim, fill::zeros);
}

void GMM::update_GMM(mat* error_gt_mat, int joint)
{
    if (_kmeans_initialized.at(joint))
    {
        weightedEM(error_gt_mat);
    }
    else
    {
        bool k_mean_success = init_k_means(error_gt_mat);
        if (k_mean_success)
        {
            _kmeans_initialized.at(joint) = true;
        }
    }
}

void GMM::get_variance()
{
    uword inputDim  = 0;
    uword outputDIm = 1;

    mat expData(_nbDim, 30);
    expData.row(0) = linspace<rowvec>(0.1, 3, 30);

    mat Pxi(expData.n_cols, _nbStates);
    for (uword i = 0; i < _nbStates; i++)
    {
        Pxi.col(i) = _model.priors(i) *
                     gaussPDF(conv_to<rowvec>::from(expData.row(0)), _model.Mu.row(inputDim).col(i), _model.Sigma.at(i).row(inputDim).col(inputDim));
    }
    mat beta(expData.n_cols, _nbStates);
    beta = Pxi / repmat(sum(Pxi, 1) + REALMIN, 1, _nbStates);

    cube y_tmp(1, expData.n_cols, _nbStates);

    for (uword i = 0; i < _nbStates; i++)
    {
        y_tmp.slice(i) = repmat(_model.Mu.row(outputDIm).col(i), 1, expData.n_cols) +
                         _model.Sigma.at(i).row(outputDIm).col(inputDim) * inv(_model.Sigma.at(i).row(inputDim).col(inputDim)) *
                             (expData.row(0) - repmat(_model.Mu.row(inputDim).col(i), 1, expData.n_cols));
    }
    cube beta_tmp(beta.memptr(), 1, expData.n_cols, _nbStates);

    rowvec y;
    y  = sum(beta_tmp % y_tmp, 2);
    _y = conv_to<vec>::from(y);

    rowvec sigma_y_tmp(_nbStates, fill::zeros);
    for (uword i = 0; i < _nbStates; i++)
    {
        sigma_y_tmp(i) = conv_to<double>::from(_model.Sigma.at(i).row(outputDIm).col(outputDIm)) -
                         (conv_to<double>::from(_model.Sigma.at(i).row(outputDIm).col(inputDim)) *
                          conv_to<double>::from(inv(_model.Sigma.at(i).row(inputDim).col(inputDim))) *
                          conv_to<double>::from(_model.Sigma.at(i).row(inputDim).col(outputDIm)));
    }
    vec sigma_y(expData.n_cols);
    _sigma_y = sum((beta % beta) % repmat(sigma_y_tmp, expData.n_cols, 1), 1);
}

int GMM::weightedEM(mat* data)
{
    if (_nbStates < 1 || _nbObs < 1) return -1;
    // Thresholds for the EM iterations
    int nbMaxSteps                 = 100;   // maximum # of iterations
    int nbMinSteps                 = 10;    // minimum # of iterations
    float diagRegularizationFactor = 1E-5;  // Avoiding numerical instability in Sigma calculations

    double loglik = 0, loglik_old = -REALMAX;
    double loglik_threshold = 1E-7;
    // Data preparation
    mat input(2, data->size(), fill::zeros);
    input.row(0) = repmat(linspace<rowvec>(0.1, 3, 30), 1, _nbObs);  // time vector
    input.row(1) = trans(*data);                                     // errors between gt and extrap
    // Fill Up weights vector
    if (_weights.is_empty() && _weighting)
    {
        _weights = trans(repmat(linspace<rowvec>(3, 0.1, 30), 1, _nbObs) / 3);  // weights are inverse to extrapolation time
    }
    int nbVar  = _nbDim;        // time and error --> 2 dimensions
    int nbData = input.n_cols;  // length of error samples--> from _nbObs * _nbExtrapolationSteps (for 60 observaations and 30 steps 1800)

    // Initilaize start GMM parameters
    mat Mu                 = _model.Mu;                             // Mean Values/Centroids of GMM components
    std::vector<mat> Sigma = _model.Sigma;                          // Covariance matrices for each GMM component
    rowvec priors          = conv_to<rowvec>::from(_model.priors);  // Priors or probabilities of each GMM Component
    mat Pxi(nbData, _nbStates, fill::zeros);
    mat Pix_tmp(nbData, _nbStates, fill::zeros);
    mat Pix(nbData, _nbStates, fill::zeros);
    rowvec E = zeros<rowvec>(_nbStates);

    for (int nbIter = 0; nbIter < nbMaxSteps; nbIter++)
    {
        // ------------ E step -----------------
        for (int i = 0; i < _nbStates; i++)
        {
            Pxi.col(i) = gaussPDF(input, Mu.col(i), Sigma.at(i));  // probability p(x|i)
        }
        if (Pxi.has_nan())
        {
            init_k_means(data);
            Mu     = _model.Mu;                             // Mean Values/Centroids of GMM components
            Sigma  = _model.Sigma;                          // Covariance matrices for each GMM component
            priors = conv_to<rowvec>::from(_model.priors);  // Priors or probabilities of each GMM Component
            for (int i = 0; i < _nbStates; i++)
            {
                Pxi.col(i) = gaussPDF(input, Mu.col(i), Sigma.at(i));  // probability p(x|i)
            }
        }
        Pix_tmp = repmat(priors, nbData, 1) % Pxi;

        Pix = Pix_tmp / repmat(sum(Pix_tmp, 1), 1, _nbStates);  // Posterior Probability p(i|x)

        if (_weighting)
        {
            Pix = Pix.each_col() % _weights;
        }

        // Cumulated posterior probability
        E = sum(Pix, 0);
        // ----------- M step -------------
        for (int i = 0; i < _nbStates; i++)
        {
            // Update priors
            priors(i) = E(i) / nbData;

            // Update Mu
            Mu.col(i) = (input * Pix.col(i)) / E(i);
            Mu(1, i)  = 0;

            // Update Sigma
            Sigma.at(i) =
                (repmat(Pix.col(i).t(), nbVar, 1) % (input - repmat(Mu.col(i), 1, nbData)) * (input - repmat(Mu.col(i), 1, nbData)).t()) / E(i);

            Sigma.at(i) = Sigma.at(i) + diagRegularizationFactor * mat(nbVar, nbVar, fill::eye);
        }

        // Checkout for stopping based on loglikelihood
        for (int i = 0; i < _nbStates; i++)
        {
            Pxi.col(i) = gaussPDF(input, Mu.col(i), Sigma.at(i));  // probability p(x|i)
        }
        vec F = Pxi * trans(priors);
        F.replace(0, REALMIN);
        loglik = sum(log(F));

        if (abs((loglik / loglik_old) - 1) < loglik_threshold && nbIter >= nbMinSteps)
        {
            break;
        }
        else
        {
            loglik_old = loglik;
        }
    }
    // Assign Values into model struct
    _model.Mu     = Mu;
    _model.priors = conv_to<vec>::from(priors);
    _model.Sigma  = Sigma;
    return 1;
};

vec GMM::gaussPDF(mat M, vec Mu, mat Sigma)
{
    mat D_Tmp = trans(M) - repmat(trans(Mu), M.n_cols, 1);
    mat invTmp;
    if (inv(invTmp, Sigma))
    {
        vec Probs = sum(D_Tmp % ((D_Tmp) * (invTmp)), 1);
        Probs     = exp(-0.5 * Probs) / (sqrt(pow((2 * M_PI), Sigma.n_cols) * (abs(det(Sigma)) + REALMIN)));
        if (any(Probs == 0))
        {
            Probs.replace(0, REALMIN);
        }
        return Probs;
    }
    else
    {
        vec Probs(M.n_cols);
        Probs = REALMIN;
        return Probs;
    }
};

bool GMM::init_k_means(mat* data)
{
    mat means;

    mat input(2, data->size(), fill::zeros);

    input.row(0) = repmat(linspace<rowvec>(0.1, 3, 30), 1, _nbObs);
    input.row(1) = trans(*data);
    _data_vec.resize(input.n_cols);
    for (int var = 0; var < input.n_cols; ++var)
    {
        _data_vec.at(var) = {input(0, var), input(1, var)};
    }

#ifdef DKM
    std::tuple cluster_data = dkm::kmeans_lloyd(_data_vec, _nbStates);
    mat centroids(_nbDim, _nbStates);
    int counter = 0;
    for (const auto& mean : std::get<0>(cluster_data))
    {
        centroids(0, counter) = mean[0];
        centroids(1, counter) = mean[1];
        counter++;
    }
    std::vector labels = std::get<1>(cluster_data);
    uvec labels2       = conv_to<uvec>::from(labels);
#else   // DKM
    ROS_WARN_ONCE("DKM not found, initializing GMM with centorids and lables as zeros. This can lead to bad estimation results!");
    mat centroids(_nbDim, _nbStates, arma::fill::zeros);
    uvec labels2(input.n_cols, arma::fill::zeros);
#endif  // DKM
    _model.Sigma.resize(_nbStates);
    _model.priors.resize(_nbStates);
    _model.Mu.resize(_nbDim, _nbStates);
    for (uword i = 0; i < _nbStates; ++i)
    {
        centroids(1, i)  = 0;
        _model.Mu        = centroids;
        uvec idxs        = find(labels2 == i);
        _model.priors(i) = idxs.size() / double(data->size());
        mat tmp          = join_rows(input.cols(idxs), input.cols(idxs)).t();
        mat xy           = input.cols(idxs);
        mat addSmallIdentity(_nbDim, _nbDim, fill::eye);
        _model.Sigma.at(i) = cov(tmp) + 0.00001 * addSmallIdentity;
    }

    return true;
}

#endif  // ARMADILLO