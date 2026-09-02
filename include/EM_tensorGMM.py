import numpy as np
import sys
from gaussPDFfast import gaussPDFfast

def EM_tensorGMM(s, model):
    nbMinSteps = 5
    nbMaxSteps = 100
    maxDiffLL = 1e-4
    realmin = sys.float_info[3]
    diagRegularizationFactor = 1e-5

    nbSamples = len(s)
    nbDataTotal = sum(currSample.nbData for currSample in s)
    
    # tensorData shape: (nbVar, nbFrames, nbDataTotal)
    tensorData = np.zeros((model.nbVar, model.nbFrames, nbDataTotal))

    # Project demonstrations into each frame m: invA @ (Data - b)
    DataIndStart = 0
    for currSample in s:
        DataIndEnd = DataIndStart + currSample.nbData
        for m in range(model.nbFrames):
            invA = currSample.p[m, 0].invA
            b = currSample.p[m, 0].b
            diff = currSample.Data - b
            tensorData[:, m, DataIndStart:DataIndEnd] = np.dot(invA, diff)
        DataIndStart = DataIndEnd

    LL = []
    Mu = np.zeros((model.nbVar, model.nbFrames, model.nbStates))
    Sigma = np.zeros((model.nbVar, model.nbVar, model.nbFrames, model.nbStates))

    for nbIter in range(nbMaxSteps):
        # E-step: compute responsibilities
        Lik, GAMMA, GAMMA0 = computeGamma(tensorData, model)
        
        # Responsibilities normalized per state across all data points
        gamma_sum = np.sum(GAMMA, axis=1, keepdims=True)
        GAMMA2 = GAMMA / (gamma_sum + 1e-8)
        
        # M-step: update Priors, Mu, Sigma for each state and frame
        for i in range(model.nbStates):
            model.Priors[i] = float(np.sum(GAMMA[i, :]) / nbDataTotal)
            for m in range(model.nbFrames):
                DataMat = tensorData[:, m, :]  # (nbVar, nbDataTotal)
                
                # Update Mean in frame m
                Mu[:, m, i] = np.dot(DataMat, GAMMA2[i, :])
                
                # Update Covariance in frame m (vectorized, avoiding large np.diag allocation)
                DataTmp = DataMat - Mu[:, m, i:i+1]
                weighted_diff = DataTmp * GAMMA2[i, :]
                Sigma[:, :, m, i] = np.dot(weighted_diff, DataTmp.T) + np.eye(model.nbVar) * diagRegularizationFactor
        
        # Normalize priors to ensure valid probability distribution
        prior_sum = sum(model.Priors)
        if prior_sum > 0:
            model.Priors = [p / prior_sum for p in model.Priors]
            
        # Update model parameters for next iteration
        for m in range(model.nbFrames):
            model.ref[m].ZMu = Mu[:, m, :].copy()
            model.ref[m].ZSigma = Sigma[:, :, m, :].copy()

        # Compute average log-likelihood per data point
        avg_ll = float(np.sum(np.log(np.sum(Lik, axis=0) + realmin)) / nbDataTotal)
        LL.append(avg_ll)

        if nbIter > nbMinSteps:
            if abs(LL[nbIter] - LL[nbIter-1]) < maxDiffLL or nbIter == nbMaxSteps - 1:
                print(f"EM converged after {nbIter} iterations (LL: {LL[nbIter]:.4f})")
                return model

    print(f"The maximum number of {nbMaxSteps} EM iterations has been reached")
    return model

def computeGamma(Data, model):
    import sys
    from gaussPDFfast import gaussPDFfast
    realmin = sys.float_info[3]
    nbData = np.shape(Data)[2]
    Lik = np.ones((model.nbStates, nbData))
    GAMMA0 = np.zeros((model.nbStates, model.nbFrames, nbData))

    for i in range(model.nbStates):
        for m in range(model.nbFrames):
            DataMat = Data[:, m, :]
            cov = model.ref[m].ZSigma[:, :, i]
            # Ensure symmetry and positive-definiteness for numerical stability
            cov = 0.5 * (cov + cov.T)
            det = np.linalg.det(cov)
            if det <= 0:
                cov = cov + np.eye(model.nbVar) * 1e-4
                det = np.linalg.det(cov)
            inv_cov = np.linalg.inv(cov)

            GAMMA0[i, m, :] = gaussPDFfast(DataMat, model.ref[m].ZMu[:, i], inv_cov, det)
            Lik[i, :] = Lik[i, :] * np.squeeze(GAMMA0[i, m, :])
        Lik[i, :] = Lik[i, :] * model.Priors[i]

    sumLik = np.sum(Lik, axis=0, keepdims=True) + realmin
    GAMMA = Lik / sumLik
    return Lik, GAMMA, GAMMA0