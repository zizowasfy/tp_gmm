import numpy as np

def computeResultingGaussians(model, pp):
    from prodResClass import prodRes
    prodResList = []
    reg_factor = 1e-8

    for t in range(0, np.shape(pp)[1]):
        for i in range(0, model.nbStates):
            for m in range(0, model.nbFrames):
                # Transform mean to global frame: Mu_global = A @ Mu_local + b
                pp[m, t].Mu[:, i] = np.reshape(
                    np.dot(pp[m, t].A, np.reshape(model.ref[m].ZMu[:, i], (model.nbVar, 1))) + pp[m, t].b,
                    (model.nbVar,)
                )
                # Transform covariance to global frame: Sigma_global = A @ Sigma_local @ A^T
                a = np.dot(pp[m, t].A, np.reshape(model.ref[m].ZSigma[:, :, i], (model.nbVar, model.nbVar)))
                sigma_global = np.dot(a, pp[m, t].A.T) + np.identity(model.nbVar) * reg_factor
                # Guarantee numerical symmetry
                pp[m, t].Sigma[:, :, i] = 0.5 * (sigma_global + sigma_global.T)

    for t in range(0, np.shape(pp)[1]):
        prodResList.append(prodRes(model.nbVar))
        for i in range(0, model.nbStates):
            SigmaTmp = np.zeros((model.nbVar, model.nbVar))
            MuTmp = np.zeros((model.nbVar, 1))
            for m in range(0, model.nbFrames):
                inv_cov = np.linalg.inv(pp[m, t].Sigma[:, :, i])
                SigmaTmp = SigmaTmp + inv_cov
                MuTmp = MuTmp + np.dot(inv_cov, np.reshape(pp[m, t].Mu[:, i], (model.nbVar, 1)))

            # Guarantee symmetry of precision and covariance
            SigmaTmp = 0.5 * (SigmaTmp + SigmaTmp.T)
            prodResList[t].invSigma = np.dstack((prodResList[t].invSigma, SigmaTmp))

            cov_prod = np.linalg.inv(SigmaTmp)
            cov_prod = 0.5 * (cov_prod + cov_prod.T)
            prodResList[t].Sigma = np.dstack((prodResList[t].Sigma, cov_prod))

            det_val = max(float(np.linalg.det(cov_prod)), 1e-12)
            prodResList[t].detSigma = np.hstack((prodResList[t].detSigma, det_val))
            prodResList[t].Mu = np.hstack((prodResList[t].Mu, np.dot(cov_prod, MuTmp)))

    return prodResList, pp