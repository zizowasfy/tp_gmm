import numpy as np

def init_proposedPGMM_timeBased(s, modelcur):
    from refClass import ref
    from modelClass import model
    diagRegularizationFactor = 0.03

    nbSamples = len(s)
    
    # Vectorized transformation of all demonstration points into each frame
    frame_data_list = []
    for i in range(modelcur.nbFrames):
        chunks = []
        for j in range(nbSamples):
            # Transform global data to local frame i: invA @ (Data - b)
            invA = s[j].p[i, 0].invA
            b = s[j].p[i, 0].b
            chunks.append(np.dot(invA, s[j].Data - b))
        frame_data_list.append(np.hstack(chunks))
    DataAll = np.vstack(frame_data_list)

    total_points = DataAll.shape[1]
    TimingSep = np.linspace(np.amin(DataAll[0, :]), np.amax(DataAll[0, :]), num=modelcur.nbStates + 1)
    
    Priors = []
    dim_total = DataAll.shape[0]
    Mu = np.zeros((dim_total, modelcur.nbStates))
    Sigma = np.zeros((dim_total, dim_total, modelcur.nbStates))

    for i in range(modelcur.nbStates):
        # On the last state, include the right boundary (<=) so goal points are not dropped
        if i == modelcur.nbStates - 1:
            idtmp = np.intersect1d(
                np.nonzero(DataAll[0, :] >= TimingSep[i]),
                np.nonzero(DataAll[0, :] <= TimingSep[i + 1])
            )
        else:
            idtmp = np.intersect1d(
                np.nonzero(DataAll[0, :] >= TimingSep[i]),
                np.nonzero(DataAll[0, :] < TimingSep[i + 1])
            )

        n_pts = len(idtmp)
        Priors.append(n_pts)

        if n_pts > 0:
            muData = DataAll[:, idtmp]
            Mu[:, i] = np.mean(muData, axis=1)
            if n_pts > 1:
                cov_mat = np.cov(muData)
            else:
                cov_mat = np.zeros((dim_total, dim_total))
            cov_mat = cov_mat + np.identity(dim_total) * diagRegularizationFactor
            Sigma[:, :, i] = 0.5 * (cov_mat + cov_mat.T)
        else:
            Mu[:, i] = 0.0
            Sigma[:, :, i] = np.identity(dim_total) * diagRegularizationFactor

    prior_sum = sum(Priors)
    if prior_sum > 0:
        Priors = [float(x) / prior_sum for x in Priors]
    else:
        Priors = [1.0 / modelcur.nbStates] * modelcur.nbStates

    reflist = []
    for i in range(modelcur.nbFrames):
        v_start = i * modelcur.nbVar
        v_end = (i + 1) * modelcur.nbVar
        ZMuTmp = Mu[v_start:v_end, :].copy()
        ZSigmaTmp = Sigma[v_start:v_end, v_start:v_end, :].copy()
        # Add small ridge for positive definiteness
        ZSigmaTmp = ZSigmaTmp + np.tile(
            np.identity(modelcur.nbVar)[:, :, np.newaxis] * 1e-6,
            (1, 1, modelcur.nbStates)
        )
        reflist.append(ref(ZMuTmp, ZSigmaTmp))

    return model(modelcur.nbStates, modelcur.nbFrames, modelcur.nbVar, reflist, Priors, None, None, None)