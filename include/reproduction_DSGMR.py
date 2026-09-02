import numpy as np
def reproduction_DSGMR(DataIn, model, rr, currPos):
    # DataIn = np.linspace(np.amin(DataIn), np.amax(DataIn), np.shape(DataIn)[0])
    # from sklearn import tree

    model.dt = 0.05 # 0.01
    model.kP = 150
    model.kV = 20
    DataIn = np.reshape(DataIn, (1, np.shape(DataIn)[0]))
    from rClass import r
    from computeResultingGaussians import computeResultingGaussians
    from gaussPDFfast import gaussPDFfast
    nbData = np.shape(DataIn)[1]
    iN = range(0, np.shape(DataIn)[0])
    out = range(iN[-1]+1, model.nbVar)
    nbVarOut = len(out)
    currPos = np.reshape(currPos, (nbVarOut,1))

    a = r(nbData,model)
    a.Data = np.zeros((nbVarOut+len(iN), np.shape(DataIn)[1]))
    a.Data[np.ix_(range(0,np.shape(DataIn)[0]), range(0, np.shape(DataIn)[1]))] = DataIn

    prodRes, a.p = computeResultingGaussians(model, rr)
    for t in range(0, len(prodRes)):
        a.Mu[:,:,t] = prodRes[t].Mu
        a.Sigma[:,:,:,t] = prodRes[t].Sigma
        for i in range(0, model.nbStates):
            sigma_in = prodRes[t].Sigma[iN, iN, i][0]
            prodRes[t].invSigmaIn = np.dstack((prodRes[t].invSigmaIn, 1.0 / (sigma_in + 1e-8)))
            prodRes[t].detSigmaIn = np.hstack((prodRes[t].detSigmaIn, max(sigma_in, 1e-12)))

    currVel = np.zeros(shape=(nbVarOut, 1))
    y = np.zeros((len(out), nbData))
    for n in range(0, nbData):
        if len(prodRes) > 1:
            nn = n
        else:
            nn = 0
        for i in range(0, model.nbStates):
            val = gaussPDFfast(np.reshape(DataIn[:, n], (1, 1)), prodRes[nn].Mu[iN, i], prodRes[nn].invSigmaIn[iN, iN, i], prodRes[nn].detSigmaIn[i])
            a.H[i, n] = model.Priors[i] * np.squeeze(val)

        sum_H = np.sum(a.H[:, n])
        if sum_H > 0:
            a.H[:, n] = a.H[:, n] / sum_H

        currTar = np.zeros((nbVarOut, 1))
        for i in range(0, model.nbStates):
            sigma_in_val = prodRes[nn].Sigma[iN, iN, i]
            MuTmp = np.reshape(
                np.reshape(prodRes[nn].Mu[out, i], (nbVarOut, 1)) +
                np.reshape(prodRes[nn].Sigma[out, iN, i], (nbVarOut, len(iN))) * (1.0 / (sigma_in_val + 1e-8)) * (DataIn[:, n] - prodRes[nn].Mu[iN, i]),
                (len(out),)
            )
            currTar = currTar + a.H[i, n] * np.reshape(MuTmp, (nbVarOut, 1))

        currAcc = model.kP * (currTar - currPos) - model.kV * currVel
        currVel = currVel + currAcc * model.dt
        currPos = currPos + currVel * model.dt
        a.Data[:, n] = np.reshape(np.vstack((DataIn[:, n], currPos)), (nbVarOut + np.size(iN),))
        # expData, expSigma, Mu, Sigma = process(a.Data, 5, 100)
        # a.Data = np.vstack((DataIn, y))
    return a

def recompute_DSGMR(a, model, currPos):
    from gaussPDFfast import gaussPDFfast
    model.dt = 0.05
    model.kP = 150
    model.kV = 20

    DataIn = a.Data[0:1, :]
    nbData = np.shape(DataIn)[1]
    iN = range(0, 1)
    out = range(1, model.nbVar)
    nbVarOut = len(out)
    currPos = np.reshape(currPos, (nbVarOut, 1)).astype(np.float64)
    currVel = np.zeros((nbVarOut, 1))

    num_t = a.Mu.shape[2]

    # If the time-dimension exists but only the last slice was updated, propagate it across all time steps
    if num_t > 1 and not np.allclose(a.Mu[1:4, :, -1], a.Mu[1:4, :, 0]):
        if np.allclose(a.Mu[1:4, :, 0], a.Mu[1:4, :, 1]):
            for t in range(num_t - 1):
                a.Mu[1:4, :, t] = a.Mu[1:4, :, -1]

    for n in range(0, nbData):
        t_idx = n if num_t > 1 else -1
        for i in range(0, model.nbStates):
            invSigmaIn = np.array([[1.0 / (a.Sigma[0, 0, i, t_idx] + 1e-8)]])
            detSigmaIn = a.Sigma[0, 0, i, t_idx]
            muIn = np.reshape(a.Mu[0:1, i, t_idx], (1, 1))
            val = gaussPDFfast(np.reshape(DataIn[:, n], (1, 1)), muIn, invSigmaIn, detSigmaIn)
            a.H[i, n] = model.Priors[i] * np.squeeze(val)

        sum_H = np.sum(a.H[:, n])
        if sum_H > 0:
            a.H[:, n] = a.H[:, n] / sum_H

        currTar = np.zeros((nbVarOut, 1))
        for i in range(0, model.nbStates):
            mu_in = a.Mu[0:1, i, t_idx]
            mu_out = np.reshape(a.Mu[out, i, t_idx], (nbVarOut, 1))
            sigma_out_in = np.reshape(a.Sigma[out, 0, i, t_idx], (nbVarOut, 1))
            sigma_in_in = a.Sigma[0, 0, i, t_idx]

            MuTmp = mu_out + sigma_out_in * (1.0 / (sigma_in_in + 1e-8)) * (DataIn[:, n] - mu_in)
            currTar = currTar + a.H[i, n] * np.reshape(MuTmp, (nbVarOut, 1))

        currAcc = model.kP * (currTar - currPos) - model.kV * currVel
        currVel = currVel + currAcc * model.dt
        currPos = currPos + currVel * model.dt
        a.Data[:, n] = np.squeeze(np.vstack((DataIn[:, n], currPos)))

    return a