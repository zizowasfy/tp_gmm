import numpy as np
import math
import sys

def gaussPDFfast(Data, Mu, invSigma, detSigma):
    realmin = sys.float_info[3]
    nbVar, nbData = np.shape(Data)

    # Efficient broadcasting instead of tiling
    # Data is (nbVar, nbData), Mu is (nbVar,) or (nbVar, 1)
    diff = Data - Mu.reshape(nbVar, 1)

    # Compute Mahalanobis distance efficiently: (x-mu)^T * Sigma^-1 * (x-mu)
    # invSigma @ diff -> (nbVar, nbData)
    # element-wise multiply with diff -> (nbVar, nbData)
    # sum over rows -> (nbData,)
    prob = np.sum((np.dot(invSigma, diff)) * diff, axis=0)

    prob = np.exp(-0.5*prob)/np.sqrt((np.power((2*math.pi), nbVar))*(detSigma+realmin))
    return prob
