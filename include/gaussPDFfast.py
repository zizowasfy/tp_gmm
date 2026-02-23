import numpy as np
import math
import sys

def gaussPDFfast(Data, Mu, invSigma, detSigma):
    """
    Optimized version of Gaussian PDF calculation.

    Args:
        Data: (nbVar, nbData) array of data points
        Mu: (nbVar,) or (nbVar, 1) array of means
        invSigma: (nbVar, nbVar) inverse covariance matrix
        detSigma: scalar determinant of covariance matrix
    """
    realmin = sys.float_info[3]
    nbVar, nbData = np.shape(Data)

    # Ensure Mu is column vector for correct broadcasting: (nbVar, 1) against (nbVar, nbData)
    if Mu.ndim == 1:
        Mu = Mu[:, np.newaxis]
    elif Mu.ndim == 2 and Mu.shape[0] == 1 and Mu.shape[1] == nbVar:
         # Handle case where Mu might be passed as a row vector (1, nbVar)
         Mu = Mu.T

    # Calculate Data - Mu without transposing/tiling. Broadcasting handles expansion.
    # Data is (nbVar, nbData), Mu is (nbVar, 1) -> diff is (nbVar, nbData)
    diff = Data - Mu

    # Efficient Mahalanobis distance calculation: (x-u)^T S^-1 (x-u)
    # We compute: sum( (S^-1 @ (x-u)) * (x-u), axis=0 )
    # This avoids creating large intermediate matrices from tiling.
    prob = np.sum(np.dot(invSigma, diff) * diff, axis=0)

    prob = np.exp(-0.5*prob) / np.sqrt(((2*math.pi)**nbVar) * (detSigma + realmin))
    return prob
