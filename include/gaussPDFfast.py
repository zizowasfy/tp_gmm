import numpy as np
import math
import sys

def gaussPDFfast(Data, Mu, invSigma, detSigma):
    """
    Computes the Gaussian Probability Density Function for multiple data points.

    Optimized version using broadcasting to avoid large matrix allocations (np.tile).

    Args:
        Data: (nbVar, nbData) array of data points
        Mu: (nbVar, 1) or (nbVar,) mean vector
        invSigma: (nbVar, nbVar) inverse covariance matrix
        detSigma: scalar determinant of covariance matrix
    Returns:
        prob: (nbData,) probability density values
    """
    realmin = sys.float_info[3]
    nbVar, nbData = np.shape(Data)

    # Ensure Mu is (nbVar, 1) for broadcasting against (nbVar, nbData)
    if Mu.ndim == 1:
        Mu = Mu[:, np.newaxis]

    # Center the data
    # (nbVar, nbData) - (nbVar, 1) -> (nbVar, nbData)
    D = Data - Mu

    # Mahalanobis distance calculation: (x-mu)^T * Sigma^-1 * (x-mu)
    # Equivalent to sum((invSigma @ D) * D, axis=0) using broadcasting
    # invSigma @ D -> (nbVar, nbData)
    # * D -> element-wise multiplication -> (nbVar, nbData)
    # sum(..., axis=0) -> sum over variables -> (nbData,)
    mahalanobis = np.sum((invSigma @ D) * D, axis=0)

    # Calculate probability
    prob = np.exp(-0.5 * mahalanobis) / np.sqrt(((2*math.pi)**nbVar) * (detSigma + realmin))
    return prob
