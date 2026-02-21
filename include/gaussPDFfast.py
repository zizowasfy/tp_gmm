import numpy as np
import math
import sys

def gaussPDFfast(Data, Mu, invSigma, detSigma):
    """
    Computes the Gaussian Probability Density Function (PDF) for given data points.

    Optimized version:
    - Avoids np.tile() and np.transpose() to save memory and time.
    - Uses broadcasting for centering data.
    - Uses efficient algebraic simplification for Mahalanobis distance computation.
    """
    realmin = sys.float_info[3]
    nbVar, nbData = np.shape(Data)

    # Ensure Mu is (nbVar, 1) for broadcasting
    # Mu can be (nbVar,) or (nbVar, 1)
    if Mu.ndim == 1:
        Mu = Mu[:, np.newaxis]

    # Optimization: Use broadcasting instead of tile and avoid unnecessary transposes
    centered_data = Data - Mu

    # Optimization: Use algebraic simplification for Mahalanobis distance
    # Original: sum( ((Data-Mu).T @ invSigma) * (Data-Mu).T, axis=1 )
    # Optimized: sum( (invSigma @ (Data-Mu)) * (Data-Mu), axis=0 )
    # This computes x^T * invSigma * x for each column x
    prob = np.sum(np.dot(invSigma, centered_data) * centered_data, 0)

    prob = np.exp(-0.5*prob)/np.sqrt((np.power((2*math.pi), nbVar))*(detSigma+realmin))
    return prob
