import numpy as np
import math
import sys

def gaussPDFfast(Data, Mu, invSigma, detSigma):
    """
    Computes the Gaussian Probability Density Function (PDF) for a given dataset.

    Optimized implementation using broadcasting and einsum to improve performance.

    Args:
        Data: (nbVar, nbData) numpy array of data points.
        Mu: (nbVar,) or (nbVar, 1) numpy array representing the mean vector.
        invSigma: (nbVar, nbVar) numpy array representing the inverse covariance matrix.
        detSigma: scalar, determinant of the covariance matrix.

    Returns:
        prob: (nbData,) numpy array of probability densities.
    """
    realmin = sys.float_info[3]
    nbVar, nbData = np.shape(Data)

    # Center the data.
    # Data is (nbVar, nbData). Mu is typically (nbVar,) or (nbVar, 1).
    # We transpose Data to (nbData, nbVar) for row-major operations.
    # Mu is broadcasted automatically against the rows of Data.T, avoiding the need for np.tile.
    Data = Data.T - Mu.T

    # Calculate the Mahalanobis distance squared: (x-mu)^T * invSigma * (x-mu)
    # We use np.einsum for efficient computation:
    # 'ij,jk,ik->i':
    #   i: sample index (0 to nbData-1)
    #   j: dimension index (0 to nbVar-1)
    #   k: dimension index (0 to nbVar-1)
    # This computes sum_j sum_k (Data[i,j] * invSigma[j,k] * Data[i,k]) for each sample i.
    # Using optimize=True allows numpy to choose the best contraction path.
    prob = np.einsum('ij,jk,ik->i', Data, invSigma, Data, optimize=True)

    # Calculate the normalization constant
    denom = np.sqrt(((2*math.pi)**nbVar) * (detSigma + realmin))

    # Compute final probability density
    prob = np.exp(-0.5 * prob) / denom

    return prob
