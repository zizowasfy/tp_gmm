import numpy as np
import math
import sys

def gaussPDFfast(Data, Mu, invSigma, detSigma):
    realmin = sys.float_info[3]
    nbVar, nbData = np.shape(Data)

    # Use broadcasting to avoid tile and transpose.
    # Data is (nbVar, nbData)
    # Mu is (nbVar,) or (nbVar, 1). Ensure it is (nbVar, 1) for broadcasting
    Mu = Mu.reshape(nbVar, 1)

    Data = Data - Mu

    # Compute Mahalanobis distance term: (x-mu)^T * invSigma * (x-mu)
    # equivalent to sum( (invSigma @ Data) * Data, axis=0 )
    # (nbVar, nbVar) @ (nbVar, nbData) -> (nbVar, nbData)
    # Element-wise multiply -> (nbVar, nbData)
    # Sum over rows (axis 0) -> (nbData,)

    prob = np.sum(np.dot(invSigma, Data)*Data, 0)

    prob = np.exp(-0.5*prob)/np.sqrt((np.power((2*math.pi), nbVar))*(detSigma+realmin))
    return prob
