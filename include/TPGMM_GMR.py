from modelClass import model
from init_proposedPGMM_timeBased import init_proposedPGMM_timeBased
from EM_tensorGMM import EM_tensorGMM
from reproduction_DSGMR import reproduction_DSGMR
from plotGMM import plotGMM
import numpy as np
import copy

# import rosbag

from geometry_msgs.msg import PoseArray, Pose
from copy import deepcopy
# from tp_gmm.msg import GaussianMixture, Gaussian

from pathlib import Path
Data_DIR = str(Path(__file__).resolve().parent.parent / 'data') + '/'

class TPGMM_GMR(object):
    def __init__(self, nbStates, nbFrames, nbVar):
        self.model = model(nbStates, nbFrames, nbVar, None, None, None, None, None)

    def fit(self, s):
        self.s = s
        self.model = init_proposedPGMM_timeBased(s, self.model)
        self.model = EM_tensorGMM(s, self.model)

    def reproduce(self, p, currentPosition):
        return reproduction_DSGMR(self.s[0].Data[0,:], self.model, p, currentPosition)

    def plotReproduction(self, r, xaxis, yaxis, ax, showGaussians = True, lw = 7):
        for m in range(r.p.shape[0]):
            ax.plot([r.p[m, 0].b[xaxis, 0], r.p[m, 0].b[xaxis, 0] + r.p[m, 0].A[xaxis, yaxis]],
                     [r.p[m, 0].b[yaxis, 0], r.p[m, 0].b[yaxis, 0] + r.p[m, 0].A[yaxis, yaxis]],
                     lw=lw, color=[0, 1, m])
            ax.plot(r.p[m, 0].b[xaxis, 0], r.p[m, 0].b[yaxis, 0], ms=30, marker='.', color=[0, 1, m])
        ax.plot(r.Data[xaxis, 0], r.Data[yaxis, 0], marker='.', ms=15)
        ax.plot(r.Data[xaxis, :], r.Data[yaxis, :])
        if showGaussians:
            plotGMM(r.Mu[np.ix_([xaxis, yaxis], range(r.Mu.shape[1]), [0])],
                    r.Sigma[np.ix_([xaxis, yaxis], [xaxis, yaxis], range(r.Mu.shape[1]), [0])], [0.5, 0.5, 0.5], 1, ax)

    def getReproductionMatrix(self, r):
        return r.Data
    
    ## Get the Data points after being transfromed w.r.t. local frames (frame1, frame2), and then converts them into: PoseArray for visualization ; FLOATARRAY to compute the BIC  
    def getDataAll(self, s):
        nbSamples = len(s)
        DataTotalSize = 0
        for i in range(0, len(s)):
            DataTotalSize = DataTotalSize + s[i].nbData
        DataAll = np.ndarray(shape = (0,DataTotalSize))
        Data_posearray = PoseArray()
        Data_pose = Pose()
        for i in range (0, 1): #self.model.nbFrames): # A try to include the points from one frame only
            DataTmp = np.ndarray(shape=(np.shape(s[0].Data)[0], 0))
            # print("s[0].Data.shape: ", s[0].Data.shape)
            # print("DataTmp.shape: ", DataTmp.shape)
            # print("nbSamples=len(s): ", len(s))
            for j in range (0, nbSamples):
                for k in range (0, s[j].nbData):
                    DataTmp = np.append(DataTmp, np.dot(s[j].p[i,k].invA,(np.reshape(s[j].Data[:,k], (np.shape(s[0].Data)[0],1)) - np.reshape(s[j].p[i, k].b, (np.shape(s[0].Data)[0], 1)))), axis = 1) # invA @ (Data-b)
                    ## Putting the Data points (after transforming them w.r.t. the local frame not the the world frame) into PoseArray mainly for visualization
                    # print("DataTmp.shape: ", DataTmp.shape)
                    Data_pose.position.x = DataTmp[1,k]
                    Data_pose.position.y = DataTmp[2,k]
                    Data_pose.position.z = DataTmp[3,k]
                    Data_posearray.poses.append(deepcopy(Data_pose))        
            DataAll = np.append(DataAll, DataTmp, axis=0)
        return Data_posearray, DataAll
    