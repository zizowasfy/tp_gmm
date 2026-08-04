from modelClass import model
from init_proposedPGMM_timeBased import init_proposedPGMM_timeBased
from EM_tensorGMM import EM_tensorGMM
from reproduction_DSGMR import reproduction_DSGMR, recompute_DSGMR
from plotGMM import plotGMM
import numpy as np
import copy

# import rosbag
# from gaussian_mixture_model.msg import GaussianMixture, Gaussian
from geometry_msgs.msg import PoseArray, Pose
from copy import deepcopy
from tp_gmm.msg import GaussianMixture, Gaussian

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

    def recompute_trajectory(self, rnew, currentPosition):
        return recompute_DSGMR(rnew, self.model, currentPosition)

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
    
    # Converts the learnded GMM into a format that gmm_rviz_converter node can visualize it in Rviz
    def convertToGM(self, r, down_sample_factor, frame_id):

        ## converting to GaussianMixture() msg 
        nbGaussians = r.Mu.shape[1]
        gmm = GaussianMixture()
        g = Gaussian()

        # allMeans = np.hsplit(r.Mu[:,:,0], r.Mu.shape[1]) # (4,5) == (4,1)x5  -- each column represents one-unit Gaussian (if nbGaussian = 5)
        # allCovariances = np.ravel(r.Sigma[:,:,:,0]) # (4,4,5) -> (80, 1) --every 16 elements represent one-unit Gaussian 

        for gaus in range(nbGaussians):
            g.means = r.Mu[:,gaus,-1].tolist()
            g.covariances = np.ravel(r.Sigma[:,:,gaus,-1]).tolist()
            # print("convertToGM g.covariances: ", g.covariances)
            gmm.gaussians.append(copy.deepcopy(g))
        gmm.weights = [float(w) for w in self.model.Priors] # or r.H
        gmm.bic = float(r.Data.shape[1]*down_sample_factor)
        # gmm.bic = r.Data.shape[1]
        gmm.header.frame_id = frame_id

        # print("r.Mu[:,1,0] == r.Mu[:,1,-1]: ", r.Mu[:,1,0] == r.Mu[:,1,-1]) # debuging.
        ## Writing to rosbag (ROS 2 format)
        import shutil
        from rosbags.rosbag2 import Writer
        from rclpy.serialization import serialize_message
        import time

        bag_path = Path(Data_DIR) / "tpgmm_mix_bag"
        if bag_path.exists():
            shutil.rmtree(bag_path)
            
        with Writer(bag_path, version=8) as writer:
            conn = writer.add_connection("/gmm/mix", "tp_gmm/msg/GaussianMixture", msgdef="", rihs01="dummy")
            ts = int(time.time() * 1e9)
            writer.write(conn, ts, serialize_message(gmm))

        return gmm