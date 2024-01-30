# -*- coding: utf-8 -*-
"""
Created on Mon Sep  2 09:45:01 2019

@author: administration
"""

import numpy as np 
import pandas as pd 
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt
import os
from pclpy import pcl
import pclpy

def segClusters(vLevel):
    pathbase = 'F:/Modeling'
    
    fileInput = os.path.join(pathbase,'analyze', 'SOR')
    levelS = []
    filenames = os.listdir(fileInput)
    for ii, filename in enumerate(filenames):
        if filename[0] == 'H':
            levelS.append(filename)

    for ii, levelN in enumerate(levelS):
        fileInput_hLevel = os.path.join(fileInput, levelN, str(vLevel)+'.pts')
        fileOtput_vLevel = os.path.join(pathbase,'segDevice', str(vLevel) )
        fileOtput_hLevel = os.path.join(fileOtput_vLevel,levelN)
        
        if not os.path.exists(fileOtput_vLevel):
            os.mkdir(fileOtput_vLevel)
        if not os.path.exists(fileOtput_hLevel):
            os.mkdir(fileOtput_hLevel)
            
        data = loadPCD(fileInput_hLevel)
        
        if vLevel == 500:
            if ii == 0:
                planeVoxel = .3
                data = data[ np.where(data[:,2]>1.5)[0],:]
            else:
                planeVoxel = .4
            radiusThreshold = 4*planeVoxel
        elif vLevel == 220:
            if ii == 0:
                planeVoxel = .1
            elif ii == 1:
                planeVoxel = .2
            elif ii ==2:
                planeVoxel = .3
            elif ii==3:
                planeVoxel = .3
            elif ii==4:
                planeVoxel = .2
            radiusThreshold = 3*planeVoxel
        elif vLevel == 35:
            planeVoxel = .2
            radiusThreshold = .8
        
        data, loc, idxCluster = Loc_Idx(data, planeVoxel=planeVoxel, 
                                        radiusThreshold=radiusThreshold)
        ioClusters(data, idxCluster, loc, plotB=False,  saveB=True, 
                   savepath = fileOtput_hLevel)
    return 

def subsample_Voxel(data, resl_T):
    
    data = np.array(data, np.float32)
    pc = pcl.PointCloud.PointXYZ.from_array( data[:,:3] )
    pc = pclpy.octree_voxel_downsample(pc, resolution=resl_T, centroids=True)
    data = pc.xyz 

    return data
    
def loadPCD(filename):
    data = pd.read_csv( filename, sep=' ', skiprows = 1, header=None )
    data = data.iloc[:,0:3]
    return data.to_numpy()

def grabLoc_Lattice(data, planeVoxel, N_thred = 0.):
    x_min, x_max, y_min, y_max = calConerPts2D(data)
    N, Xedges, Yedges = np.histogram2d( data[:,0], data[:,1],  [ 
                                   int(np.floor( (x_max- x_min)/planeVoxel)),
                                   int(np.floor( (y_max- y_min)/planeVoxel))] )  
    idxRow, idxCol = np.where( N>N_thred )
    Xnodes = ( Xedges[0:-1] + Xedges[1:])/2
    Ynodes = ( Yedges[0:-1] + Yedges[1:])/2
    Xloc = Xnodes[idxRow]
    Yloc = Ynodes[idxCol]
#    figure; plot(Yloc,-Xloc,'*','MarkerSize',1)
    loc = np.vstack([Xloc,Yloc]).transpose()
    return loc
    
def Loc_Idx(data, planeVoxel, radiusThreshold):
    
    
    def grabClusters_Lattice(loc, radiusThreshold=.6):
        tree = cKDTree(loc, 50)
    #    distance, idxPairs = tree.query(x = loc, k = 10)
    #    idxRow, idxCol = np.where(distance<radiusThreshold)
        idxPairs = tree.query_pairs(r=radiusThreshold, output_type='ndarray' )
        idxPairs = idxPairs[np.argsort(idxPairs[:,0], kind='mergesort'),:]
        
#        idxMerge = mergePairs(idxPairs)
        
        return idxPairs
    def mergePairs(idxPairs):
        def checkIndNew(idxNew):
            r = 0
            for ii in range(len(idxNew)):
                for jj in range(len(idxNew)):
                    if ii!=jj:
                        r = r + len( np.intersect1d(idxNew[ii],idxNew[jj]) )
            return r
        idxNew = []
        idxNew.append( idxPairs[0,:] )
        idxPairsRemain = idxPairs
        while len(idxPairsRemain)>0:
            nextList = False
            while not nextList:
                for jj, jcell in enumerate(idxNew[-1]):
                    idxTemp = np.where(idxPairsRemain==jcell)[0]
                    if len(idxTemp) > 0:
                        celTemp = np.ravel(idxPairsRemain[idxTemp,:])
                        idxNew[-1] = np.unique( np.hstack( [idxNew[-1],celTemp] ) )
                        idxPairsRemain = np.delete(idxPairsRemain, idxTemp, axis=0)
                        if len(idxPairsRemain) == 0:
                            if ( checkIndNew(idxNew) ) == 0:
                                print ( 'finish segment' )
                            return idxNew
                        break
                    if (jj+1) == len(idxNew[-1]):
                        nextList = True
                        idxNew.append(idxPairsRemain[0])
                        idxPairsRemain = np.delete(idxPairsRemain, 0, axis=0)
                

        return idxNew
    loc = grabLoc_Lattice(data, planeVoxel) 
    idxPairs = grabClusters_Lattice(loc, radiusThreshold )
    resl_T = .1
    while len(idxPairs) > 40000:
        resl_T = resl_T+.1
        data = subsample_Voxel(data, resl_T)
        loc = grabLoc_Lattice(data, planeVoxel)    
        idxPairs = grabClusters_Lattice(loc, radiusThreshold )
        if resl_T > .1:
            break
        
    idx_Cluster = mergePairs(idxPairs)
    return data, loc, idx_Cluster

def clusters2box(loc_Lattice, clustersIdx):
    deviceRange = np.zeros( (len(clustersIdx),5), dtype = float )
    for ii, clusterIdx in enumerate(clustersIdx):
        x_min, x_max, y_min, y_max = calConerPts2D(loc_Lattice[clusterIdx,:])
        deviceRange[ii,0] = ii+1
        deviceRange[ii,1] = x_min
        deviceRange[ii,2] = x_max
        deviceRange[ii,3] = y_min
        deviceRange[ii,4] = y_max
    return deviceRange
def ioClusters(data, clustersIdx, loc_Lattice, plotB=False, saveB=False, savepath=False):
    if plotB:
        figure,ax=plt.subplots(1,1)
        ax.set_xlim( [np.min(data[:,0]), np.max(data[:,0])] )
        ax.set_ylim( [np.min(data[:,1]), np.max(data[:,1])] )
        for ii, clusterIdx in enumerate(clustersIdx):
            plt.plot( loc_Lattice[clusterIdx,0], loc_Lattice[clusterIdx,1],  '.', 'markersize',.1)
            plt.pause(.1)
    if saveB:
        deviceRange = clusters2box(loc_Lattice, clustersIdx)
        if not os.path.exists(savepath):
            os.mkdir(savepath)
        np.savetxt( savepath+'/idx_BOX.out', deviceRange,  fmt='%4.2f' )
    return

def calConerPts2D(data):
    x_max = np.max( data[:,0] )
    x_min = np.min( data[:,0] )
    y_max = np.max( data[:,1] )
    y_min = np.min( data[:,1] )
    return x_min, x_max, y_min, y_max



if __name__ == '__main__':
#    segRange(VoltageLevel = 35) 
#    segRange(VoltageLevel = 220) 
    segClusters(vLevel = 220) 
    segClusters(vLevel = 500) 
    print ('Good Job!!!')