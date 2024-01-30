# -*- coding: utf-8 -*-
"""
Created on Wed Dec 25 10:27:37 2019

@author: administration
"""
from Cluster_Kernal import chekCross, Merge_AdjacentBox, megerIdx, pltFrame, Get_dimXYZ_from_PCD
from IO_PCD import Get_PTS_from_file, Get_PTS_from_path, Get_PTS_from_path_FileS
from IO_PCD import Get_PTS_from_fileS, SavePCD, SavePCDs, SaveBox, Save_PCD_use_BoxS, Load_PCDascii_to_mat
from Utilize import Get_PCD_idx_use_Box, Get_PTSname_from_Path, Get_PCD_in_PCD_use_Boxs
import IO_PCD

# from seg_classify_Level import megerIdx, pltFrame, class_base_Long, addBoxs, expd_chek_Box, extrct_pts_Grd
import Basic_Info
import cluster_height
import Utilize
from pclpy import pcl
import numpy as np
import pandas as pd
import os, time
from scipy import spatial
from functools import reduce
from sklearn.cluster import KMeans
from scipy.spatial import cKDTree
from sklearn.neighbors import KDTree
from sklearn.cluster import DBSCAN
from matplotlib import pyplot as plt
from scipy.spatial.distance import cdist

from scipy.spatial.distance import pdist
from scipy.cluster.hierarchy import single, fcluster

import pcd__filter_PCD_SOR


def Cluster_DBSCAN_3D( data, eps=3, minPts=10 ):
    labelS = DBSCAN(eps=eps, min_samples=minPts).fit_predict(data[:,:3])
    labelS_uniq = np.unique(labelS)
    clustersIdx = []
    for label in labelS_uniq:
        if label >=0:
            clustersIdx.append( np.where(labelS==label)[0] )
    return clustersIdx
def Cluster_Loc_Lattice(loc_Lattice, radiusThreshold):
    data = np.hstack([loc_Lattice,np.zeros((len(loc_Lattice),1),dtype=float)])
    pc = pcl.PointCloud.PointXYZ.from_array( data[:,:3] )
    indices = pc.extract_clusters(tolerance = radiusThreshold, min_size=2, max_size=1000000)
    clustersIdx = []
    for idx_i in range(len(indices)):
        indice = indices[idx_i].indices
        cTemp = []
        for idx_j in range(len(indice)):
            cTemp.append(indice[idx_j])
        clustersIdx.append(cTemp)
    return clustersIdx

def Cluster_Loc_3D(data, radiusThreshold):
    pc = pcl.PointCloud.PointXYZ.from_array( data[:,:3] )
    indices = pc.extract_clusters(tolerance = radiusThreshold, min_size=2, max_size=100000000)
    clustersIdx = []
    for idx_i in range(len(indices)):
        indice = indices[idx_i].indices
        cTemp = []
        for idx_j in range(len(indice)):
            cTemp.append(indice[idx_j])
        clustersIdx.append(cTemp)
    return clustersIdx

def ExtractData_Horizental_Slice(data, h_low, h_top):
    idx_low = np.where( data[:,2] >= h_low )[0]
    idx_top = np.where( data[:,2] <  h_top )[0]
    idxS = np.intersect1d(idx_low, idx_top)
    data = data[idxS, :]
    return data

def Get_BOX_from_file(pathName, fileName):
    filebox = os.path.join(pathName, fileName)
    boxS = np.loadtxt(filebox)
    return boxS



def Get_SLICEname_from_Path(pathName):
    fileNames = FileNameS_Filter( os.listdir(  pathName ), 0,'H')    
    if   len(fileNames) == 0:
        fileSlices = False
    elif len(fileNames) == 1:
        fileSlices = fileNames
    elif len(fileNames) >  1:
        fileSlices = fileNames
    return fileSlices


def Get_Cluster_in_Slices(pathRegion, levelS):
    boxS_Cluster = []
    for levelN in levelS:
        pathLevel = os.path.join(pathRegion, levelN) 
        boxS = Get_BOX_from_file(pathLevel , 'box.out')
        idxBox = np.argsort(boxS[:,0])
        boxS = boxS[idxBox,:]
        boxS_Cluster.append(boxS)
    boxS = boxS_Cluster
    return boxS

def Update_Box_Remove(boxS, boxR):
    for ib, boxes in enumerate(boxS):
        boxTemp = []
        for box in boxes:
            boxCross = False
            for box_r in boxR:
                if chekCross(box[1:], box_r[1:]):
                    boxCross = True
                    break
            if not boxCross:
                boxTemp.append(box)
        boxTemp = np.asarray(boxTemp)
        boxS[ib] = boxTemp
        
    return boxS

def Remove_BusBar(boxS_dw, boxS_up, fil_Len_dw, fil_Len_up, fil_Up_num):
    boxS_dw = Filter_boxS_XorY_More_Len(boxS_dw, fil_Len_dw)
    boxS_up = Filter_boxS_XorY_More_Len(boxS_up, fil_Len_up)
#    pltFrame(boxS_1[:,1:], ['b','b'])
#    pltFrame(boxS_2[:,1:], ['g','g'])
    boxT = []
#    plt.figure()
    for box_dw in boxS_dw:
        box_Num = 0
        for box_up in boxS_up:
            if chekCross(box_up[1:], box_dw[1:]):
                box_Num = box_Num+1
#                pltFrame(box_up[1:], ['r','r'])
            if box_Num == 3:
                boxT.append(box_dw)
#                pltFrame(box_dw[1:], ['g','g'])
                break
    boxT = np.asarray(boxT)
    boxT[:,1] = boxT[:,1]-0.5
    boxT[:,2] = boxT[:,2]+0.5
    boxT[:,3] = boxT[:,3]-0.5
    boxT[:,4] = boxT[:,4]+0.5
    return boxT

def Filter_boxS_XandY_More_Len(boxS, fil_Len):
    boxT = []
    for box in boxS:
        dx = np.abs( box[1] - box[2] )
        dy = np.abs( box[3] - box[4] )
        if dx > fil_Len and dy >fil_Len:
            boxT.append(box)
    return np.asarray(boxT)
def Filter_boxS_XorY_Less_Len(boxS, fil_Len):
    boxT = []
    for box in boxS:
        dx = np.abs( box[1] - box[2] )
        dy = np.abs( box[3] - box[4] )
        if dx < fil_Len or dy < fil_Len:
            boxT.append(box)
    return np.asarray(boxT)

def Filter_boxS_XorY_noMore_Len(boxS, fil_Len):
    boxT = []
    for box in boxS:
        dx = np.abs( box[1] - box[2] )
        dy = np.abs( box[3] - box[4] )
        if dx > fil_Len or dy > fil_Len:
            continue
        else:
            boxT.append(box)
    return np.asarray(boxT)

def Filter_boxS_XandY_noMore_Len(boxS, fil_Len):
    boxT = []
    for box in boxS:
        dx = np.abs( box[1] - box[2] )
        dy = np.abs( box[3] - box[4] )
        if dx > fil_Len and dy > fil_Len:
            continue
        else:
            boxT.append(box)
    return np.asarray(boxT)

def Filter_boxS_XorY_More_Len(boxS, fil_Len):
    boxT = []
    for box in boxS:
        dx = np.abs( box[1] - box[2] )
        dy = np.abs( box[3] - box[4] )
        if dx > fil_Len or dy > fil_Len:
            boxT.append(box)
    return np.asarray(boxT)

def Filter_pcd_with_X_and_Y_Len_Greater(data, xt, yt):
    x_dim, y_dim, _ = Get_dimXYZ_from_PCD(data, 'xy')
    if x_dim >xt and  y_dim >yt:
        return True
    else:
        return False
    
def Filter_pcd_with_X_and_Y_Less_Len(data, xt, yt):
    x_dim, y_dim, _ = Get_dimXYZ_from_PCD(data, 'xy')
    if x_dim <xt and  y_dim<yt:
        return True
    else:
        return False
    
def Filter_pcd_with_X_or_Y_Len_Greater(data, xt, yt):
    x_dim, y_dim, _ = Get_dimXYZ_from_PCD(data, 'xy')
    if x_dim >xt or y_dim >yt:
        return True
    else:
        return False

def Filter_pcd_with_X_or_Y_Less_Len(data, xt, yt):
    x_dim, y_dim, _ = Get_dimXYZ_from_PCD(data, 'xy')
    if x_dim <xt or y_dim <yt:
        return True
    else:
        return False
    
def Filter_pcd_with_X_or_Y_or_Z_Len_Less(data, xt, yt, zt):
    x_dim, y_dim, z_dim = Get_dimXYZ_from_PCD(data, 'xyz')
    if x_dim <xt or y_dim <yt or z_dim <zt:
        return True
    else:
        return False
def Filter_pcd_with_XorY_and_Z_Len_Less(data, xt, yt, zt):
    x_dim, y_dim, z_dim = Get_dimXYZ_from_PCD(data, 'xyz')
    if (x_dim <xt or y_dim <yt) and z_dim <zt:
        return True
    else:
        return False

def Filter_pcd_with_Z_Less_Len(data, zt):
    _, _, z_dim = Get_dimXYZ_from_PCD(data, 'z')
    if  z_dim <zt:
        return True
    else:
        return False
    


def Removal_Gantry_Feet(pathBase):
    fileRegions= os.listdir(pathBase)
    for fileRegion in fileRegions:
#        fileRegion = fileRegions[-1]
        pathRegion = os.path.join(pathBase,fileRegion)
        
        filePtsS = Get_PTSname_from_Path(pathRegion)
        pathGantry = os.path.join(pathRegion,'Gantry_Feet')
        boxS = np.loadtxt( os.path.join(pathGantry,'box.out') )

        for ii, filePts in enumerate(filePtsS):
            if ii>0:
                ii=ii
            data = Get_PTS_from_file(pathRegion, filePts)
            
            for ib, box in enumerate(boxS):
                print('\r'+':  {ib}/{lenF}'.format(ib=ib+1,lenF=len(boxS)),end='',flush=True)
                idxM = Get_PCD_idx_use_Box(data, box)
                if len(idxM) > 0:
                    data = np.delete(data, idxM, axis=0)
            print('\n')
            SavePCD(data, pathGantry, filePts)

def Cluster_in_Slices_for_Region(pathRegion):
    # fileRegions = os.listdir(pathBase)
    p_Voxel = 0.2
    
    # for fileRegion in fileRegions:
    # fileRegion = fileRegions[1]
    # pathRegion = os.path.join(pathBase,fileRegion)
    fileNames = os.listdir(pathRegion)
            
    levelS = FileNameS_Filter(fileNames, 0, 'H')
    
    for levelN in levelS:
        path_Hlvl = os.path.join(pathRegion, levelN)
        data = Get_PTS_from_path(path_Hlvl)
        
        deviceBoxS = Trans_PCD_2_BoxS(data, p_Voxel, clusterR = 2*p_Voxel)
        SaveBox(deviceBoxS, path_Hlvl, 'box.out')
#            cluster_height.ioClusters(data, clustersIdx, loc_Lattice, plotB=False, saveB=True, savepath=savepath)
    return 

def Cluster_in_Slices(pathBase):
    fileRegions = os.listdir(pathBase)
    p_Voxel = 0.2
    
    for fileRegion in fileRegions:
        # fileRegion = fileRegions[1]
        pathRegion = os.path.join(pathBase,fileRegion)
        fileNames = os.listdir(pathRegion)
                
        levelS = FileNameS_Filter(fileNames, 0, 'H')
        
        for levelN in levelS:
            path_Hlvl = os.path.join(pathRegion, levelN)
            data = Get_PTS_from_path(path_Hlvl)
            
            deviceBoxS = Trans_PCD_2_BoxS(data, p_Voxel, clusterR = 2*p_Voxel)
            SaveBox(deviceBoxS, path_Hlvl, 'box.out')
#            cluster_height.ioClusters(data, clustersIdx, loc_Lattice, plotB=False, saveB=True, savepath=savepath)
    return 

def FileNameS_Filter(fileNames, idx, word_Chk):
    fileName_F = []
    if idx == 0:
        for ii, fileName in enumerate(fileNames):
            if ii > 0: ii=ii
            if fileName[0] == word_Chk:
                fileName_F.append(fileName)
    elif idx > 0:
        for ii, fileName in enumerate(fileNames):
            if fileName[:idx] == word_Chk:
                fileName_F.append(fileName)                
    elif idx <0:
        for ii, fileName in enumerate(fileNames):
            if fileName[idx:] == word_Chk:
                fileName_F.append(fileName)
    return fileName_F
    
def Trans_PCD_2_BoxS(data, p_Voxel, clusterR):
    loc_Lattice = cluster_height.grabLoc_Lattice(data, planeVoxel=p_Voxel)
            
    clustersIdx = Cluster_Loc_Lattice(loc_Lattice, clusterR)
            
    deviceBoxS = cluster_height.clusters2box(loc_Lattice, clustersIdx)
    deviceBoxS = Reorganize_Mergeboxs(deviceBoxS[:,1:], p_Voxel)
    deviceBoxS = np.hstack( [np.arange(len(deviceBoxS))[:,np.newaxis],deviceBoxS])
    return deviceBoxS
    
            
def Reorganize_Mergeboxs(boxSourceS, planeVoxel):
    boxTargetS = []
    
    iit = 0
    boxTargetS = boxSourceS[iit][np.newaxis,:]
    boxSourceS = np.delete(boxSourceS, iit,axis=0)
    
    cross_t = True
    cross_add = False
    len_pre = 0
    while cross_t:
        
        if cross_add:
#            boxTargetS,_ = Merge_AdjacentBox(boxTargetS, idxS=0)
            boxTemp,_ = Merge_AdjacentBox(boxTargetS[iit:], idxS=0)
            boxTargetS= np.vstack( [boxTargetS[:iit,:], boxTemp] )
            
#        print(len(boxTargetS), iit)
        boxTarget = boxTargetS[iit, :]
        cross_add = False
        cross_s = True
        len_pre = len(boxTargetS)
        iis = 0
        while cross_s and iis<len(boxSourceS):               
            boxSource = boxSourceS[iis,:]
            if chekCross(boxTarget, boxSource):
                boxTargetS = np.vstack( [boxTargetS, boxSource[np.newaxis,:]] )
                boxSourceS = np.delete(boxSourceS, iis,axis=0)
                cross_add = True
                continue
            iis = iis+1
            if iis == len(boxSourceS):
                cross_s = False
        if len_pre == len(boxTargetS):
            iit = iit + 1
            boxTargetS = np.vstack( [boxTargetS, boxSourceS[0][np.newaxis,:]] )
            boxSourceS = np.delete(boxSourceS, 0, axis=0)
        print('\r'+'remain: {lenB}'.format(lenB=len(boxSourceS)),end='',flush=True)

        if len(boxSourceS)==0:
            cross_t = False
    print('\n')
    boxTargetS,_ = Merge_AdjacentBox(boxTargetS, idxS=0)
    return boxTargetS

def Search_ZB_use_Wall(boxS, boxS_Walls):
    box_exp = 0.5
    x_walls = (boxS_Walls[:,1] + boxS_Walls[:,2] )/2
    Z = single(pdist(x_walls[:,np.newaxis]))
    clusterS = fcluster(Z, 20, criterion='distance')
    clusterS_uniq = np.unique(clusterS)
    box_Wall_AreaS = []
    for cluster_u in clusterS_uniq:
        boxID = np.where(clusterS==cluster_u)[0]
        box_Wall_AreaS.append( boxS_Walls[boxID] )

    boxZBS = []
    for box_WallS in box_Wall_AreaS:
        boxS = Update_Box_Remove([boxS], box_WallS)[0]
        dx = np.mean( box_WallS[:,2] - box_WallS[:,1] )
        dy = np.mean( box_WallS[:,4] - box_WallS[:,3] )
        
        if dy > dx:
            axis_srh = 1
            axis_vrt = 3
        if dy < dx:
            axis_srh = 3
            axis_vrt = 1
        ax_cent = ( box_WallS[:,axis_srh+1] + box_WallS[:,axis_srh] ) / 2
        ax_idx = np.argsort(ax_cent, kind='mergesort')
        boxWS = []
        boxZB = []
        for ib, axID in enumerate(ax_idx):
            boxW = box_WallS[axID]
#            pltFrame(boxW[1:], ['g','g'])
            boxWS.append( box_WallS[axID] )
            boxT1 = []
            boxT2 = []
            for box in boxS:
                if ib == 0:
                    if box[axis_vrt] > boxW[axis_vrt]-box_exp and \
                       box[axis_vrt+1] < boxW[axis_vrt+1]+box_exp and \
                       boxW[axis_srh] - box[axis_srh] <= 12 and boxW[axis_srh] - box[axis_srh+1]>=0:
                        boxT1.append(box)
#                        pltFrame(box[1:], ['g','g'])
                if ib < len(ax_idx) - 1:
                    box_Wall_nxt = box_WallS[ ax_idx[ib+1] ]
                    if box[axis_vrt] > boxW[axis_vrt]-box_exp and \
                       box[axis_vrt+1] < boxW[axis_vrt+1]+box_exp and \
                       boxW[axis_srh+1] <= box[axis_srh] and box_Wall_nxt[axis_srh] >= box[axis_srh+1]:
                        boxT2.append(box)
#                        pltFrame(box[1:], ['g','g'])
                if ib == len(ax_idx) - 1:
                    if box[axis_vrt] > boxW[axis_vrt]-box_exp and \
                       box[axis_vrt+1] < boxW[axis_vrt+1]+box_exp and \
                       box[axis_srh+1] - boxW[axis_srh+1]  <= 12 and box[axis_srh] - boxW[axis_srh+1]>=0 :
                        boxT2.append(box)
#                        pltFrame(box[1:], ['g','g'])
            if len(boxT1) > 0:
                boxZB.append(np.vstack(boxT1))
                if len(boxT2)>0:
                    boxZB.append(np.vstack(boxT2))
            elif len(boxT1) == 0:
                boxZB.append(np.vstack(boxT2))
                
        boxZBS.append(boxZB)
    return boxZBS, box_Wall_AreaS
    


def Save_box_ZB_Wall_MixInfo(boxS_ZBS_AreaS, boxS_Wall_AreaS, pathBase):
    if not os.path.exists(pathBase):
        os.mkdir(pathBase)
        
    for i_area, box_Wall_Area in enumerate(boxS_Wall_AreaS):
        file_Area_Name = 'area_{ii:03d}'.format(ii=i_area)
        box_Area_Name = 'area_{ii:03d}.out'.format(ii=i_area)
        path_Area_Name =  os.path.join(pathBase, file_Area_Name)
        if not os.path.exists( path_Area_Name ):
            os.mkdir( path_Area_Name )
            
        SaveBox(box_Wall_Area, path_Area_Name, box_Area_Name)
        
        boxS_ZBS = boxS_ZBS_AreaS[i_area]
        for i_zb, boxS_ZB in enumerate(boxS_ZBS):
            file_ZB_Name = 'ZB_{ii:03d}'.format(ii=i_zb)
            box_ZB_Name = 'ZB_{ii:03d}'.format(ii=i_zb)
            
            SaveBox(np.vstack(boxS_ZB), os.path.join(path_Area_Name,file_ZB_Name), box_ZB_Name)
            
        
def Trans_ZB_Wall_MixInfo_into_boxS(boxS_ZBS, boxS_Wall_AreaS):
    boxS_ZBS_AreaS = []
    boxDEL = []
    for ib in range(len(boxS_ZBS)):
        boxS_ZBS_AreaS.append( np.vstack(boxS_ZBS[ib]) )
        boxDEL.append([ib, np.min(boxS_ZBS_AreaS[-1][:,1]), 
                       np.max(boxS_ZBS_AreaS[-1][:,2]), 
                       np.min(boxS_ZBS_AreaS[-1][:,3]), 
                       np.max(boxS_ZBS_AreaS[-1][:,4])] )
    
    boxS_ZBS_AreaS  = np.vstack(boxS_ZBS_AreaS)
    boxS_Wall_AreaS = np.vstack(boxS_Wall_AreaS)
    
    boxS = np.vstack([boxS_ZBS_AreaS,boxS_Wall_AreaS] )
    return boxS, np.vstack(boxDEL)
    
def Cluster_35(pathRegion, pathfilePCDS_in, ZB=0):
    fileSlices = Get_SLICEname_from_Path(pathRegion)
    boxS = Get_Cluster_in_Slices(pathRegion, fileSlices)
    
    pathSaveBox = os.path.join(pathRegion,'zbBox')
    if not os.path.exists(pathSaveBox):
        os.mkdir(pathSaveBox)
    boxS_DEL = []
    ''' 去除建筑物 '''
    box_H04  = megerIdx( boxS[0], boxS[4])
    box_H042 = megerIdx( box_H04, boxS[2])
    box_Building = Filter_boxS_XandY_More_Len(box_H042, 10)
    box_search = Update_Box_Remove([box_H042], box_Building)[0]
    pltFrame(box_search[:,1:], ['red','red'])
    
    ''' 去除主变 '''
    if ZB==1:
        box_Wall = Filter_boxS_XorY_More_Len(box_search, 12)
        box_Wall = Filter_boxS_XandY_noMore_Len(box_Wall, 3)
        box_ZBS, box_Wall_AreaS = Search_ZB_use_Wall(box_search, box_Wall)
        box_ZB_Wall, boxS_ZB_DEL = Trans_ZB_Wall_MixInfo_into_boxS(box_ZBS, box_Wall_AreaS)
        Save_box_ZB_Wall_MixInfo(box_ZBS, box_Wall_AreaS, os.path.join(pathSaveBox,'ZB'))
        
        boxS = Update_Box_Remove(boxS, box_ZB_Wall)
        # boxS_DEL.append(boxS_ZB_DEL)
        pltFrame(box_ZB_Wall[:,1:], ['gray','gray'])
        
    
    ''' 去除龙门架 '''
    # box_Gantry = Get_BOX_from_file(os.path.join(pathRegion, 'Gantry_Feet') , 'box.out')
    # SaveBox(box_Gantry, os.path.join(pathSaveBox,'Gantry_Feet'), 'box.out')            
    # boxS = Update_Box_Remove(boxS, box_Gantry)
    # boxS_DEL.append(box_Gantry)

    ''' 去除电容器'''
    box_DRQ = Filter_boxS_XorY_More_Len(boxS[1], 4.5)
    box_DRQ_Filter = Filter_boxS_XorY_Less_Len(box_DRQ, 10)            
    SaveBox(box_DRQ_Filter, os.path.join(pathSaveBox,'DRQ'), 'box.out')
    
    boxS = Update_Box_Remove(boxS, box_DRQ)
    boxS_DEL.append(box_DRQ)
    pltFrame(box_DRQ_Filter[:,1:], ['gray','gray'])            
    
    ''' 去除电抗器 '''
    box_DKQ = Filter_boxS_XandY_More_Len(boxS[6], 3)
    SaveBox(box_DKQ, os.path.join(pathSaveBox,'DKQ'), 'box.out')
    
    boxS = Update_Box_Remove(boxS, box_DKQ)
    boxS_DEL.append(box_DKQ)
    pltFrame(box_DKQ[:,1:], ['gray','gray'])
    
    
    ''' 去除多相设备'''
    box_Mphase = Filter_boxS_XorY_More_Len(boxS[2], 2.5)            
    box_Mphase = Filter_boxS_XorY_noMore_Len(box_Mphase, 10)
    SaveBox(box_Mphase, os.path.join(pathSaveBox,'Mphase'), 'box.out')
    
    boxS = Update_Box_Remove(boxS, box_Mphase)
    boxS_DEL.append(box_Mphase)
    pltFrame(box_Mphase[:,1:], ['gray','gray'])
    
    
    ''' 去除线架 '''
    box_H24  = megerIdx( boxS[2], boxS[4])
    box_Bracket = Filter_boxS_XorY_More_Len(box_H24, 2.5)   
    SaveBox(box_Bracket, os.path.join(pathSaveBox,'Bracket'), 'box.out')
    
    boxS = Update_Box_Remove(boxS, box_Bracket)
    boxS_DEL.append(box_Bracket)
    pltFrame(box_Bracket[:,1:], ['gray','gray'])
    
    
    ''' 去除龙门架 '''
#             box_Gantry = Get_BOX_from_file(os.path.join(pathRegion, 'Gantry_Feet') , 'box.out')
#             SaveBox(box_Gantry, os.path.join(pathSaveBox,'Gantry_Feet'), 'box.out')
    
#             boxS = Update_Box_Remove(boxS, box_Gantry)
#             boxS_DEL.append(box_Gantry)
# #            pltFrame(box_R[:,1:], ['red','red'])
    
    
    box_H04  = megerIdx( boxS[0], boxS[4])
    box_H042 = megerIdx( box_H04, boxS[2])
    boxS_DEL.append(box_H042)
    pltFrame(box_H042[:,1:], ['gray','gray'])
    
    
    boxS_DEL = np.vstack(boxS_DEL)
    
    data = Load_PCDascii_to_mat(pathfilePCDS_in)
    data = data[np.where(data[:,2]<10)[0],:]

    Save_PCD_use_BoxS(data, boxS_DEL, os.path.join(pathRegion,'result') )
    return

def Cluster_220(pathRegion, pathfilePCDS_in, ZB=0):
    fileSlices = Get_SLICEname_from_Path(pathRegion)
    boxS = Get_Cluster_in_Slices(pathRegion, fileSlices)

    ''' 去龙门架 '''
    # box_R = Get_BOX_from_file(os.path.join(pathRegion, 'Gantry_Feet') , 'box.out')
    # boxS = Update_Box_Remove(boxS, box_R)
    
    ''' 去除母线 '''
    box_R = Remove_BusBar(boxS[6], boxS[7], fil_Len_dw=5, fil_Len_up= 0.2, fil_Up_num=3)
    boxS = Update_Box_Remove(boxS, box_R)
    
    ''' 降噪：将不同高度聚类重新聚类（语义聚类） '''
    box_H02 = megerIdx( boxS[0], boxS[2])
    box_H13 = megerIdx( boxS[1], boxS[3])
    
    box_H0123 = megerIdx(box_H02, box_H13)

    boxS = box_H0123
    pltFrame(boxS[:,1:], ['gray','gray'])
    ''' 三相设备不连续性, 扩展box'''
    for ib, box in enumerate(boxS):
        boxS[ib, 1:] = [box[1]-0.7, box[2]+0.7, box[3]-0.7, box[4]+0.7]
    
    
    # filePts = Get_PTSname_from_Path(pathRegion)
    # data = Get_PCD_from_fileS(pathRegion, filePts)
                
    data = Load_PCDascii_to_mat(pathfilePCDS_in)
    data = data[np.where(data[:,2]<11)[0],:]
    
    Save_PCD_use_BoxS(data, boxS, os.path.join(pathRegion,'result')  )
    return


def Cluster_500(pathRegion, pathfilePCDS_in, ZB=0):
    fileSlices = Get_SLICEname_from_Path(pathRegion)
    boxS = Get_Cluster_in_Slices(pathRegion, fileSlices)

    ''' 去龙门架 '''
    # box_R = Get_BOX_from_file(os.path.join(pathRegion, 'Gantry_Feet') , 'box.out')
    # boxS = Update_Box_Remove(boxS, box_R)
    
    ''' 降噪：将不同高度聚类重新聚类（语义聚类） '''
    box_H02 = megerIdx( boxS[0], boxS[2])
    box_H46 = megerIdx( boxS[4], boxS[6])
    
    box_H0146 = megerIdx(box_H02, box_H46)
#            pltFrame(boxS[1][:,1:], ['b','b'])
#            plt.pause(.1)
    boxS = box_H0146
    pltFrame(boxS[:,1:], ['red','red'])
    ''' 三相设备不连续性, 扩展box'''
    for ib, box in enumerate(boxS):
        boxS[ib, 1:] = [box[1]-0.7, box[2]+0.7, box[3]-0.7, box[4]+0.7]
    
#            extrct_pts_Grd('F:/Modeling', boxS, os.path.join(pathRegion,'pcd_rgb_Denoise'), 15.7)
    
    data = Load_PCDascii_to_mat(pathfilePCDS_in)
        
    data = data[np.where(data[:,2]<20)[0],:]
    Save_PCD_use_BoxS(data, boxS, os.path.join(pathRegion,'result')  )
    return

def Cluster_Denoise(pathBase, pathfilePCDS_in, pathRegions_out):
    fileRegions = os.listdir(  pathBase )
    
    for fileRegion in fileRegions:
        fileRegion = fileRegions[0]
        pathRegion = os.path.join(pathBase,fileRegion)
        fileSlices = Get_SLICEname_from_Path(pathRegion)
        boxS = Get_Cluster_in_Slices(pathRegion, fileSlices)
        
        if fileRegion[0] == '3':
            pathSaveBox = os.path.join(pathRegion,'resultBox')
            if not os.path.exists(pathSaveBox):
                os.mkdir(pathSaveBox)
            boxS_DEL = []
            ''' 去除建筑物 '''
            box_H04  = megerIdx( boxS[0], boxS[4])
            box_H042 = megerIdx( box_H04, boxS[2])
            box_Building = Filter_boxS_XandY_More_Len(box_H042, 10)
            box_search = Update_Box_Remove([box_H042], box_Building)[0]
            pltFrame(box_search[:,1:], ['red','red'])
            
            ''' 去除主变 '''
            box_Wall = Filter_boxS_XorY_More_Len(box_search, 12)
            box_ZBS, box_Wall_AreaS = Search_ZB_use_Wall(box_search, box_Wall)
            box_ZB_Wall, boxS_ZB_DEL = Trans_ZB_Wall_MixInfo_into_boxS(box_ZBS, box_Wall_AreaS)
            Save_box_ZB_Wall_MixInfo(box_ZBS, box_Wall_AreaS, os.path.join(pathSaveBox,'ZB'))
            
            boxS = Update_Box_Remove(boxS, box_ZB_Wall)
            boxS_DEL.append(boxS_ZB_DEL)
#            pltFrame(box_ZB_Wall[:,1:], ['gray','gray'])
            
            ''' 去除龙门架 '''
            # box_Gantry = Get_BOX_from_file(os.path.join(pathRegion, 'Gantry_Feet') , 'box.out')
            # SaveBox(box_Gantry, os.path.join(pathSaveBox,'Gantry_Feet'), 'box.out')            
            # boxS = Update_Box_Remove(boxS, box_Gantry)
            # boxS_DEL.append(box_Gantry)

            ''' 去除电容器'''
            box_DRQ = Filter_boxS_XorY_More_Len(boxS[1], 4.5)
            box_DRQ_Filter = Filter_boxS_XorY_Less_Len(box_DRQ, 10)            
            SaveBox(box_DRQ_Filter, os.path.join(pathSaveBox,'DRQ'), 'box.out')
            
            boxS = Update_Box_Remove(boxS, box_DRQ)
            boxS_DEL.append(box_DRQ)
            pltFrame(box_DRQ_Filter[:,1:], ['gray','gray'])
#            pathSave = os.path.join(pathRegion, 'pcd_rgb_Denoise', 'DRQ')
#            data = Get_PTS_from_file('F:\Modeling\pending', '35_Device.pts')
#            data = data[np.where(data[:,2]<10)[0],:] 
#            _ = Save_PCD_use_BoxS(data, box_DRQ, pathSave )
            
#             box_H12 = megerIdx( boxS[1], boxS[2])
# #            pltFrame(box_H12[:,1:], ['b','b'])
# #            plt.pause(.1)
#             data1 = Get_PTS_from_path('F:/Modeling/extract/35kv_001/H1')
#             data2 = Get_PTS_from_path('F:/Modeling/extract/35kv_001/H2')
#             data = np.vstack([data1, data2])
#             data = Get_PCD_in_PCD_use_Boxs(data, box_H12)
#             boxS = Trans_PCD_2_BoxS(data, p_Voxel=0.1, clusterR=1)
            
#             data = Get_PTS_from_file('F:\Modeling\pending', '35_Device.pts')
            
#             data = data[np.where(data[:,2]<10)[0],:] 
#             pathSave = os.path.join(pathRegion, 'pcd_rgb_Denoise')
#             _ = Save_PCD_use_BoxS(data, boxS, pathSave )
            
            
            ''' 去除电抗器 '''
            box_DKQ = Filter_boxS_XandY_More_Len(boxS[6], 3)
            SaveBox(box_DKQ, os.path.join(pathSaveBox,'DKQ'), 'box.out')
            
            boxS = Update_Box_Remove(boxS, box_DKQ)
            boxS_DEL.append(box_DKQ)
            pltFrame(box_DKQ[:,1:], ['gray','gray'])
            
            
            ''' 去除多相设备'''
            box_Mphase = Filter_boxS_XorY_More_Len(boxS[2], 2.5)            
            box_Mphase = Filter_boxS_XorY_noMore_Len(box_Mphase, 10)
            SaveBox(box_Mphase, os.path.join(pathSaveBox,'Mphase'), 'box.out')
            
            boxS = Update_Box_Remove(boxS, box_Mphase)
            boxS_DEL.append(box_Mphase)
            pltFrame(box_Mphase[:,1:], ['gray','gray'])
            
            
            ''' 去除线架 '''
            box_H24  = megerIdx( boxS[2], boxS[4])
            box_Bracket = Filter_boxS_XorY_More_Len(box_H24, 2.5)   
            SaveBox(box_Bracket, os.path.join(pathSaveBox,'Bracket'), 'box.out')
            
            boxS = Update_Box_Remove(boxS, box_Bracket)
            boxS_DEL.append(box_Bracket)
            pltFrame(box_Bracket[:,1:], ['gray','gray'])
            
            
            ''' 去除龙门架 '''
#             box_Gantry = Get_BOX_from_file(os.path.join(pathRegion, 'Gantry_Feet') , 'box.out')
#             SaveBox(box_Gantry, os.path.join(pathSaveBox,'Gantry_Feet'), 'box.out')
            
#             boxS = Update_Box_Remove(boxS, box_Gantry)
#             boxS_DEL.append(box_Gantry)
# #            pltFrame(box_R[:,1:], ['red','red'])
            
            
            box_H04  = megerIdx( boxS[0], boxS[4])
            box_H042 = megerIdx( box_H04, boxS[2])
            boxS_DEL.append(box_H042)
            pltFrame(box_H042[:,1:], ['blue','blue'])
            
            
            data = Get_PTS_from_file(pathfilePCDS_in[0][0])
            data = data[np.where(data[:,2]<10)[0],:]
            Save_PCD_use_BoxS(data, boxS, os.path.join(pathRegions_out[0],'rslt') )
            
            # ''' 更新点云 '''s
            # sliceS = FileNameS_Filter((os.listdir(pathRegion)), 0,'H' )
            # data = Get_PTS_from_path_FileS(pathRegion, sliceS) 
            # data = Update_RemainPCD_use_DelSavBoxs(data, boxS, boxS_DEL, expand_box=0.5)
            # SavePCD(data ,os.path.join(pathRegion, 'Remain'), 'remain.pts')
                        
            
        if fileRegion[0] == '5':            
            ''' 去龙门架 '''
            box_R = Get_BOX_from_file(os.path.join(pathRegion, 'Gantry_Feet') , 'box.out')
            boxS = Update_Box_Remove(boxS, box_R)
            
            ''' 降噪：将不同高度聚类重新聚类（语义聚类） '''
            box_H02 = megerIdx( boxS[0], boxS[2])
            box_H46 = megerIdx( boxS[4], boxS[6])
            
            box_H0146 = megerIdx(box_H02, box_H46)
#            pltFrame(boxS[1][:,1:], ['b','b'])
#            plt.pause(.1)
            boxS = box_H0146
            
            ''' 三相设备不连续性, 扩展box'''
            for ib, box in enumerate(boxS):
                boxS[ib, 1:] = [box[1]-0.7, box[2]+0.7, box[3]-0.7, box[4]+0.7]
            
#            extrct_pts_Grd('F:/Modeling', boxS, os.path.join(pathRegion,'pcd_rgb_Denoise'), 15.7)
            
            filePts = Get_PTSname_from_Path(pathRegion)
            data = Get_PTS_from_fileS(pathRegion, filePts)
                
            pathSave = os.path.join(pathRegion, 'Denoise')
            Save_PCD_use_BoxS(data, boxS, pathSave )
        
        if fileRegion[0] == '2':
            
            ''' 去龙门架 '''
            box_R = Get_BOX_from_file(os.path.join(pathRegion, 'Gantry_Feet') , 'box.out')
            boxS = Update_Box_Remove(boxS, box_R)
            
            ''' 去除母线 '''
            box_R = Remove_BusBar(boxS[6], boxS[7], fil_Len_dw=5, fil_Len_up= 0.2, fil_Up_num=3)
            boxS = Update_Box_Remove(boxS, box_R)
            
            ''' 降噪：将不同高度聚类重新聚类（语义聚类） '''
            box_H02 = megerIdx( boxS[0], boxS[2])
            box_H13 = megerIdx( boxS[1], boxS[3])
            
            box_H0123 = megerIdx(box_H02, box_H13)

            boxS = box_H0123
            ''' 三相设备不连续性, 扩展box'''
            for ib, box in enumerate(boxS):
                boxS[ib, 1:] = [box[1]-0.7, box[2]+0.7, box[3]-0.7, box[4]+0.7]
            
            
            # filePts = Get_PTSname_from_Path(pathRegion)
            # data = Get_PCD_from_fileS(pathRegion, filePts)
                        
            data = Get_PTS_from_file('F:\Modeling\pending', '220_Device.pts')
            data = data[np.where(data[:,2]<11)[0],:]
            
            pathSave = os.path.join(pathRegion, 'pcd_rgb_Denoise')
            Save_PCD_use_BoxS(data, boxS, pathSave )
#            data_2_One = Get_PTS_from_path(pathSave)
#            SavePCD(data, os.path.join(pathRegion, 'Denoise_One'), data_2_One)
            
            
#            box_H23 = megerIdx( boxS[2], boxS[3])
#            pltFrame(boxS[1][:,1:], ['b','b'])
            
#            idxBox_H13_L, idxBox_H13_Remain = class_base_Long(pathBase, box_H13[:,1:], 
#                                                        thresh_Long=10.) 
#            box_H34 = megerIdx( boxS[3], boxS[4])
#            box_H14 = addBoxs( box_H34, box_H13[idxBox_H13_Remain,:])
#           
#            box_H14 = expd_chek_Box(box_H14, delta=.25)
#            idxBox_H14_L, idxBox_H14_Remain = class_base_Long(pathBase, box_H14[:,1:], 
#                                                        thresh_Long=2.5)
#           
##            save_Idx_Box(pathsave_L, box_H13[ idxBox_H13_L,: ])
##            save_Idx_Box(pathsave_M, box_H14[ idxBox_H14_L,: ] )
##            save_Idx_Box(pathsave_S, box_H14[ idxBox_H14_Remain,: ] )
##        
##            extrct_pts_Grd(pathbase, box_H13[ idxBox_H13_L,: ] , pathsave_L,h_Threshold=9)
##            extrct_pts_Grd(pathbase, box_H14[ idxBox_H14_L,: ] , pathsave_M,h_Threshold=9)
##            extrct_pts_Grd(pathbase, box_H14[ idxBox_H14_Remain,: ] , pathsave_S,h_Threshold=9)
        if fileRegion[0] == '1':
            
            ''' 去龙门架 '''
#            box_R = Get_BOX_from_file(os.path.join(pathRegion, 'Gantry_Feet') , 'box.out')
#            boxS = Update_Box_Remove(boxS, box_R)
            
            ''' 去除母线 '''
#            box_R = Remove_BusBar(boxS[6], boxS[7], fil_Len_dw=5, fil_Len_up= 0.2, fil_Up_num=3)
#            boxS = Update_Box_Remove(boxS, box_R)
            
            ''' 降噪：将不同高度聚类重新聚类（语义聚类） '''
            box_H02 = megerIdx( boxS[0], boxS[2])
#            pltFrame(box_H02[:,1:], ['r','r'])
#            box_H13 = megerIdx( boxS[1], boxS[3])
            
            box_H012 = megerIdx(box_H02, boxS[1])

            boxS = box_H012
            ''' 三相设备不连续性, 扩展box'''
            for ib, box in enumerate(boxS):
                boxS[ib, 1:] = [box[1]-0.7, box[2]+0.7, box[3]-0.7, box[4]+0.7]
            
            
            # filePts = Get_PTSname_from_Path(pathRegion)
            # data = Get_PCD_from_fileS(pathRegion, filePts)
            filePCD = os.path.join('F:/Modeling_TJ/preExtract/Cropped_Grd_SOR/AC0110/AC0110_001', 'output_002_Device.pcd')
            data = Load_PCDascii_to_mat(filePCD)
#            data = Get_PTS_from_file()
            
            data = data[np.where(data[:,2]<8)[0],:]
            
            pathSave = os.path.join(pathRegion, 'pcd_rgb_Denoise')
            Save_PCD_use_BoxS(data, boxS, pathSave )
#            data_2_One = Get_PTS_from_path(pathSave)
#            SavePCD(data, os.path.join(pathRegion, 'Denoise_One'), data_2_One)
            
            
#            box_H23 = megerIdx( boxS[2], boxS[3])
#            pltFrame(boxS[1][:,1:], ['b','b'])
            
#            idxBox_H13_L, idxBox_H13_Remain = class_base_Long(pathBase, box_H13[:,1:], 
#                                                        thresh_Long=10.) 
#            box_H34 = megerIdx( boxS[3], boxS[4])
#            box_H14 = addBoxs( box_H34, box_H13[idxBox_H13_Remain,:])
#           
#            box_H14 = expd_chek_Box(box_H14, delta=.25)
#            idxBox_H14_L, idxBox_H14_Remain = class_base_Long(pathBase, box_H14[:,1:], 
#                                                        thresh_Long=2.5)
#           
##            save_Idx_Box(pathsave_L, box_H13[ idxBox_H13_L,: ])
##            save_Idx_Box(pathsave_M, box_H14[ idxBox_H14_L,: ] )
##            save_Idx_Box(pathsave_S, box_H14[ idxBox_H14_Remain,: ] )
##        
##            extrct_pts_Grd(pathbase, box_H13[ idxBox_H13_L,: ] , pathsave_L,h_Threshold=9)
##            extrct_pts_Grd(pathbase, box_H14[ idxBox_H14_L,: ] , pathsave_M,h_Threshold=9)
##            extrct_pts_Grd(pathbase, box_H14[ idxBox_H14_Remain,: ] , pathsave_S,h_Threshold=9)
    return


def Cluster_Refine(pathBase):

    fileRegions = os.listdir(pathBase)
    
    for fileRegion in fileRegions:
        pathRegion = os.path.join(pathBase,fileRegion)
        
        data = Get_PTS_from_path( os.path.join(pathRegion,'Denoise') )
        
        dataTemp= []
        for dat in data:
            if Filter_pcd_with_X_or_Y_Len_Greater(dat, xt=15, yt=15):
                continue
            else:
                dataTemp.append(dat)
        data = np.vstack(dataTemp)
        
        idx_CutHeigh_top = np.where( data[:,2] < 3.5+0.71 )[0]
        data = data[idx_CutHeigh_top,:]
        SavePCD(data, os.path.join(pathRegion,'Core_Area_CutTop'), fileRegion+'.pts')
        
        data = Utilize.subsample_PCL(data, 0.03, 'xyzi')
        
        idx = pcd__filter_PCD_SOR.SOR(data[:,:3], k=10, n_std=1.)
        data = data[idx,:]
        SavePCD(data, os.path.join(pathRegion,'Core_Area_CutTop_SOR'), fileRegion+'_sor.pts')
        
        
        clustersIdx = Cluster_Loc_3D(data, radiusThreshold=0.3)
#        clustersIdx = Cluster_DBSCAN_3D( data, eps=3, minPts=10 )
        dataTemp = []
        for iic, clusterID in enumerate(clustersIdx):
            print(iic)
            dat = data[clusterID]

            if Filter_pcd_with_X_or_Y_or_Z_Len_Less(dat, xt=0.2, yt=0.2, zt=2.5):
                continue
            if Filter_pcd_with_XorY_and_Z_Len_Less(dat, xt=0.4, yt=0.4, zt=1.5):
                continue
            dataTemp.append(dat)
        data = dataTemp
        pathSave = os.path.join(pathRegion,'re_cluster_dist')
#        pathSave = os.path.join(pathRegion,'re_cluster_dbscan')
        SavePCDs(data, pathSave)
        
#        dataTemp = []
#        for iid, dat in enumerate(data_pcd):
#            idx = np.where( dat[:,2]<1.5+0.72 )[0]
#            dataTemp.append(dat[idx,:])
#            savePath = os.path.join(pathRegion,'3_phase_cut_Height','un_connect')
#            saveFile = '{ii:03d}.pts'.format(ii=iic+1)
#            SavePCD(dat[idx,:], savePath, saveFile)
#            
#        data_pcd = Merge_Cluster_4_3phase(dataTemp, distR=1.5)
def Cluster_Refine_35():
    pathRegion = 'F:/Modeling/extract/35kv_001'
    pathRemain = os.path.join(pathRegion,'Remain')
    pathSaveBox = os.path.join(pathRegion,'resultBox')
#    path_H = os.path.join(pathRemain,'H')
    data = Get_PTS_from_path(pathRemain)
    
    data_new = []
    boxS_new = []
    hS_new = []
    
    hT = np.arange(np.min(data[:,2]), np.max(data[:,2]), 0.5)
    for ih in range(len(hT)):
        if ih < len(hT)-1:
            h_low, h_top = hT[ih], hT[ih+1]
        elif ih == len(hT)-1:
            h_low, h_top = hT[ih], np.max(data[:,2])
        dataH = ExtractData_Horizental_Slice(data, h_low, h_top)
        dataH = Utilize.subsample_PCL(dataH, 0.01, 'xyzi')
        idx = pcd__filter_PCD_SOR.SOR(dataH[:,:3], k=10, n_std=1.)
        dataH = dataH[idx,:]
        boxS = Trans_PCD_2_BoxS(dataH, p_Voxel=0.2, clusterR=0.4)
        hh = h_low - np.min(data[:,2])
        
        data_new.append(dataH)
        boxS_new.append(boxS)
        hS_new.append(hh)
        
    data, boxS, hS = data_new, boxS_new, hS_new
    del data_new, boxS_new, hS_new, hT, ih, h_low, h_top, dataH, idx, hh
#        SavePCD(dataH, path_H,'remain_{h:03d}.pts'.format(h=hh))
#        SaveBox(boxS, path_H, 'remain_box_{h:03d}.out'.format(h=hh))
#    
#    fileNameS = FileNameS_Filter(os.listdir(path_H), -4, '.out')
#    boxS = []
#    for fileName in fileNameS:
#        boxS.append(Get_BOX_from_file(path_H, fileName))
        
    ''' 去除电抗器 '''
#    45+50+55电抗器，直接聚类，R=1,  Filter(x_and_y >1)
    data_Cur = Get_PCD_Remain_with_Hs(data, hS, 4.5, 5.5)
    clustersIdx = Cluster_Loc_Lattice(data_Cur, radiusThreshold=0.5)
    bos_DKQ = []
    for clusterID in clustersIdx:
        dat_T = data_Cur[clusterID,:]
        if Filter_pcd_with_X_and_Y_Len_Greater(dat_T, xt=1, yt=1):
            bos_DKQ.append([np.min(dat_T[:,0]), np.max(dat_T[:,0]), 
                            np.min(dat_T[:,1]), np.max(dat_T[:,1]) ] )
    bos_DKQ = np.hstack([np.arange(len(bos_DKQ))[:,np.newaxis], np.vstack(bos_DKQ)])
    bos_DKQ = Expand_boxS(bos_DKQ, 0.5)
    SaveBox(bos_DKQ, os.path.join(pathSaveBox,'DKQ_1'), 'box.out') 
    
    data = Update_PCD_use_DeleteBoxs(data, bos_DKQ)
    pltFrame(bos_DKQ[:,1:], ['m','m'] )
    
    
    ''' 两相支架1 '''
#    30\两相支架  R=1,  Filter(x_or_y >1.5)
    data_Cur = Get_PCD_Remain_with_Hs(data, hS,  3.0, 3.4)
    clustersIdx = Cluster_Loc_Lattice(data_Cur, radiusThreshold=.5)
    box_Bracket= []
    for clusterID in clustersIdx:
        dat_T = data_Cur[clusterID,:]
        if Filter_pcd_with_X_or_Y_Len_Greater(dat_T, xt=1.5, yt=1.5):
            box_Bracket.append([np.min(dat_T[:,0]), np.max(dat_T[:,0]), 
                            np.min(dat_T[:,1]), np.max(dat_T[:,1]) ] )
    box_Bracket = np.hstack([np.arange(len(box_Bracket))[:,np.newaxis], np.vstack(box_Bracket)])
    box_Bracket = Expand_boxS(box_Bracket, 0.5)
    SaveBox(box_Bracket, os.path.join(pathSaveBox,'Bracket_1'), 'box.out') 
    
    data = Update_PCD_use_DeleteBoxs(data, box_Bracket)
    pltFrame(box_Bracket[:,1:], ['green','green'] )

    ''' 两相支架2 '''
#    30\两相支架  R=1,  Filter(x_or_y >1.5)
    data_Cur = Get_PCD_Remain_with_Hs(data, hS,  1.9, 2.4)
    clustersIdx = Cluster_Loc_Lattice(data_Cur, radiusThreshold=.7)
    
    box_Bracket= []
    for clusterID in clustersIdx:
        dat_T = data_Cur[clusterID,:]
        if Filter_pcd_with_X_or_Y_Len_Greater(dat_T, xt=1.2, yt=1.2):
            box_Bracket.append([np.min(dat_T[:,0]), np.max(dat_T[:,0]), 
                            np.min(dat_T[:,1]), np.max(dat_T[:,1]) ] )
    box_Bracket = np.hstack([np.arange(len(box_Bracket))[:,np.newaxis], np.vstack(box_Bracket)])
    box_Bracket = Expand_boxS(box_Bracket, 0.5)
    SaveBox(box_Bracket, os.path.join(pathSaveBox,'Bracket_2'), 'box.out') 
    
    
    data = Update_PCD_use_DeleteBoxs(data, box_Bracket)
    pltFrame(box_Bracket[:,1:], ['blue','blue'] )
    ''' 单相支架 '''
#    14\30单个支架
    p_Voxel=.2
    data_Cur1 = Get_PCD_Remain_with_Hs(data, hS,  1.4, 1.8)
    box1 = Trans_PCD_2_BoxS(data_Cur1, p_Voxel, clusterR = 2*p_Voxel)
    data_Cur2 = Get_PCD_Remain_with_Hs(data, hS,  3.0, 3.4)
    box2 = Trans_PCD_2_BoxS(data_Cur2, p_Voxel, clusterR = 2*p_Voxel)
    box_Single = megerIdx(box1, box2)

    box_Single = Expand_boxS(box_Single, 0.5)
    SaveBox(box_Single, os.path.join(pathSaveBox,'Single_1'), 'box.out') 
    
    
    data = Update_PCD_use_DeleteBoxs(data, box_Single)
    pltFrame(box_Single[:,1:], ['red','red'])
    
def Cluster_Refine_220():
    pathRegion = 'F:/Modeling/extract/220kv_001'
    pathRst = os.path.join(pathRegion,'rst')
    data = Get_PTS_from_path(os.path.join(pathRegion,'re_cluster_dist'))
    data = np.vstack(data)
    
    data_new = []
    boxS_new = []
    hS_new = []
    
    hT = np.arange(np.min(data[:,2]), np.max(data[:,2]), 0.5)
    for ih in range(len(hT)):
        if ih < len(hT)-1:
            h_low, h_top = hT[ih], hT[ih+1]
        elif ih == len(hT)-1:
            h_low, h_top = hT[ih], np.max(data[:,2])
        dataH = ExtractData_Horizental_Slice(data, h_low, h_top)
        dataH = Utilize.subsample_PCL(dataH, 0.01, 'xyzi')
#        idx = SOR(dataH[:,:3], k=10, n_std=1.)
#        dataH = dataH[idx,:]
        
        boxS = Trans_PCD_2_BoxS(dataH, p_Voxel=0.2, clusterR=0.4)
        hh = h_low - np.min(data[:,2]) 
#        SavePCD(dataH, os.path.join(pathRegion,'remian'),'remian_{hh:03d}.pts'.format(hh=int(10*hh)))
        
        data_new.append(dataH)
        boxS_new.append(boxS)
        hS_new.append(hh) 
    data, boxS, hS = data_new, boxS_new, hS_new
    del data_new, boxS_new, hS_new, hT, ih, h_low, h_top, dataH, hh
    
    if not os.path.exists(pathRst):
        os.mkdir(pathRst)
    data_Org = Get_PTS_from_path(pathRegion)
    idxZ = np.where( data_Org[:,2] < np.min(data_Org[:,2])+13 )[0]
    data_Org = data_Org[idxZ,:]
    
    
    '''提取0.5m-1.8m 长度超过5m的设备（三相设备）, 排除长宽同时大于4m的设备（斜线设备）'''
    data_Cur = Get_PCD_Remain_with_Hs(data, hS, 0.5, 1.8)
    clustersIdx = Cluster_Loc_Lattice(data_Cur, radiusThreshold=1.2)
    plt.scatter(data_Cur[:,0], data_Cur[:,1], s=0.1, c='gray' )
    plt.pause(0.1)
    dat_Device = []
    box_Device = []
    for clusterID in clustersIdx:
        dat_T = data_Cur[clusterID,:]
        if Filter_pcd_with_X_or_Y_Len_Greater(dat_T, xt=5, yt=5):
            if Filter_pcd_with_X_and_Y_Len_Greater(dat_T, xt=4, yt=4):
                continue
            else:
                dat_Device.append(dat_T)
            
            box_Device.append([np.min(dat_T[:,0]), np.max(dat_T[:,0]), 
                               np.min(dat_T[:,1]), np.max(dat_T[:,1]) ] )
    box_Device = np.hstack([np.arange(len(box_Device))[:,np.newaxis], np.vstack(box_Device)])
    box_Device = Expand_boxS(box_Device, 0.5)
    pltFrame(box_Device[:,1:], ['m','m'] )
    plt.pause(0.1)
    data = Update_PCD_use_DeleteBoxs(data, box_Device)
    data_Org = Save_PCD_use_BoxS(data_Org, box_Device, os.path.join(pathRst,'class_1'))
    
    '''提取1.9m-2.4m 长度超过5m的设备, 排除长宽同时大于4.5m的设备, ClusterR = 1.5'''
    data_Cur = Get_PCD_Remain_with_Hs(data, hS, 1.9, 2.4)
    clustersIdx = Cluster_Loc_Lattice(data_Cur, radiusThreshold=1.5)
    plt.scatter(data_Cur[:,0], data_Cur[:,1], s=0.1, c='gray' )
    plt.pause(0.1)
    dat_Device = []
    box_Device = []
    for clusterID in clustersIdx:
        dat_T = data_Cur[clusterID,:]
        if Filter_pcd_with_X_or_Y_Len_Greater(dat_T, xt=5, yt=5):
            if Filter_pcd_with_X_and_Y_Len_Greater(dat_T, xt=4.5, yt=4.5):
                continue
            else:
                dat_Device.append(dat_T)
            
            box_Device.append([np.min(dat_T[:,0]), np.max(dat_T[:,0]), 
                               np.min(dat_T[:,1]), np.max(dat_T[:,1]) ] )
    box_Device = np.hstack([np.arange(len(box_Device))[:,np.newaxis], np.vstack(box_Device)])
    box_Device = Expand_boxS(box_Device, 0.5)
    pltFrame(box_Device[:,1:], ['r','r'] )
    plt.pause(0.1)
    data = Update_PCD_use_DeleteBoxs(data, box_Device)
    data_Org = Save_PCD_use_BoxS(data_Org, box_Device, os.path.join(pathRst,'class_2'))
    
    
    '''提取0.5m-2.4m 长度小于1m的设备, ClusterR = 0.5'''
    data_Cur = Get_PCD_Remain_with_Hs(data, hS, 0.5, 2.4)
    clustersIdx = Cluster_Loc_Lattice(data_Cur, radiusThreshold=0.5)
    plt.scatter(data_Cur[:,0], data_Cur[:,1], s=0.1, c='gray' )
    plt.pause(0.1)
    dat_Device = []
    box_Device = []
    for clusterID in clustersIdx:
        dat_T = data_Cur[clusterID,:]
        if Filter_pcd_with_X_and_Y_Less_Len(dat_T, xt=2, yt=2):
            if Filter_pcd_with_X_or_Y_Less_Len(dat_T, xt=.2, yt=.2):
                continue
            else:
                dat_Device.append(dat_T)
                box_Device.append([np.min(dat_T[:,0]), np.max(dat_T[:,0]), 
                                   np.min(dat_T[:,1]), np.max(dat_T[:,1]) ])
        
    box_Device = np.hstack([np.arange(len(box_Device))[:,np.newaxis], np.vstack(box_Device)])
    box_Device = Expand_boxS(box_Device, 0.5)
    pltFrame(box_Device[:,1:], ['g','g'] )
    plt.pause(0.1)
    data = Update_PCD_use_DeleteBoxs(data, box_Device)
    data_Org = Save_PCD_use_BoxS(data_Org, box_Device, os.path.join(pathRst,'class_3'))
    
    
    '''提取0.5m-2.4m 长度大于5m的设备, ClusterR = 0.5'''
    data_Cur = Get_PCD_Remain_with_Hs(data, hS, 0.5, 2.4)
    clustersIdx = Cluster_Loc_Lattice(data_Cur, radiusThreshold=0.5)
    plt.scatter(data_Cur[:,0], data_Cur[:,1], s=0.1, c='gray' )
    plt.pause(0.1)
    dat_Device = []
    box_Device = []
    for clusterID in clustersIdx:
        dat_T = data_Cur[clusterID,:]
        if Filter_pcd_with_X_and_Y_Len_Greater(dat_T, xt=5, yt=5):
            dat_Device.append(dat_T)
            box_Device.append([np.min(dat_T[:,0]), np.max(dat_T[:,0]), 
                               np.min(dat_T[:,1]), np.max(dat_T[:,1]) ] )
    box_Device = np.hstack([np.arange(len(box_Device))[:,np.newaxis], np.vstack(box_Device)])
    box_Device = Expand_boxS(box_Device, 0.5)
    pltFrame(box_Device[:,1:], ['y','y'] )
    plt.pause(0.1)
    data = Update_PCD_use_DeleteBoxs(data, box_Device)
    data_Org = Save_PCD_use_BoxS(data_Org, box_Device, os.path.join(pathRst,'class_4'))
    
    
    '''提取0.5m-2.4m 长度大于4.5m的设备, ClusterR = 0.5， 检查三相设备是否存在断开'''
    data_Cur = Get_PCD_Remain_with_Hs(data, hS, 0.5, 2.4)
    clustersIdx = Cluster_Loc_Lattice(data_Cur, radiusThreshold=0.5)
    plt.scatter(data_Cur[:,0], data_Cur[:,1], s=0.1, c='blue' )
    plt.pause(0.1)
    dat_Device = []
    box_Device = []
    for clusterID in clustersIdx:
        dat_T = data_Cur[clusterID,:]
        if Filter_pcd_with_X_or_Y_Len_Greater(dat_T, xt=4.5, yt=4.5):
            dat_Device.append(dat_T)
            box_Device.append([np.min(dat_T[:,0]), np.max(dat_T[:,0]), 
                               np.min(dat_T[:,1]), np.max(dat_T[:,1]) ] )
    box_Device = np.hstack([np.arange(len(box_Device))[:,np.newaxis], np.vstack(box_Device)])
    box_Device = Expand_boxS(box_Device, 0.5)
    pltFrame(box_Device[:,1:], ['k','k'] )
    plt.pause(0.1)
    data = Update_PCD_use_DeleteBoxs(data, box_Device)
    data_Org = Save_PCD_use_BoxS(data_Org, box_Device, os.path.join(pathRst,'class_5'))
    
    return
    
def Extract_Cluster_Device_35():
    pathRegion = 'F:/Modeling/extract/35kv_001'
    path_rst_Box = os.path.join(pathRegion,'resultBox')
    path_rst = os.path.join(pathRegion,'rst')
    if not os.path.exists(path_rst):
        os.mkdir(path_rst)
    
    data = Get_PTS_from_path(pathRegion)
    fileNames = os.listdir(path_rst_Box)
    for fileName in fileNames:
        path_rst_pcd = os.path.join(path_rst,fileName)
        if not os.path.exists(path_rst_pcd):
            os.mkdir(path_rst_pcd)
            
        if fileName[:2] != 'ZB':
            # continue
            pathBox = os.path.join(path_rst_Box,fileName)
            print(pathBox)
            fileBox = os.listdir(pathBox)[0]
            boxS = np.loadtxt( os.path.join(pathBox,fileBox) )
            Save_PCD_use_BoxS(data, boxS, path_rst_pcd)
        elif fileName[:2] == 'ZB':
            pathAreas = os.path.join(path_rst_Box,fileName)
            fileAreaS = os.listdir( pathAreas )
            for i_area, fileArea in enumerate(fileAreaS):
                pathArea = os.path.join(pathAreas,fileArea)
                fileWallS = FileNameS_Filter(os.listdir(pathArea), -4, '.out')[0]
                boxS_wallS = np.loadtxt( os.path.join(pathArea, fileWallS) )
                boxArea = [0, 
                           np.min(boxS_wallS[:,1]), np.max(boxS_wallS[:,2]),
                           np.min(boxS_wallS[:,3]), np.max(boxS_wallS[:,4])]
                if (boxArea[2]-boxArea[1]) > (boxArea[4]-boxArea[3]):
                    boxArea[1], boxArea[2] = boxArea[1]-12, boxArea[2] +12
                elif (boxArea[2]-boxArea[1]) <= (boxArea[4]-boxArea[3]):
                    boxArea[3], boxArea[4] = boxArea[3]-12, boxArea[4] +12
                idx = Get_PCD_idx_use_Box(data, boxArea)
                dataArea = data[idx,:]
                data = np.delete(data, idx, axis=0)
                
                fileZBS = FileNameS_Filter(os.listdir(pathArea), 2, 'ZB')
                for i_zb, fileZB in enumerate(fileZBS):
                    pathZB = os.path.join(pathArea, fileZB)
                    fileZB_box = os.listdir( pathZB )[0]
                    boxS_ZB = np.loadtxt( os.path.join(pathZB, fileZB_box) )
                    dataZB_Temp = []
                    for box_ZB in boxS_ZB:
                        idx = Get_PCD_idx_use_Box(dataArea, box_ZB)
                        dataZB_Temp.append( dataArea[idx] )
                        dataArea = np.delete(dataArea, idx, axis=0)
                    dataZB = np.vstack(dataZB_Temp)
                    filePts = 'ZB__area_{i_area:03d}__devece{i_zb:03d}.pts'.format(i_area=i_area, i_zb=i_zb)
                    SavePCD(dataZB, path_rst_pcd, filePts)
        

def Expand_box(box, expand_H):
    box[1],box[3] = box[1]-expand_H, box[3]-expand_H
    box[2],box[4] = box[2]+expand_H, box[4]+expand_H
    return box

def Expand_boxS(boxS, expand_H):
    for ib, box in enumerate(boxS):
        box[1],box[3] = box[1]-expand_H, box[3]-expand_H
        box[2],box[4] = box[2]+expand_H, box[4]+expand_H
        boxS[ib] = box
    return boxS
    
def Update_PCD_use_DeleteBoxs(data, boxS, expand_box=0.):
    data_New = []
    for dat in data:
        for box in boxS:
            if expand_box>0:
                box = Expand_box(box, expand_H=expand_box)
            idx = Get_PCD_idx_use_Box(dat, box)
            if len(idx) > 0:
                dat = np.delete(dat, idx, axis=0)
        data_New.append(dat)    
    return data_New

def Update_RemainPCD_use_DelSavBoxs(data_HS, box_Save, boxS_Del, expand_box):
    data = np.vstack(data_HS)
    boxS = np.vstack(boxS_Del)
    
    for iid, box in enumerate(boxS):
        print('\r'+':  {ib}/{lenF}'.format(ib=iid+1,lenF=len(boxS)),end='',flush=True)
        box = Expand_box(box, expand_H=expand_box)
        idx = Get_PCD_idx_use_Box(data, box)
        if len(idx) > 0:
            data = np.delete(data, idx, axis=0)
    print('\n')
    
    boxS = np.vstack(box_Save)
    data_new = []
    for iid, box in enumerate(boxS):
        print('\r'+':  {ib}/{lenF}'.format(ib=iid+1,lenF=len(boxS)),end='',flush=True)
        idx = Get_PCD_idx_use_Box(data, box)
        if len(idx) > 0:
            data_new.append(data[idx])
            data = np.delete(data, idx, axis=0)
    print('\n')
    return np.vstack(data_new)  
    
def Get_PCD_Remain_with_Hs(dataS, hS, h_low, h_up):
    hS = np.asarray(hS)
    idx1 = np.where(hS>=h_low)[0]
    idx2 = np.where(hS<=h_up)[0]
    idx = np.intersect1d(idx1,idx2)
    data = []
    for iid in idx:
        data.append(dataS[iid])

    return np.vstack(data)
        
    
def Merge_Cluster_4_3phase(data, distR):
    iib = 0
    while iib < len(data):
        iic = iib+1
        while iic < len(data):
            if np.min( cdist(data[iib][:,:2], data[iic][:,:2]) ) < distR:
                data[iib] = np.vstack([data[iib],data[iic]])
                del(data[iic])
            else:
                iic = iic + 1
        iib = iib+1
    return data
        

    
def Cluster_Refine_500():
    pathRegion = 'F:/Modeling/extract/500kv_001'
    pathRst = os.path.join(pathRegion,'rst')
    data = Get_PTS_from_path(os.path.join(pathRegion,'Denoise'))
    data = np.vstack(data)
    
    data_new = []
    boxS_new = []
    hS_new = []
    hT = np.arange(np.min(data[:,2]), 15, 0.5)
    data = ExtractData_Horizental_Slice(data, np.min(data[:,2]), 15)
    for ih in range(len(hT)):
        if ih < len(hT)-1:
            h_low, h_top = hT[ih], hT[ih+1]
        elif ih == len(hT)-1:
            h_low, h_top = hT[ih], np.max(data[:,2])
        dataH = ExtractData_Horizental_Slice(data, h_low, h_top)
        dataH = Utilize.subsample_PCL(dataH, 0.02, 'xyzi')
        idx = pcd__filter_PCD_SOR.SOR(dataH[:,:3], k=10, n_std=1.)
        dataH = dataH[idx,:]
        
        boxS = Trans_PCD_2_BoxS(dataH, p_Voxel=0.2, clusterR=0.4)
        hh = h_low - np.min(data[:,2]) 
#        SavePCD(dataH, os.path.join(pathRegion,'remian'),'remian_{hh:03d}.pts'.format(hh=int(10*hh)))
        
        data_new.append(dataH)
        boxS_new.append(boxS)
        hS_new.append(hh) 
    data, boxS, hS = data_new, boxS_new, hS_new
    del data_new, boxS_new, hS_new, hT, ih, h_low, h_top, dataH, hh
    
    if not os.path.exists(pathRst):
        os.mkdir(pathRst)
    data_Org = Get_PTS_from_path(pathRegion)
    idxZ = np.where( data_Org[:,2] < np.min(data_Org[:,2])+13 )[0]
    data_Org = data_Org[idxZ,:]
    
    '''提取1.5m-2.0m 长度超过25m的GIS设备 '''
    data_Cur = Get_PCD_Remain_with_Hs(data, hS, 1.5, 2.0)
    clustersIdx = Cluster_Loc_Lattice(data_Cur[:,:2], radiusThreshold=0.5)
#    plt.scatter(data_Cur[:,0], data_Cur[:,1], s=0.1, c='gray' )
#    plt.pause(0.1)
    dat_Device = []
    box_Device = []
    for clusterID in clustersIdx:
        dat_T = data_Cur[clusterID,:]
        if Filter_pcd_with_X_or_Y_Len_Greater(dat_T, xt=25, yt=25):
            dat_Device.append(dat_T)
            box_Device.append([np.min(dat_T[:,0]), np.max(dat_T[:,0]), 
                               np.min(dat_T[:,1]), np.max(dat_T[:,1]) ] )
                    
    box_Device = np.hstack([np.arange(len(box_Device))[:,np.newaxis], np.vstack(box_Device)])
    box_Device = Expand_boxS(box_Device, 1)
    pltFrame(box_Device[:,1:], ['m','m'] )
    plt.pause(0.1)
    data = Update_PCD_use_DeleteBoxs(data, box_Device)
    data_Org = Save_PCD_use_BoxS(data_Org, box_Device, os.path.join(pathRst,'GIS_1'))

    ''' 提取1.5m-2.0m 长度15-25m的GIS设备 '''
    data_Cur = Get_PCD_Remain_with_Hs(data, hS, 1.5, 2.0)
    clustersIdx = Cluster_Loc_Lattice(data_Cur[:,:2], radiusThreshold=0.5)
#    plt.scatter(data_Cur[:,0], data_Cur[:,1], s=0.1, c='gray' )
#    plt.pause(0.1)
    dat_Device = []
    box_Device = []
    for clusterID in clustersIdx:
        dat_T = data_Cur[clusterID,:]
        if Filter_pcd_with_X_or_Y_Len_Greater(dat_T, xt=15, yt=15):
            dat_Device.append(dat_T)
            box_Device.append([np.min(dat_T[:,0]), np.max(dat_T[:,0]), 
                               np.min(dat_T[:,1]), np.max(dat_T[:,1]) ] )
                    
    box_Device = np.hstack([np.arange(len(box_Device))[:,np.newaxis], np.vstack(box_Device)])
    box_Device = Expand_boxS(box_Device, 1)
    pltFrame(box_Device[:,1:], ['m','m'] )
    plt.pause(0.1)
    data = Update_PCD_use_DeleteBoxs(data, box_Device)
    data_Org = Save_PCD_use_BoxS(data_Org, box_Device, os.path.join(pathRst,'GIS_2'))

    ''' 提取1.5m-2.0m 长度超过7m的GIS设备 '''
    data_Cur = Get_PCD_Remain_with_Hs(data, hS, 1.5, 2.0)
    clustersIdx = Cluster_Loc_Lattice(data_Cur[:,:2], radiusThreshold=0.5)
#    plt.scatter(data_Cur[:,0], data_Cur[:,1], s=0.1, c='gray' )
#    plt.pause(0.1)
    dat_Device = []
    box_Device = []
    box_Building = []
    for clusterID in clustersIdx:
        dat_T = data_Cur[clusterID,:]
        if Filter_pcd_with_X_or_Y_Len_Greater(dat_T, xt=7, yt=7):
            if Filter_pcd_with_X_and_Y_Len_Greater(dat_T, xt=6.5, yt=6.5):
                box_Building.append([np.min(dat_T[:,0]), np.max(dat_T[:,0]), 
                                     np.min(dat_T[:,1]), np.max(dat_T[:,1]) ] )
            else:
                dat_Device.append(dat_T)
                box_Device.append([np.min(dat_T[:,0]), np.max(dat_T[:,0]), 
                                   np.min(dat_T[:,1]), np.max(dat_T[:,1]) ] )
                    
    box_Device = np.hstack([np.arange(len(box_Device))[:,np.newaxis], np.vstack(box_Device)])
    box_Device = Expand_boxS(box_Device, 1)
    pltFrame(box_Device[:,1:], ['m','m'] )
    plt.pause(0.1)
    data = Update_PCD_use_DeleteBoxs(data, box_Device)
    if len(box_Building) > 0:
        box_Building = np.hstack([np.arange(len(box_Building))[:,np.newaxis], np.vstack(box_Building)])
        data = Update_PCD_use_DeleteBoxs(data, box_Building)
    data_Org = Save_PCD_use_BoxS(data_Org, box_Device, os.path.join(pathRst,'GIS_3'))
    
    '''提取3.5m-4.8m 长度超过10m的多相设备, ClusterR = 1.2'''
    data_Cur = Get_PCD_Remain_with_Hs(data, hS, 3.5, 4.8)
    clustersIdx = Cluster_Loc_Lattice(data_Cur[:,:2], radiusThreshold=1.2)
#    plt.scatter(data_Cur[:,0], data_Cur[:,1], s=0.1, c='gray' )
#    plt.pause(0.1)
    dat_Device = []
    box_Device = []
    for clusterID in clustersIdx:
        dat_T = data_Cur[clusterID,:]
        if Filter_pcd_with_X_or_Y_Len_Greater(dat_T, xt=10, yt=10):
            dat_Device.append(dat_T)
            box_Device.append([np.min(dat_T[:,0]), np.max(dat_T[:,0]), 
                               np.min(dat_T[:,1]), np.max(dat_T[:,1]) ] )
    box_Device = np.hstack([np.arange(len(box_Device))[:,np.newaxis], np.vstack(box_Device)])
    box_Device = Expand_boxS(box_Device, 1)
    pltFrame(box_Device[:,1:], ['r','r'] )
    plt.pause(0.1)
    data = Update_PCD_use_DeleteBoxs(data, box_Device)
    data_Org = Save_PCD_use_BoxS(data_Org, box_Device, os.path.join(pathRst,'Mphase_1'))
    
    '''提取3.5m-4.8m 长度超过4.5m的多相设备, ClusterR = 1.2'''
    data_Cur = Get_PCD_Remain_with_Hs(data, hS, 3.5, 4.8)
    clustersIdx = Cluster_Loc_Lattice(data_Cur[:,:2], radiusThreshold=1.2)
#    plt.scatter(data_Cur[:,0], data_Cur[:,1], s=0.1, c='gray' )
#    plt.pause(0.1)
    dat_Device = []
    box_Device = []
    for clusterID in clustersIdx:
        dat_T = data_Cur[clusterID,:]
        if Filter_pcd_with_X_or_Y_Len_Greater(dat_T, xt=4.5, yt=4.5):
            dat_Device.append(dat_T)
            box_Device.append([np.min(dat_T[:,0]), np.max(dat_T[:,0]), 
                               np.min(dat_T[:,1]), np.max(dat_T[:,1]) ] )
    box_Device = np.hstack([np.arange(len(box_Device))[:,np.newaxis], np.vstack(box_Device)])
    box_Device = Expand_boxS(box_Device,1)
    pltFrame(box_Device[:,1:], ['r','r'] )
    plt.pause(0.1)
    data = Update_PCD_use_DeleteBoxs(data, box_Device)
    data_Org = Save_PCD_use_BoxS(data_Org, box_Device, os.path.join(pathRst,'Mphase_2'))
    
    '''提取8.5m-9.2m 长度大于3.5m的DLQ设备, ClusterR = 0.5'''
    data_Cur = Get_PCD_Remain_with_Hs(data, hS, 8.5, 9.2)
    clustersIdx = Cluster_Loc_Lattice(data_Cur[:,:2], radiusThreshold=0.5)
#    plt.scatter(data_Cur[:,0], data_Cur[:,1], s=0.1, c='blue' )
#    plt.pause(0.1)
    dat_Device = []
    box_Device = []
    for clusterID in clustersIdx:
        dat_T = data_Cur[clusterID,:]
        if Filter_pcd_with_X_or_Y_Len_Greater(dat_T, xt=3.5, yt=3.5):
                dat_Device.append(dat_T)
                box_Device.append([np.min(dat_T[:,0]), np.max(dat_T[:,0]), 
                                   np.min(dat_T[:,1]), np.max(dat_T[:,1]) ])
        
    box_Device = np.hstack([np.arange(len(box_Device))[:,np.newaxis], np.vstack(box_Device)])
    box_Device = Expand_boxS(box_Device, 1)
    pltFrame(box_Device[:,1:], ['g','g'] )
    plt.pause(0.1)
    data = Update_PCD_use_DeleteBoxs(data, box_Device)
    data_Org = Save_PCD_use_BoxS(data_Org, box_Device, os.path.join(pathRst,'DLQ'))
    
    
    '''提取8.5m-10.4m 长度大于0.5m的设备, ClusterR = 0.5'''
    data_Cur = Get_PCD_Remain_with_Hs(data, hS, 8.5, 10.4)
    clustersIdx = Cluster_Loc_Lattice(data_Cur[:,:2], radiusThreshold=0.4)
    plt.scatter(data_Cur[:,0], data_Cur[:,1], s=0.1, c='blue' )
    plt.pause(0.1)
    dat_Device = []
    box_Device = []
    for clusterID in clustersIdx:
        dat_T = data_Cur[clusterID,:]
        if Filter_pcd_with_X_and_Y_Len_Greater(dat_T, xt=0.5, yt=0.5):
            dat_Device.append(dat_T)
            box_Device.append([np.min(dat_T[:,0]), np.max(dat_T[:,0]), 
                               np.min(dat_T[:,1]), np.max(dat_T[:,1]) ] )
    box_Device = np.hstack([np.arange(len(box_Device))[:,np.newaxis], np.vstack(box_Device)])
    box_Device = Expand_boxS(box_Device, 0.5)
    pltFrame(box_Device[:,1:], ['y','y'] )
    plt.pause(0.1)
    data = Update_PCD_use_DeleteBoxs(data, box_Device)
    data_Org = Save_PCD_use_BoxS(data_Org, box_Device, os.path.join(pathRst,'single'))
    
    return


def Seg_2_Slices(pathRegions_in, pathfilePCDS_in, pathRegions_out, path_PCL):
    
    # hTs = [ [[35 ],[0,1],[1,2],[2,3],[3,4],[4,5], [5,6], [6,7], [7,8]], 
    #         [[220],[0,1],[1,2],[2,3],[3,4],[4,5], [5,6], [6,7], [7,8]], 
    #         [[500],[0,1],[1,2],[2,3],[3,4],[4,5], [5,6], [6,7], [7,8]]]

    hT = [[0,1],[1,2],[2,3],[3,4],[4,5], [5,6], [6,7], [7,8]]
    print('Feature reduction processing........ \n')

    for iregion, pathRegion in enumerate(pathRegions_in):
        print('\n  Process the {ii}/{size} Region Data \n'.format(ii=iregion+1, size=len(pathfilePCDS_in)))

#        fileRegion = fileRegions[1]
        # filePCDS = Utilize.Get_PCDname_from_Path(pathRegion)       
        
        for jj, pathfilePCD in enumerate(pathfilePCDS_in[iregion]):
            # print('        Process the {ii}/{size} PCD \n'.format(ii=jj+1, size=len(pathfilePCDS[ii])))
            
            pathInput, PCDName = os.path.split(pathfilePCD)
            PCDNameTemp = PCDName[:-4]+'Temp'+'.pcd'
            Utilize.Trans_Pcd_binary_2_ascii(pathInput, pathInput, PCDName, PCDNameTemp, path_PCL)
            os.remove(os.path.join(pathInput, PCDName))
            os.rename( os.path.join(pathInput, PCDNameTemp), os.path.join(pathInput, PCDName))
            data = IO_PCD.Load_PCDascii_to_mat(pathfilePCD)
            groundTop = Picking_SurfElevation(data)
            # for h_temp in hTs:
            #     if filePCD[0] == str(h_temp[0][0])[0]:
            #         hT = h_temp[1:]
            #         break
            for ih, hd in enumerate(hT):
                print('        Process the {ii}/{size} SLice \n'.format(ii=ih+1, size=len(hT)))
                h_low, h_top = hd[0]+groundTop, hd[1]+groundTop
                
                dataH = ExtractData_Horizental_Slice(data, h_low, h_top)
                dataH = Utilize.subsample_PCL(dataH, 0.01, 'xyzi')
                idx = pcd__filter_PCD_SOR.SOR(dataH[:,:3], k=20, n_std=1.)
                dataH = dataH[idx,:]
                SavePCD(dataH, os.path.join(pathRegions_out[iregion],'H'+str(ih)), 'H'+str(ih)+'.pts')
    return
def Picking_SurfElevation(data):
    zs = data[:,2]
    hists, bin_edges = np.histogram(zs, bins=int((np.max(zs)-np.min(zs))/0.03))
    num_mean = np.mean(hists)
    for ii, hist in enumerate(hists):
        if hist > num_mean:
            break
    groundTop = (bin_edges[ii]+bin_edges[ii+1])/2

    plt.plot((bin_edges[:-1]+bin_edges[1:])/2, hists)
    plt.plot(np.ones((100,1))*groundTop, np.linspace(np.min(hists),np.max(hists),100))
    plt.pause(.1)
    
    return groundTop
if  __name__ == '__main__':

    # pathBase, _ = pcd_preproc.basic_Info()
#    pathBase = Basic_Info.path_current()
    pathBase = 'F:/Modeling_TJ'
    pathExtract = os.path.join(pathBase,'extract')
#    pathRegions_in = ['C:/Modeling/preExtract/Cropped/35',
#                   'C:/Modeling/preExtract/Cropped/220',
#                   'C:/Modeling/preExtract/Cropped/500']
#    pathfilePCDS_in = [['C:/Modeling/preExtract/Cropped_Grd_SOR/35/35_Device_ascii.pcd'],
#                    ['C:/Modeling/preExtract/Cropped_Grd_SOR/220/220_Device_ascii.pcd'],
#                    ['C:/Modeling/preExtract/Cropped_Grd_SOR/500/500_Device_ascii.pcd']]
#    
#    pathRegions_out = ['C:/Modeling/Extract/35',
#                   'C:/Modeling/Extract/220',
#                   'C:/Modeling/Extract/500']
    
    pathRegions_in  = [os.path.join(pathBase,'preExtract','Cropped','AC0110')]
    pathfilePCDS_in = [[os.path.join(pathBase,'preExtract','Cropped_Grd_SOR','AC0110','AC0110_001','output_002_Device.pcd')]]
    
    pathRegions_out = [os.path.join(pathExtract,'110')]
#    # 666s 
#    t0 = time.time()
#    path_PCL = 'F:\code\pcl_release'
#    Seg_2_Slices(pathRegions_in, pathfilePCDS_in, pathRegions_out, path_PCL)
#    print ('SegHeight  time: {time:4.2f}s'.format(time=time.time()-t0)) 

##     350s
#    t0 = time.time()
#    Cluster_in_Slices(pathExtract)
#    print ('segClusters  time: {time:4.2f}s'.format(time=time.time()-t0)) 

#     1000s
    # t0 = time.time()
    # # Cluster_Denoise(pathExtract, pathfilePCDS_in, pathRegions_out)
    # Cluster_35(pathRegions_out[0], pathfilePCDS_in[0][0], ZB=1)
    # print ('35kV segClusters  time: {time:4.2f}s'.format(time=time.time()-t0))

    t0 = time.time()
    Cluster_Denoise(pathExtract, pathfilePCDS_in, pathRegions_out)
    Cluster_220(pathRegions_out[0], pathfilePCDS_in[0][0], ZB=1)
    print ('220kV segClusters  time: {time:4.2f}s'.format(time=time.time()-t0))    
    
    # t0 = time.time()
    # # Cluster_Denoise(pathExtract, pathfilePCDS_in, pathRegions_out)
    # Cluster_220(pathRegions_out[1], pathfilePCDS_in[1][0], ZB=1)
    # print ('220kV segClusters  time: {time:4.2f}s'.format(time=time.time()-t0))
    
    # t0 = time.time()
    # # Cluster_Denoise(pathExtract, pathfilePCDS_in, pathRegions_out)
    # Cluster_500(pathRegions_out[2], pathfilePCDS_in[2][0], ZB=1)
    # print ('500kV segClusters  time: {time:4.2f}s'.format(time=time.time()-t0)) 
    
    # t0 = time.time()
    # # Cluster_Denoise(pathExtract, pathfilePCDS_in, pathRegions_out)
    # Cluster_35(pathRegions_out[0], pathfilePCDS_in[0][0], ZB=1)
    # print ('35kV segClusters  time: {time:4.2f}s'.format(time=time.time()-t0)) 
    
##    170s
#    t0 = time.time()
#    Cluster_Refine(pathExtract)
#    print ('segClusters  time: {time:4.2f}s'.format(time=time.time()-t0)) 
    
#    Cluster_Refine_35()
#    Extract_Cluster_Device_35()
    
#    Cluster_Refine_220()
#    Cluster_Refine_500()