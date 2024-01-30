# -*- coding: utf-8 -*-
"""
Created on Thu Apr 30 21:16:00 2020

@author: administration
"""
# import extract
import os
import subprocess
import Utilize
import time
import numpy as np
from sklearn.neighbors import KDTree

def PCD_filter_SOR_py():
#    pathRegion = 'F:/Modeling/extract/220kv_001'
    pathRegion = 'F:/Modeling/extract/35kv_001'
    fileName = 'pcd_rgb_Denoise' 
    data = Utilize.Get_PCD_from_path( os.path.join(pathRegion,fileName,'DRQ') )
    data_SOR = []
    for dat in data:
        idx = SOR(dat[:,:3], k=6, n_std=1.)
        if len(idx)>0:
            data_SOR.append(dat[idx])
    Utilize.SavePCDs( data_SOR, os.path.join(pathRegion,fileName+'_SOR','DRQ') )
    
    return

def SOR(data, k=6, n_std=1.): 
    print('Total number of Point cloud is {len:4.2f} million'.format(len=len(data)/1e6))
    t0 = time.time()
    print('KD_Tree control ...')
    tree_KD = KDTree(data)
    print('Analyzing  Distance ...')
    distanceS, _ = tree_KD.query(data , k=k)
    print('\n SOR time: {tt:4.2f}s'.format(tt=time.time()-t0))
    d_avg_local  = np.mean(distanceS, axis=1) 
    d_avg_global = np.mean(d_avg_local)
    d_max =  d_avg_global+ n_std*np.std (d_avg_local)
    idx_save = np.where( d_avg_local < d_max )[0]			
    return idx_save

def PCD_filter_SOR_PCL(path_PCL, inputPCD, outputPCD, k_means=6.0, n_std=1.0):
    if path_PCL[-3:] == '1.9':
        cmds_EXE = os.path.join(path_PCL, 'pcl_outlier_removal.exe')
    else:
        cmds_EXE = os.path.join(path_PCL, 'pcl_outlier_removal_release.exe')
    cmds_SOR = '-method statistical -mean_k '+str(k_means)+' -std_dev_mul '+str(n_std)
    cmds = cmds_EXE + ' ' + cmds_SOR + ' ' + inputPCD + ' ' +outputPCD\
    
    subprocess.call(cmds)
    
    return

def PCD_filter_SOR_PCL_Files2Files(pathBase_Dat, fileScript, pathInput, pathOutput, k_means=6.0, n_std=1.0):
    path_PCL = os.path.join(pathBase_Dat, fileScript, 'pcl_release_1.9')
    filePtsS = Utilize.Get_PCDname_from_Path(pathInput)
    if not os.path.exists(pathOutput):
        os.mkdir(pathOutput)

    for filePts in filePtsS:
        inputPCD  = os.path.join(pathInput, filePts)
        outputPCD  = os.path.join(pathOutput, filePts)
        PCD_filter_SOR_PCL(path_PCL, inputPCD, outputPCD, k_means=k_means, n_std=n_std)

        
    return

if __name__=='__main__':
    PCD_filter_SOR_py()
    
#    PCD_filter_SOR_PCL_Files2Files('F:/Modeling', 'codeTest',
#                                   'F:/Modeling/extract/35kv_001/Denoise',
#                                   'F:/Modeling/extract/35kv_001/Denoise_SOR')