# -*- coding: utf-8 -*-
"""
Created on Thu Apr 30 22:28:27 2020

@author: administration
"""
import os 
import pandas as pd
import numpy as np
import subprocess

def pcd_Trans_obj2pcd( pathBase ):
    voxel = 0.01
    fileNameS = os.listdir( pathBase )
    fileNameT = []
    for fileName in fileNameS:
        if os.path.isdir( os.path.join(pathBase, fileName) ) :
            fileNameT.append(fileName)
    fileNameS = fileNameT 
    
    cmd_exe = pathBase + '/pcl_mesh_sampling_release.exe'
    for fileName in fileNameS:
        fileModels = os.listdir(os.path.join(pathBase, fileName))
        for fileModel in fileModels:
            f_obj = os.path.join(pathBase, fileName, fileModel)
            f_pcd = f_obj[:-4]+'.pcd'
            cmd = cmd_exe + ' ' + f_obj + ' ' + f_pcd + ' -leaf_size '+ str(voxel) + ' -no_vis_result'
            os.system(cmd)
            # subprocess.call(cmd)
            data = pd.read_csv(f_pcd, header=None, skiprows = 11, sep =' ').to_numpy()
            data = np.hstack([data, np.zeros((len(data),1))])
            f_pts = f_obj[:-4]+'.pts'
            np.savetxt(f_pts, data, fmt='%4.2f')
    return
    
if __name__=='__main__':
    pathBase = 'D:/pcd_register/obj2pcd_MH'
    pathBase = 'C:/Modeling/codeTest/配准型号/test_20200806/model_LIB/500kV_OBJ/obj'
    pcd_Trans_obj2pcd(pathBase)
#    return