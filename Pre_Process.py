# -*- coding: utf-8 -*-
"""
Created on Tue Nov 19 14:02:07 2019

@author: administration
"""

import os
import numpy as np
import pandas as pd

import time
#import shutil
import Utilize
from RGB_Decode_Encode import RGB_Encode
import IO_PCD
import subprocess
import Basic_Info
from Utilize import subsample_PCL


def Trans_Pts_XYZARGB_2_Pcd_XYZrgb_in_Origin(pathInput, pathOutput):
    if not os.path.exists(pathOutput):
        os.mkdir(pathOutput)

    boxH = [0,0,0,0,0,0]
    regions = Utilize.Find_Subfiles_in_File(pathInput)
        
    for region in regions:
        pathRegion = os.path.join(pathInput, region)
        file_ptsS = os.listdir(pathRegion)
        for file_pts in file_ptsS:
            file_pcd = file_pts[:-4] + '.pcd'
            print(region,': ',file_pts)
            path_pts = os.path.join(pathRegion, file_pts)
            data = pd.read_csv(path_pts, sep=' ', header=None, skiprows=1).to_numpy()
            
            dat_XYZ = data[:,:3]
            dat_rgb = RGB_Encode(data[:,4:])[:,np.newaxis]
            data = np.hstack([dat_XYZ, dat_rgb])
            
            IO_PCD.Save_Numpy2pcd(pathOutput, file_pcd, data)
            
            boxH = Utilize.Update_Box3D_from_newPCD(data, boxH)
    np.savetxt( os.path.join(pathInput,'boxH.box'), boxH, fmt='%4.2f'  )
    return


def Pres_Process(pathBase_Dat, fileScript, volexResl=.01): 
    print('==============================================================')
    print('                PATH INFORMATION')
    print('==============================================================')

    pathInput  = os.path.join(pathBase_Dat, 'original')
    path_PCL = os.path.join(pathBase_Dat, fileScript, 'pcl_release_1.9')
    
    pathOutput = os.path.join(pathBase_Dat, 'original_Compress')
    if not os.path.exists(pathOutput):
        os.mkdir(pathOutput)
    path_RGBEncode = os.path.join(pathOutput, 'RGB_Encode')
    path_VoxelGrid = os.path.join(pathOutput, 'Voxel_Grid')
    path_Ensemble = os.path.join(pathOutput, 'Ensemble')
    path_Ensemble_ascii = os.path.join(pathOutput, 'Ensemble_ascii')
    
    print('\n  Input Data Path: '+ pathInput)
    print('\n Output Data Path: '+ pathOutput)
    print('\n RGBEncode Data Path: '+path_RGBEncode)
    print('\n VoxelGrid Data Path: '+path_VoxelGrid)
    print('\n path_Ensemble Data Path: '+path_Ensemble)
    print('\n path_Ensemble_ascii Data Path: '+path_Ensemble_ascii)
    print('\n')
    print('\n')

    print('\n')
    print('==============================================================')
    print('                        RGBEncode HEX                         ')
    print('==============================================================')
    print('\n')
    Trans_Pts_XYZARGB_2_Pcd_XYZrgb_in_Origin(pathInput, path_RGBEncode)
    print('\n')
    
    print('\n')
    print('==============================================================')
    print('                        Voxelization                          ')
    print('==============================================================')
    print('\n')
    Utilize.Voxelization_PCDs(path_RGBEncode, path_VoxelGrid, path_PCL, voxelGrid=volexResl)
    print('\n')
    
    print('\n')
    print('==============================================================')
    print('                          Ensemble                            ')
    print('==============================================================')
    print('\n')
    Utilize.Ensemble_PCDs(path_VoxelGrid, path_Ensemble, path_PCL)
    print('\n')

    print('\n')
    print('==============================================================')
    print('                Re-Voxelization  Cross Area                   ')
    print('==============================================================')
    print('\n')
    os.chdir(pathBase_Dat)
    Utilize.Voxelization_PCDs(path_Ensemble, path_Ensemble, path_PCL, voxelGrid=volexResl)
    print('\n')
    
    
    print('\n')
    print('==============================================================')
    print('                        bin -> ascii                          ')
    print('==============================================================')
    print('\n')
    Utilize.Trans_Pcds_binary_2_ascii(path_Ensemble, path_Ensemble_ascii, path_PCL)
    print('\n')

if __name__=='__main__':
    pathBase = Basic_Info.path_current()
    
    # Run Time: 7000s
    t0 = time.time()
    Pres_Process(pathBase, 'lirh_pack', volexResl=.01)
    print ('Pre-Process Compress time: {time:4.2f}s'.format(time=time.time()-t0))
    t0 = time.time()
    

