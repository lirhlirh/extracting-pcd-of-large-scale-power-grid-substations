# -*- coding: utf-8 -*-
"""
Created on Tue Nov 19 14:02:07 2019

@author: administration
"""

import os
import numpy as np
import pandas as pd
import shutil

import time
#import shutil
import Utilize
from RGB_Decode_Encode import RGB_Encode
import IO_PCD
import subprocess
import Basic_Info
from Utilize import subsample_PCL
from pcd__filter_PCD_SOR import PCD_filter_SOR_PCL
import Ground_CSF_Filter
import RGB_Decode_Encode

from pclpy import pcl
import pclpy
# import open3d as o3d

def Voxelization_Ready4Crop(pathBase_Dat, fileScript, volexResl=.1): 

    pathInput  = os.path.join(pathBase_Dat, 'original_Compress', 'Ensemble')
    path_PCL = os.path.join(pathBase_Dat, fileScript, 'pcl_release_1.9')
    
    pathOutput = os.path.join(pathBase_Dat, 'preExtract')
    if not os.path.exists(pathOutput):
        os.mkdir(pathOutput)

    path_Seperated = os.path.join(pathOutput, 'Separated_Temp') 
    Utilize.Voxelization_PCDs(pathInput, path_Seperated, path_PCL, voxelGrid=volexResl)

    path_Merged = os.path.join(pathOutput, 'Merged_Temp')
    Utilize.Ensemble_PCDs(path_Seperated, path_Merged, path_PCL, sizeThresh=5.0)
    
    Utilize.Voxelization_PCDs(path_Merged, path_Merged, path_PCL, voxelGrid=volexResl)

    Rename_4_a_file_with_Resl(path_Merged, volexResl)
    shutil.rmtree( path_Seperated )

def Filter_SOR_Ready4Crop(pathBase_Dat, fileScript, k_means=6, n_std=1.0):
    pathOutput = os.path.join(pathBase_Dat, 'preExtract')
    if not os.path.exists(pathOutput):
        os.mkdir(pathOutput)
    path_PCL = os.path.join(pathBase_Dat, fileScript, 'pcl_release_1.9')

    path_Merged = os.path.join(pathOutput, 'Merged_Temp')
    path_Merged_SOR = os.path.join(pathOutput, 'Whole_PCD_TO_BE_Crop')

    pcdName = Utilize.Get_PCDname_from_Path(path_Merged)[0]
    inputPCD  = os.path.join(path_Merged,pcdName)
    outputPCD = os.path.join(path_Merged_SOR,pcdName)
    if not os.path.exists(path_Merged_SOR):
        os.mkdir(path_Merged_SOR)

    PCD_filter_SOR_PCL(path_PCL, inputPCD, outputPCD, k_means=k_means, n_std=n_std)

    PCD_filter_SOR_PCL(path_PCL, outputPCD, outputPCD, k_means=k_means, n_std=n_std)
    PCD_filter_SOR_PCL(path_PCL, outputPCD, outputPCD, k_means=k_means, n_std=n_std)
    
    Utilize.Trans_Pcd_binary_2_ascii(path_Merged_SOR,path_Merged_SOR,pcdName,pcdName,path_PCL)
    
    os.chdir(pathOutput)
    shutil.rmtree( path_Merged )

def pre_4_unity(pathBase, fileScript):
    volex_Resolution_S = [0.3, 0.2, 0.1, 0.05]
    print('\n')
    print('==============================================================')
    print('\n                     PATH INFORMATION                       ')
    print('==============================================================')
    print('\n')
    print('\n  Input Data Path: '+ os.path.join(pathBase, 'original_Compress', 'Ensemble') )
    print('\n Output Data Path: '+ os.path.join(pathBase, 'preExtract','Whole_PCD_TO_BE_Crop'))
    print('\n Temp Path: '+os.path.join(pathBase, 'preExtract', 'Separated_Temp'))
    print('\n Temp Path: '+os.path.join(pathBase, 'preExtract', 'Merged_Temp'))
    print('\n')
    print('\n')
    for ii, volex_Reslution in enumerate(volex_Resolution_S):
        print('\n')
        print('==============================================================')
        print('\n  Process the {ii}/{size} PCD file \n'.format(ii=ii+1, size=len(volex_Resolution_S)))
        print('==============================================================')

        Voxelization_Ready4Crop(pathBase, fileScript, volexResl= volex_Reslution)
        Filter_SOR_Ready4Crop(pathBase, fileScript, k_means=30, n_std=1.0)
        
        

def Rename_4_a_file_with_Resl(pathInput, resolution):
    fileName_src = Utilize.Get_PCDname_from_Path(pathInput)[0]
    resolution_string = '__res_{suffix:04d}mm'.format(suffix=int(resolution*1000))
    fileName_dst = fileName_src[:-4] + resolution_string + fileName_src[-4:]
    os.rename(os.path.join(pathInput,fileName_src),os.path.join(pathInput,fileName_dst))

if __name__=='__main__':
    pathBase = Basic_Info.path_current()
    # Run Time: 
    t0 = time.time()
    pre_4_unity(pathBase, 'lirh_pack')
    
    # data = CSF_Ready4Crop(pathBase, 'codeTest')
    # Trans_Pts3D_2_Pts2D(pathBase, data, volexResl=0.1)
    print ('Pre-Process Voxel_SOR 4 Crop: {time:4.2f}s'.format(time=time.time()-t0))

    

