# -*- coding: utf-8 -*-
"""
Created on Wed Aug  5 13:28:32 2020

@author: senyuan
"""
import os
import Basic_Info
import Utilize
from  Extract_Devices import Seg_2_Slices, Cluster_in_Slices_for_Region
from  Extract_Devices import Cluster_35, Cluster_220, Cluster_500

def Seg_Scene_basic_Resion():
    print('==============================================================')
    print('                PATH INFORMATION about Region and PCDs ')
    print('==============================================================')
    pathRegion, path_PCD_of_Region, vol_lvl, zb, path_PCL = Load_Seg_Info()
    
    print('\n  Input Data Path: '+ path_PCD_of_Region)
    print('\n Output Region Path: '+ pathRegion)
    print('\n Voltage Level is: {vol_lvl}'.format(vol_lvl=vol_lvl))
    print('\n Main Tranformer Search Bool {zb} \n'.format(zb=zb))
    print('\n')
    print('\n')
    
    print('==============================================================')
    print('                Dimension Reduction ')
    print('==============================================================')
    print('\n')
    
    Seg_2_Slices([pathRegion], [[path_PCD_of_Region]], [pathRegion], path_PCL)
    print('\n')
    print('==============================================================')
    print('                Clustering Boxes in 2D')
    print('==============================================================')
    print('\n')
    Cluster_in_Slices_for_Region(pathRegion)
    print('\n')
    print('\n')
    

    print('==============================================================')
    print('                Hierarchical Clustering ')
    print('==============================================================')
    print('\n')
    if vol_lvl==35:
        Cluster_35(pathRegion, path_PCD_of_Region, ZB=zb)
    elif vol_lvl==220:
        Cluster_220(pathRegion, path_PCD_of_Region, ZB=zb)
    elif vol_lvl==500:
        Cluster_500(pathRegion, path_PCD_of_Region, ZB=zb)
    return 

def Load_Seg_Info():
    
    pathBase = Basic_Info.path_current()
    pathSegInfo = os.path.join(pathBase,'Temp_do')
    fileSeg = Utilize.Get_filename_with_suffix_from_path(pathSegInfo, 'seg')[0]

    path_fileSeg = os.path.join(pathSegInfo, fileSeg)
    
    f = open(path_fileSeg, mode='r')
    lines = f.readlines()
    pathRegion = lines[1].splitlines()[0]
    path_PCD_of_Region = lines[3].splitlines()[0]
    vol_lvl = int(lines[5])
    zb = int(lines[7])
    
    path_PCL = os.path.join(pathBase, 'lirh_pack', 'pcl_release_1.9')
    
    # pathRegion = 'C:/Modeling/Extract/35'
    # path_PCD_of_Region = 'C:/Modeling/preExtract/Cropped_Grd_SOR/35/35_Device_ascii.pcd'
    # vol_lvl = 35
    # zb = 1
    
    return pathRegion, path_PCD_of_Region, vol_lvl, zb, path_PCL

if __name__ == '__main__':
    Seg_Scene_basic_Resion()
    
        