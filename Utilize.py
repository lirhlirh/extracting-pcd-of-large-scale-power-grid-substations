# -*- coding: utf-8 -*-
"""
Created on Fri May  8 13:52:32 2020

@author: administration
"""
import numpy as np
from pclpy import pcl
import pclpy
import os
import subprocess
import pandas as pd
import IO_PCD
#import open3d as o3d
from functools import reduce

#def PLY2OBJ():
#    print("Testing IO for meshes ...")
#    mesh = o3d.io.read_triangle_mesh("C:/Modeling/extract/2200.ply")
#    print(mesh)
#    o3d.io.write_triangle_mesh("C:/Modeling/extract/2201.obj",mesh)
#    return

def Trans_PLY_2_PCD(fileInput, fileOutput, path_EXE):
    cmds = path_EXE+'/pcl_ply2pcd.exe' + ' ' + \
           '-format 0 ' + fileInput  + ' '  + fileOutput + ' '
    subprocess.call(cmds)
    return 

def Trans_Pcds_binary_2_ascii(pathInput, pathOutput, path_EXE):
    if not os.path.exists(pathOutput):
        os.mkdir(pathOutput)

    filePCDs = os.listdir(pathInput)
    for ii, filePCD in enumerate(filePCDs):
        print('\n  Trans the {ii}/{size} PCD bin 2 ascii \n'.format(ii=ii+1, size=len(filePCDs)))

        cmds = path_EXE+'/pcl_convert_pcd_ascii_binary.exe' + ' ' + \
               os.path.join(pathInput,filePCD)  + ' '  + \
               os.path.join(pathOutput,filePCD)  + ' '  + \
               '0' 
        subprocess.call(cmds)
    return
def Trans_Pcd_binary_2_ascii(pathInput, pathOutput, fileName_Src, fileName_Dst, path_EXE):
    if not os.path.exists(pathOutput):
        os.mkdir(pathOutput)
    cmds = path_EXE+'/pcl_convert_pcd_ascii_binary_release.exe' + ' ' + \
            os.path.join(pathInput,fileName_Src)  + ' '  + \
            os.path.join(pathOutput,fileName_Dst)  + ' '  + \
            '0' 
    subprocess.call(cmds)
    return
def Voxelization_PCD(fileInput, fileOutput, path_EXE, voxelGrid):
    cmds = path_EXE+'/pcl_voxel_grid.exe' + ' ' + \
            fileInput + ' ' + \
            fileOutput + ' ' + \
            '-leaf '+ str(voxelGrid)
    subprocess.call(cmds)

def Voxelization_PCDs(pathInput, pathOutput, path_EXE, voxelGrid):
    if not os.path.exists(pathOutput):
        os.mkdir(pathOutput)
    file_PCDs = os.listdir(pathInput)
    for ii, file_PCD in enumerate(file_PCDs):
        print('\n  Voxelize the {ii}/{size} PCD file \n'.format(ii=ii+1, size=len(file_PCDs)))
        cmds = path_EXE+'/pcl_voxel_grid.exe' + ' ' + \
               os.path.join(pathInput, file_PCD) + ' ' + \
               os.path.join(pathOutput, file_PCD) + ' ' + \
               '-leaf '+ str(voxelGrid)
        subprocess.call(cmds)
    return  

def Concat_PCDs(pathInput, path_EXE):
    fileNames = Get_PCDname_from_Path( pathInput )
    cmd =  path_EXE + '/pcl_concatenate_points_pcd.exe'
    for fileName in fileNames:
        cmd = cmd + ' ' + os.path.join(pathInput, fileName)
    subprocess.call(cmd)
def Concat_PCDs_np(pathInput, pathOutput, fileName_out):
    fileNames = Get_PCDname_from_Path(pathInput)
    data = []
    for fileName in fileNames:
        filePath = os.path.join(pathInput,fileName)
        data_temp = pd.read_csv( filePath, header=None, sep='\\s+', skiprows=11).to_numpy()
        data.append(data_temp)
    data = np.vstack(data)
    IO_PCD.Save_Numpy2pcd(pathOutput, fileName_out, data)
    return

def Compute_HUll_pcd2vtk(pathInput, pathOutput, path_EXE, filePCD, alphaX = 0):
    fileVTK = filePCD[:-4]+'.vtk'
    cmd = path_EXE + os.path.join(pathInput,filePCD) + ' ' +\
            os.path.join(pathOutput,fileVTK)
    if alphaX > 0:
        cmd = cmd + ' ' + ' -alpha '+ str(alphaX)
    subprocess.call(cmd) 
def Trans_vtk2PCD(pathInput, pathOutput, path_EXE, fileVTK):
    filePCD = fileVTK[:-4]+'.pcd'
    cmds_hull_pcd = path_EXE + os.path.join(pathOutput,fileVTK) + ' ' +\
        os.path.join(pathOutput,filePCD)
    subprocess.call(cmds_hull_pcd) 

def Ensemble_PCDs(pathInput, pathOutput, path_EXE, sizeThresh=1.0):
    if not os.path.exists(pathOutput):
        os.mkdir(pathOutput)
    
    fileSize_Total = Get_Dir_Size(pathInput)/(1024**3)
    sizeThresh = sizeThresh * fileSize_Total / np.floor(fileSize_Total) + 0.01
    file_PCDs = os.listdir(pathInput)
    os.chdir(pathOutput)
    fileSize = 0
    cmds = path_EXE+'/pcl_concatenate_points_pcd.exe'+ ' '
    ii = 0
    for file_PCD in file_PCDs:
        print('\n  Ensemble the {ii}/{size} PCD file \n'.format(ii=ii+1, size=len(file_PCDs)))
        fileSize = fileSize+os.path.getsize( os.path.join(pathInput,file_PCD))/(1024**3)

        if fileSize >= sizeThresh:
            
            subprocess.call(cmds)
            
            name_used = os.path.join(pathOutput,'output.pcd')
            name_new  = os.path.join(pathOutput,'output'+'_{ii:03d}'.format(ii=ii)+'.pcd')
            os.rename(name_used, name_new)
            
            fileSize = 0
            cmds = path_EXE+'/pcl_concatenate_points_pcd.exe'+ ' '
            ii = ii+1
        
        cmds = cmds + os.path.join(pathInput, file_PCD) + ' '
    
    if len(cmds) > len(path_EXE+'/pcl_concatenate_points_pcd.exe'+ ' '):
        subprocess.call(cmds)
        name_used = os.path.join(pathOutput,'output.pcd')
        name_new  = os.path.join(pathOutput,'output'+'_{ii:03d}'.format(ii=ii)+'.pcd')
        os.rename(name_used, name_new)
    return

def Get_Dir_Size(dir):
    size = 0
    for root, _, files in os.walk(dir):
        size += sum([os.path.getsize(os.path.join(root, name)) for name in files])
    return size

def Find_Subfiles_in_File(pathInput):
    files = os.listdir(pathInput)
    filesTmp = []
    for file in files:
        if os.path.isdir(os.path.join(pathInput,file)):
            filesTmp.append(file)
    files = filesTmp
    return files
    
    
def Update_Box_from_newPCD(data, boxH):
    x_min = np.min( data[:,0] )
    x_max = np.max( data[:,0] )
    y_min = np.min( data[:,1] )
    y_max = np.max( data[:,1] )
    boxH = [min(boxH[0],x_min), max(boxH[1],x_max),
            min(boxH[2],y_min), max(boxH[3],y_max)]
    return boxH

def Update_Box3D_from_newPCD(data, boxH):
    x_min = np.min( data[:,0] )
    x_max = np.max( data[:,0] )
    y_min = np.min( data[:,1] )
    y_max = np.max( data[:,1] )
    z_min = np.min( data[:,2] )
    z_max = np.max( data[:,2] )
    boxH = [min(boxH[0],x_min), max(boxH[1],x_max),
            min(boxH[2],y_min), max(boxH[3],y_max),
            min(boxH[4],z_min), max(boxH[5],z_max)]
    return boxH

def subsample_PCL(data, volexResl, typeDat):
    if typeDat=='xyzi':
        data = data[:,:4].astype(np.float32)
        pc = pcl.PointCloud.PointXYZI.from_array( data[:,:4] )
        pcSub   = pclpy.octree_voxel_downsample(pc, resolution=volexResl, 
                                                    centroids=True)
        dataxyz = pcSub.xyz
#        xyz_i  = []
#        for iip, xyz in enumerate(dataxyz):
#            if np.mod(iip,1000) == 0:
#                print('{iip} in {lenA}'.format(iip=iip, lenA=len(dataxyz)))
#                
#            ix = np.where( np.abs(data[:,0]-dataxyz[0,0])<0.011 )[0]
#            iy = np.where( np.abs(data[:,1]-dataxyz[0,1])<0.011 )[0]
#            ii = np.intersect1d(ix,iy)
#            if len(ii) > 2:
#                iz = np.where( np.abs(data[:,2]-dataxyz[0,2])<0.011 )[0]
#                ii = np.intersect1d(ii,iz)
#                if len(ii) > 2:
#                    ii = min(ii)
#            xyz_i.append(ii)
#        data = data[xyz_i,:]
        datai   = pcSub.intensity[:,np.newaxis]
        data    = np.hstack( [dataxyz,datai] )
    elif typeDat == 'xyzrgb':
        
        pc = pcl.PointCloud.PointXYZRGB.from_array (data[:,:3], 
                                                    data[:,4:].astype(np.uint8))
#        pc = pcl.PointCloud.PointXYZRGBA.from_array( dXYZ,dRGB,dlabel)
        pcSub   = pclpy.octree_voxel_downsample(pc, resolution=volexResl, 
                                                    centroids=True)
        
        dataxyz = pcSub.xyz
        datargb = pcSub.rgb
        data    = np.hstack( [dataxyz, datargb] ) 
        
    return data

def Get_PCD_idx_use_Box(data, box):
    id_Xmin = np.where( data[:,0]>box[1] )[0]
    id_Xmax = np.where( data[:,0]<box[2] )[0]
    id_Ymin = np.where( data[:,1]>box[3] )[0]
    id_Ymax = np.where( data[:,1]<box[4] )[0]
    idxM = reduce(np.intersect1d, ([id_Xmin,id_Xmax,id_Ymin,id_Ymax]))
    return idxM

def Get_PCD_in_PCD_use_Boxs(data, boxs):
    dataTmp = []
    for iid, box in enumerate(boxs):
        print('\r'+':  {ib}/{lenF}'.format(ib=iid+1,lenF=len(boxs)),end='',flush=True)
        idxM = Get_PCD_idx_use_Box(data, box)
        if len(idxM) > 0:
            dataTmp.append( data[idxM,:] )
    data = np.vstack(dataTmp)
    print('\n')
    return data

def Get_PTSname_from_Path(pathName):
    filesTemp  = os.listdir(  pathName )
        
    fileNames = []
    for ii, fileTemp in enumerate(filesTemp):
        if fileTemp[-4:] == '.pts':
            fileNames.append(fileTemp)
    if   len(fileNames) == 0:
        filePts = False
    elif len(fileNames) ==  1:    
        filePts = fileNames
    elif len(fileNames) >  1:
        filePts = fileNames
    return filePts

def Get_PCDname_from_Path(pathName):
    filesTemp  = os.listdir(  pathName )
        
    fileNames = []
    for ii, fileTemp in enumerate(filesTemp):
        if fileTemp[-4:] == '.pcd':
            fileNames.append(fileTemp)
    if   len(fileNames) == 0:
        filePts = False
    elif len(fileNames) ==  1:    
        filePts = fileNames
    elif len(fileNames) >  1:
        filePts = fileNames
    return filePts

def Get_PLYname_from_Path(pathName):
    filesTemp  = os.listdir(  pathName )
        
    fileNames = []
    for ii, fileTemp in enumerate(filesTemp):
        if fileTemp[-4:] == '.ply':
            fileNames.append(fileTemp)
    if   len(fileNames) == 0:
        filePts = False
    elif len(fileNames) ==  1:    
        filePts = fileNames
    elif len(fileNames) >  1:
        filePts = fileNames
    return filePts

def Get_filename_with_suffix_from_path(pathInput, suffix):

    ii = len(suffix) 
    fileNames = os.listdir(pathInput)
    fileTemp = []
    for fileName in fileNames:
        if fileName[-ii:] == suffix:
            fileTemp.append(fileName)
    return fileTemp

def Get_VTKname_from_Path(pathName):
    filesTemp  = os.listdir(  pathName )
        
    fileNames = []
    for ii, fileTemp in enumerate(filesTemp):
        if fileTemp[-4:] == '.vtk':
            fileNames.append(fileTemp)
    if   len(fileNames) == 0:
        filePts = False
    elif len(fileNames) ==  1:    
        filePts = fileNames
    elif len(fileNames) >  1:
        filePts = fileNames
    return filePts

if __name__ == '__main__':
    print()