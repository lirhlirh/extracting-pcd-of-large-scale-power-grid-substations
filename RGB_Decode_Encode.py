# -*- coding: utf-8 -*-
"""
Created on Fri May  8 11:07:44 2020

@author: administration
"""
import numpy as np
import pandas as pd
import os
import subprocess
def RGB_Decode_4_mat_XYZRGB(data):
    xyz = data[:,:3]
    rgb = data[:,3][:,np.newaxis]
    rgb = RGB_Decode(np.asarray(rgb,dtype=np.uint32))
    data = np.hstack([xyz,rgb])
    return data
def RGB_Emcode_4_mat_x_y_z_rgb(data):
    xyz = data[:,:3]
    rgb = data[:,3:]
    rgb = RGB_Encode(rgb)[:,np.newaxis]
    data = np.hstack([xyz,rgb])
    return data
def RGB_Encode(dat_rgb):
    r = np.asarray(dat_rgb[:,0], dtype=np.uint32)
    g = np.asarray(dat_rgb[:,1], dtype=np.uint32)
    b = np.asarray(dat_rgb[:,2], dtype=np.uint32)
    rgb_arr = np.array((r << 16) | (g << 8) | (b << 0), dtype=np.uint32)
    return rgb_arr

def RGB_Decode(rgb_arr):
    rgb_arr.dtype = np.uint32
    r = np.asarray((rgb_arr >> 16) & 255, dtype=np.uint8)
    g = np.asarray((rgb_arr >> 8) & 255, dtype=np.uint8)
    b = np.asarray(rgb_arr & 255, dtype=np.uint8)
    rgb = np.hstack([r,g,b])
    return rgb

def PCD_Header(data_Len):
    headers = '# .PCD v0.7 - Point Cloud Data file format \n' + \
              'VERSION 0.7 \n' + \
              'FIELDS x y z rgb \n' + \
              'SIZE 4 4 4 4 \n' + \
              'TYPE F F F U \n' + \
              'COUNT 1 1 1 1 \n' + \
              'WIDTH '+str(data_Len)+' \n' + \
              'VIEWPOINT 0 0 0 1 0 0 0 \n' + \
              'POINTS '+str(data_Len)+' \n' + \
              'DATA ascii'
    return headers

def Save_Numpy2pcd(pathName, fileName, data):
    if not os.path.exists(pathName):
        os.mkdir(pathName)        
    np.savetxt(os.path.join(pathName,fileName), data, 
               fmt='%4.2f %4.2f %4.2f %10d', header=PCD_Header(len(data)), 
               comments='')
    return
def test_RGB_Decode_Encode():
    pathInput  = 'F:/Modeling/codeTest/RGB_Encode_Decode'  
    fileInput  = '35_MH_Scan_106.pts' 

    os.path.join(pathInput,fileInput)
    data = pd.read_csv(os.path.join(pathInput,fileInput), skiprows=1, sep=' ', header=None).to_numpy()
    dat_XYZ = data[:,:3]
    dat_rgb = data[:,4:]
    dat_rgb = RGB_Encode(dat_rgb)[:,np.newaxis]
    data = np.hstack([dat_XYZ, dat_rgb])
    Save_Numpy2pcd(pathInput, 'pcd_Encode.pcd', data)
    
    dat_rgb = RGB_Decode(dat_rgb)
    data = np.hstack([dat_XYZ, dat_rgb])
#    np.savetxt(os.path.join(pathInput,'pcd_Decode.pts'), data, fmt='%4.2f')
    cmds = 'F:/Modeling/codeTest/RGB_Encode_Decode/pcl_voxel_grid_release.exe ' + \
           'F:/Modeling/codeTest/RGB_Encode_Decode/pcd_Encode.pcd ' + \
           'F:/Modeling/codeTest/RGB_Encode_Decode/pcd_subsample.pcd ' + \
           '-leaf 0.01'
    print('start')
    subprocess.call(cmds)
    print('end')
    cmds = 'F:/Modeling/codeTest/RGB_Encode_Decode/pcl_convert_pcd_ascii_binary_release.exe ' + \
           'F:/Modeling/codeTest/RGB_Encode_Decode/pcd_subsample.pcd ' + \
           'F:/Modeling/codeTest/RGB_Encode_Decode/pcd_subsample_ascii.pcd ' + \
           '0'
    subprocess.call(cmds)
    print('end2')
    return

if __name__ == '__main__':
    test_RGB_Decode_Encode()
    
    
    

    