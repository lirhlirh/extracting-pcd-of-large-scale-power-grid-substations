# -*- coding: utf-8 -*-
"""
Created on Fri May  8 13:58:45 2020

@author: administration
"""
import os
import numpy as np
import pandas as pd
import Utilize
# from Utilize import Get_PCD_idx_use_Box

def PCD_Header(data_Len):
    headers = '# .PCD v0.7 - Point Cloud Data file format \n' + \
              'VERSION 0.7 \n' + \
              'FIELDS x y z rgb \n' + \
              'SIZE 4 4 4 4 \n' + \
              'TYPE F F F U \n' + \
              'COUNT 1 1 1 1 \n' + \
              'WIDTH '+str(data_Len)+' \n' + \
              'HEIGHT 1' + \
              'VIEWPOINT 0 0 0 1 0 0 0 \n' + \
              'POINTS '+str(data_Len)+' \n' + \
              'DATA ascii'
    return headers

def Save_Numpy2pcd(pathName, fileName, data):
    if not os.path.exists(pathName):
        os.makedirs(pathName)        
    np.savetxt(os.path.join(pathName,fileName), data, 
               fmt='%4.2f %4.2f %4.2f %10d', header=PCD_Header(len(data)), 
               comments='')
    return

def Load_PCDascii_to_mat(fileInput):
#    data = pd.read_csv(fileInput, sep='\\s+', header=None, skiprows=11).to_numpy()
    data = pd.read_csv(fileInput, sep='\\s+', header=None, skiprows=11).values
    return data

def SavePCD(data, pathName, fileName):
    if not os.path.exists(pathName):
        os.makedirs(pathName)
    np.savetxt( os.path.join(pathName,fileName), data, '%4.2f')
    
def SavePCDs(data, pathName):
    if not os.path.exists(pathName):
        os.mkdir(pathName)
    for iid, dat in enumerate(data):
        SavePCD(dat, pathName,  '{ii:03d}.pts'.format(ii=iid+1))
 

def SaveBox(boxS, pathName, fileName):
    if not os.path.exists(pathName):
        os.mkdir(pathName)
    np.savetxt( os.path.join(pathName,fileName), boxS,  fmt='%4.2f' )

    
def Save_PCD_use_BoxS(data, boxS, pathSave):
    for ib, box in enumerate(boxS):
        print('\r'+'Saving the :  {ib}/{lenF} cluster'.format(ib=ib+1,lenF=len(boxS)),end='',flush=True)
        
        idxM = Utilize.Get_PCD_idx_use_Box(data, box)
        if len(idxM) >0:
            filePts = '{ii:03d}.pts'.format(ii=ib+1)
            SavePCD(data[idxM,:], pathSave, filePts)
            data = np.delete(data, idxM, axis=0)
    print('\n')
    return data


def Get_PTS_from_file(pathName, fileName):
    filePCD = os.path.join(pathName, fileName)
#    data = pd.read_csv(filePCD, sep=' ', header=None).to_numpy()
    data = pd.read_csv(filePCD, sep=' ', header=None).values
    return data
 
def Get_PTS_from_fileS(pathName, fileNameS):
    if isinstance(fileNameS, list):
        if len(fileNameS) == 1:
            data = Get_PTS_from_file(pathName, fileNameS[0])
        elif len(fileNameS) > 1:
            data = []
            for fileName in fileNameS:
                filePCD = os.path.join(pathName, fileName)
                data.append( pd.read_csv(filePCD, sep=' ', header=None).to_numpy() )
    else:
        data = Get_PTS_from_file(pathName, fileNameS[0])
    return data

def Get_PTS_from_path(fileInput):
    fileS_Pts = Utilize.Get_PTSname_from_Path(fileInput)
    if len(fileS_Pts) == 0:
        data=0
    elif len(fileS_Pts) == 1:
        data = Get_PTS_from_file(fileInput, fileS_Pts[0])
    elif len(fileS_Pts) > 1:
        data = Get_PTS_from_fileS(fileInput, fileS_Pts)
    return data

def Get_PTS_from_path_FileS(pathName, fileNames):
    dataS = []
    for fileName in fileNames:
        path_Sub = os.path.join(pathName, fileName)
        dataS.append( Get_PTS_from_path(path_Sub) )
    return dataS
if __name__ == '__main__':
    print()