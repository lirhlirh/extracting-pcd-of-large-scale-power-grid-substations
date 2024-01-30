import os
import Utilize
import numpy as np
import Ground_CSF_Filter
import IO_PCD
import Basic_Info
import pcd__filter_PCD_SOR
import pandas as pd
import pcd__filter_PCD_SOR

def Ground_Proc_CSF():
    print('==============================================================')
    print('                PATH INFORMATION')
    print('==============================================================')
    pathInput, pathOutput = Load_GroProc_Info()

    if not os.path.exists(pathOutput):
        os.makedirs(pathOutput)
    
    print('\n  Input Data Path: '+ pathInput)
    print('\n Output Data Path: '+ pathOutput)
    print('\n')
    print('\n')

    print('==============================================================')
    print('                       GROUND PROCESS')
    print('==============================================================')
    file_PCD_Names = Utilize.Get_PCDname_from_Path(pathInput)
    print('\n Total number of PCD files is {size} \n'.format(size=len(file_PCD_Names)))
    data = []
    for ii, file_PCD_Name in enumerate(file_PCD_Names):

        print('\n Load the {ii}/{size} PCD file '.format(ii=ii+1, size=len(file_PCD_Names)))
        dataT = IO_PCD.Load_PCDascii_to_mat( os.path.join(pathInput, file_PCD_Name) )

        data.append(dataT)
    data = np.vstack(data)

    print('\n  Ground Analyzing ')
    _, non_ground  = Ground_CSF_Filter.CSF_Filter_from_matXYZ(data[:,:3])
    # ground, non_ground  = Ground_CSF_Filter.CSF_Filter_from_matXYZ(data[:,:3])

    # print('\n Save Device Data')
    # filePCD_Ground = file_PCD_Name[:-4] + '_Ground.pcd'
    # data_ground = data[ground,:]
    # IO_PCD.Save_Numpy2pcd(pathOutput, filePCD_Ground, data_ground)

    print('\n Save Device Data')
    data_device = data[non_ground,:]
    filePCD_Device = file_PCD_Name[:-4] + '_Device.pcd'
    IO_PCD.Save_Numpy2pcd(pathOutput, filePCD_Device, data_device)
    print('\n')

    return 

def Load_GroProc_Info():
    pathBase = Basic_Info.path_current()
    pathCropInfo = os.path.join(pathBase,'Temp_do')
    file_GrdProc = Utilize.Get_filename_with_suffix_from_path(pathCropInfo, 'grd_proc')[0]

    path_GrdProc = os.path.join(pathCropInfo,file_GrdProc)
    f = open(path_GrdProc, mode='r')
    lines = f.readlines()
    pathInput = lines[1].splitlines()[0]
    pathOutput = lines[3].splitlines()[0]
    return pathInput, pathOutput

if __name__ == "__main__":
    Ground_Proc_CSF()