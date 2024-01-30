import os
import Utilize
import numpy as np
import IO_PCD
import Basic_Info
import pcd__filter_PCD_SOR

def SOR_Process():
    print('==============================================================')
    print('                     PATH AND PARAMETERS INFORMATION')
    print('==============================================================')
    pathInput, pathOutput, path_PCL, k_means, n_std = Load_GroProc_Info()

    if not os.path.exists(pathOutput):
        os.makedirs(pathOutput)
    
    print('\n Input Data Path: '+ pathInput)
    print('\n Output Data Path: '+ pathOutput)
    print('\n SOR Parameters: --k_means {k_means}  --n_std {n_std}'.format(k_means=k_means, n_std=n_std))
    print('\n')
    print('\n')

    print('==============================================================')
    print('                       SOR PROCESS')
    print('==============================================================')
    file_PCD_Names = Utilize.Get_PCDname_from_Path(pathInput)
    print('\n Total number of PCD files is {size} \n'.format(size=len(file_PCD_Names)))

    for ii, file_PCD_Name in enumerate(file_PCD_Names):
        print('--------------------------------------------------------------')
        print('\n Begin {ii}/{len} PCD SOR Process'.format(ii=ii+1, len=len(file_PCD_Names)))
        inputPCD = os.path.join(pathInput, file_PCD_Name)
        outputPCD = os.path.join(pathOutput, file_PCD_Name)
        sizeThresh = os.path.getsize(inputPCD)/(1024.**3)
        if sizeThresh<3:
            pcd__filter_PCD_SOR.PCD_filter_SOR_PCL(path_PCL, inputPCD, outputPCD, k_means=k_means, n_std=n_std)
        else:
            print('\n Load the PCD')
            data = IO_PCD.Load_PCDascii_to_mat(inputPCD)

            print('***************************************************************')
            print('********       The size if too big, out of memory!      *******')
            print('********  Reduce Processing Efficiency and Reprocessing *******')
            
            print('\n Computing SOR')
            data = SOR_Distributed_PCD(data, k_means=k_means, n_std=n_std )


            if len(data):
                print('********         Saving the Processed PCD              ********')
                IO_PCD.Save_Numpy2pcd(pathOutput, file_PCD_Name, data )
                print('***************************************************************')
        print('\n Finish {ii}/{len} PCD SOR'.format(ii=ii+1, len=len(file_PCD_Names)))
        print('--------------------------------------------------------------')

    return 


def Search_idx_XY(dat_X, dat_Y, x1, x2, y1, y2, x_star, y_star):
    if x_star == 0 and y_star == 0:
        idxR = np.where( (dat_X >= x1) & (dat_X <= x2) & (dat_Y >= y1) & (dat_Y <= y2) )[0]
    elif x_star == 0 and y_star > 0:
        idxR = np.where( (dat_X >= x1) & (dat_X <= x2) & (dat_Y >  y1) & (dat_Y <= y2) )[0]
    elif x_star > 0 and y_star == 0:
        idxR = np.where( (dat_X >  x1) & (dat_X <= x2) & (dat_Y >= y1) & (dat_Y <= y2) )[0]
    elif x_star > 0 and y_star > 0:
        idxR = np.where( (dat_X >  x1) & (dat_X <= x2) & (dat_Y >  y1) & (dat_Y <= y2) )[0]
    return idxR
def SOR_Distributed_PCD(data, k_means=6, n_std=1.0):
    sizePts_Thresh = 2.e7
    nn = int(np.ceil( np.sqrt(len(data)/sizePts_Thresh)))
    
    x_min = np.min(data[:,0])
    x_max = np.max(data[:,0])
    y_min = np.min(data[:,1])
    y_max = np.max(data[:,1])
    
    xx = np.linspace(x_min, x_max, nn+1)
    yy = np.linspace(y_min, y_max, nn+1)
    
    idx_stack = []
    sumsum = 0
    for ix in range(nn):
        for iy in range(nn):
            idx_range = Search_idx_XY(data[:,0], data[:,1], xx[ix], xx[ix+1], yy[iy], yy[iy+1], ix, iy)
            sumsum = sumsum + len(idx_range)
            print('\n    SOR the {ii}/{size} sub-PCD'.format(ii=ix*nn+iy+1, size=nn*nn ) )
            if len(idx_range)>0:
                idx_SOR = pcd__filter_PCD_SOR.SOR( data[idx_range,:3], k=k_means, n_std=n_std )
                idx_stack.append( idx_SOR )

    print('\n Stacking the sub-PCDs')
    idx = np.hstack(idx_stack)
    if len(idx) > 0:
        data = data[idx]
    
    return data

def Load_GroProc_Info():
    pathBase = Basic_Info.path_current()
    pathCropInfo = os.path.join(pathBase,'Temp_do')
    file_GrdProc = Utilize.Get_filename_with_suffix_from_path(pathCropInfo, 'sor')[0]

    path_GrdProc = os.path.join(pathCropInfo,file_GrdProc)
    f = open(path_GrdProc, mode='r')
    lines = f.readlines()
    pathInput = lines[1].splitlines()[0]
    pathOutput = lines[3].splitlines()[0]
    k_means = int(lines[5].splitlines()[0])
    n_std = float(lines[7].splitlines()[0])
    path_PCL = os.path.join(pathBase,'pcl_release_1.9')

    return pathInput, pathOutput, path_PCL, k_means, n_std


if __name__ == "__main__":
    SOR_Process()