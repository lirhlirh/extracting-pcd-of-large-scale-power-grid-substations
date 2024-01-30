import numpy as np
import matplotlib.pyplot as plt

def pltFrame(idxS, clr):
    for ii, a in enumerate(idxS): 
        x = [ a[0], a[1], a[1], a[0], a[0] ]
        y = [ a[2], a[2], a[3], a[3], a[2] ]
        if ii == 0:
            plt.plot(x, y, clr[0] )
        else:
            plt.plot(x, y, clr[1] )  
        plt.pause(.1)
    
def chekCross( a, b):
    zx = abs(a[0] + a[1] -b[0] - b[1])
    x  = abs(a[0] - a[1]) + abs(b[0]  - b[1])
    zy = abs(a[2] + a[3] - b[2] - b[3])
    y  = abs(a[2] - a[3]) + abs(b[2] - b[3])
    if (zx<=x and zy<=y):
        return True
    else:
        return False

def Merge_AdjacentBox(boxS, idxS=0):
    '----------------------'
    'Merge the adjacent boxs into ONE BOX  pointed by the idxS'
    '----------------------'
    cross = True
    while cross:
        cross = False
        ib = 0
#        print('\r'+': {lenF}'.format(lenF=len(boxS)),end='',flush=True)  
        while ib < len(boxS):           
            box_Chk = boxS[ib,:]
            boxSTmp = boxS[ib+1:,:]
            for jj, boxTmp in enumerate(boxSTmp):
                if chekCross( box_Chk, boxTmp):
                    boxS[ib,0] = np.min( [box_Chk[0],boxTmp[0]] )
                    boxS[ib,1] = np.max( [box_Chk[1],boxTmp[1]] )
                    boxS[ib,2] = np.min( [box_Chk[2],boxTmp[2]] )
                    boxS[ib,3] = np.max( [box_Chk[3],boxTmp[3]] )
                    boxS = np.delete(boxS, ib+1+jj, axis=0)
                    cross = True
                    if idxS==0 :
                        break
                    else:
                        for idx_add in idxS[ib+1+jj]:
                            idxS[ib].append(idx_add)
                        idxS.remove(idxS[ib+1+jj])
                        break
            ib = ib+1
#    del cross, ib, box_Chk, boxSTmp, jj, boxTmp, idx_add
    return boxS, idxS



def chekCross_2idxS(idx1,idx2):
    idx01 = []
    idx02 = []
    for idx_a in idx1:
        for idx_b in idx2:
            if chekCross( idx_a[1:], idx_b[1:]):
#                pltFrame([idx_b[1:], idx_a[1:]], ['r','b'])
                idx01.append(idx_a)
                idx02.append(idx_b)
            
    idx01 = np.asarray(idx01)  
    idx02 = np.asarray(idx02)     
    return idx01, idx02

def getMaxBox(a,b):
    a = np.ravel(a)
    b = np.ravel(b)
    c = []
    c.append ( np.min( [a[0],b[0]] ) )
    c.append ( np.max( [a[1],b[1]] ) )
    c.append ( np.min( [a[2],b[2]] ) )
    c.append ( np.max( [a[3],b[3]] ) )
    return np.asarray(c).transpose()

def merge_Box_idx(idx_box):
    idx_a = np.argsort( idx_box[:,0], axis=0 )
    idx_box = idx_box[idx_a]    
    
    mergeDetectorV =  np.where( np.diff( idx_box[:,0] ) == 0 )[0]+1
    mergeDetector = []
    for ii, iv in enumerate(mergeDetectorV):
        if ii == 0:
            mergeDetector.append([iv])
        else:
            if (mergeDetectorV[ii] - mergeDetectorV[ii-1] ) == 1 :
                mergeDetector[len(mergeDetector)-1].append(iv)
            else:
                mergeDetector.append([iv])
                
    for ii, im in enumerate(mergeDetector[::-1]):
        a_max = idx_box[im[0]-1,-4:]
        for imm in im:
             a_max = getMaxBox( a_max, idx_box[ imm,-4:])
        idx_box[ im[0]-1, -4: ] = a_max
        idx_box = np.delete( idx_box, im, axis=0 )
    return idx_box

def chekcross_self(idx00):
    crossB = True
    while crossB:
        crossB = False
        idx00_C = idx00
        for ii in range(len(idx00_C)):
            for jj in  range(len(idx00)):
                if jj != ii:
                    crossB =  chekCross( idx00_C[ii,-4:], idx00[jj,-4:])
                    if crossB:
                        a_max = getMaxBox( idx00_C[ii,-4:], idx00[jj,-4:])
                        idx00[ np.min([ii,jj]), -4:] = a_max
                        idx00 = np.delete( idx00, np.max([ii,jj]), axis=0 )
                        break
            if crossB:
                break
    return idx00

def megerIdx( idx1, idx2):
    
    idx01, idx02 = chekCross_2idxS(idx1,idx2)
    idx00 = np.zeros( (len(idx01),4) )
    for ii in range(len(idx01)):
        idx00[ii,:] =  getMaxBox(idx01[ii,1:],idx02[ii,1:])
#        pltFrame( [idx00[ii,:]], ['r'] )
    idx00 = np.hstack([idx01[:,0][:,np.newaxis],idx02[:,0][:,np.newaxis],idx00])
#    idx00 = np.hstack([idx01[:,0][:,np.newaxis],idx00])

    idx00 = merge_Box_idx(idx00)
    idx00 = merge_Box_idx(idx00[:,1:])
    idx00 = chekcross_self(idx00)
#    for ii in range(len(idx00)):
#        pltFrame( [idx00[ii,-4:]], ['r'] )
            
    idx00 = np.delete(idx00, 0, axis=1)
    idx00Temp = np.reshape( np.arange(len(idx00)), (len(idx00),1) )
    idx00 = np.hstack( [idx00Temp,idx00] )
#    for ii in range(len(idx00)):
#        pltFrame( [idx00[ii,2:]], ['r'] )
    return idx00

def Get_dimXYZ_from_PCD(data, direction):
    x_dim, y_dim, z_dim = 0, 0, 0
    if len(direction) == 3:
        x_dim = np.max(data[:,0]) - np.min(data[:,0])
        y_dim = np.max(data[:,1]) - np.min(data[:,1])
        z_dim = np.max(data[:,2]) - np.min(data[:,2])
    elif len(direction) == 2: 
        if direction == 'xy':
            x_dim = np.max(data[:,0]) - np.min(data[:,0])
            y_dim = np.max(data[:,1]) - np.min(data[:,1])
        if direction == 'yz':
            y_dim = np.max(data[:,1]) - np.min(data[:,1])
            z_dim = np.max(data[:,2]) - np.min(data[:,2])
        if direction == 'xz':
            x_dim = np.max(data[:,0]) - np.min(data[:,0])
            z_dim = np.max(data[:,2]) - np.min(data[:,2])
    elif len(direction) == 1:
        if direction == 'x':
            x_dim = np.max(data[:,0]) - np.min(data[:,0])
        if direction == 'y':
            y_dim = np.max(data[:,1]) - np.min(data[:,1])
        if direction == 'z':
            z_dim = np.max(data[:,2]) - np.min(data[:,2])
    return x_dim, y_dim, z_dim