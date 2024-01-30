# -*- coding: utf-8 -*-
"""
Created on Fri May  8 14:25:50 2020

@author: administration
"""
import os

def Basic_Info():
#    pathBase   = os.getcwd()
#    pathBase = 'C:/Modeling' 
    pathBase = 'F:/Modeling_TJ'
    pathOrigin = os.path.join(pathBase,'original')
    fileNames = os.listdir(pathOrigin )
    fileNameNew = []
    for fileName in fileNames:
        if os.path.isdir(os.path.join(pathOrigin,fileName)):
            fileNameNew.append(fileName)
    regions   = fileNameNew
    return pathBase, regions

def path_current():
    pathC = os.path.dirname(os.path.split(os.path.realpath(__file__))[0])
    return pathC

if __name__ == '__main__':
    print()