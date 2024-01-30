# -*- coding: utf-8 -*-
"""
Created on Sun Jun 28 13:55:44 2020

@author: senyuan
"""

import os

def pathTest():
    '''realpath / abspath:
    realpath 指向软连接src
    abspath  指向软连接dst
    realpath 更保险
    '''
    '================================================='
    '                    realpath 用法                '
    '================================================='
    # # 脚本文件的路径
    # path = os.path.realpath(__file__)
    # print(path)
    
    # # 脚本所在目录的路径
    # path = os.path.split(os.path.realpath(__file__))[0]
    # print(path)
    
    '================================================='
    '                    getcwd 用法                '
    '================================================='
    # # 工作目录的路径
    # path = os.getcwd() 
    # print(path)
    
    '================================================='
    '                    dirname 用法                 '
    '================================================='
    # # 脚本所在目录的路径
    # path = os.path.dirname(__file__)
    # print(path)

    # # 脚本所在目录所在目录的路径
    # path = os.path.dirname(os.path.dirname(__file__))
    # print(path)

    '================================================='
    '                    abspath 用法                 '
    '================================================='
    # # 脚本所在目录的绝对路径
    # path = os.path.abspath(os.path.dirname(__file__))
    # print(path)

    # # 脚本所在目录的目录的绝对路径
    # path = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
    # print(path)
    
    '================================================='
    '                    上级目录 用法                 '
    '================================================='

    path = os.path.abspath(os.path.dirname(os.getcwd()))
    print(path)
    
    path = os.path.abspath(os.path.join(os.getcwd(), ".."))
    print(path)
    
    path = os.path.dirname(os.path.split(os.path.realpath(__file__))[0])
    print(path)
    
    
if __name__=='__main__':
    pathTest()