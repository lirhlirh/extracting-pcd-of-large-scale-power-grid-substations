# -*- coding: utf-8 -*-
"""
Created on Thu Jun 18 14:33:41 2020

@author: senyuan
"""


import cn_PnPoly2 as cnp
import numpy.f2py as f2py
import numpy

r = f2py.run_main(['-m','cn_PnPoly4','C:/Modeling/codeTest/f_compile/cn_PnPoly_test_f2py.f90'])
r = f2py.run_main(['-m','calc','C:/Modeling/codeTest/f_compile//cn_PnPoly_test_f2py.f90'])


fsource
fsource = '''
      subroutine foo
      print*, "Hello world!"
      end 
'''

numpy.f2py.compile(fsource, modulename='cn_PnPoly5', verbose=0)
cnp.__doc__()


cn_PnPoly2.__doc__()