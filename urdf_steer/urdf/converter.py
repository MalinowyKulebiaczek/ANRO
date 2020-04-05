#!/usr/bin/python
# -*- coding: utf-8 -*-

import json

from tf.transformations import *

with open('dh_parameters.json', 'r') as file:
    dhJson= json.loads(file.read())

x_axis = (1,0,0)
y_axis = (0,1,0)
z_axis = (0,0,1)

with open('urdf_parameters.yaml', 'w') as file:
    for joint in dhJson:
        name = joint['name']
        a = joint['a']
        d = joint['d']
        alpha = joint['al']
        theta = joint['th']
        
        matrixD= translation_matrix( (0, 0, d) )
        matrixTheta = rotation_matrix( theta, z_axis )
        matrixA = translation_matrix( (a, 0, 0) )
        matrixAlpha = rotation_matrix( alpha, x_axis )
        
        macierz_jednorodna = concatenate_matrices(matrixA,matrixAlpha,matrixTheta, matrixD)
        
        [roll,pitch,yaw] = euler_from_matrix(macierz_jednorodna)
        [x,y,z] = translation_from_matrix(macierz_jednorodna)
        
        file.write(name + ":\n")
        file.write("  j_xyz: "+str(x)+" "+str(y)+" "+str(z)+"\n")
        file.write("  j_rpy: "+str(roll)+' '+str(pitch)+' '+str(yaw)+'\n')
        file.write("  l_xyz: "+str(float(a)*(0.5))+' '+str(0)+' '+str(0)+'\n')
        file.write("  l_len: "+str(a)+'\n')
