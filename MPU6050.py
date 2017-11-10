
import time
import struct
import sys
import numpy as np
import serial
import math

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *



def main():
        
    ser = setupSensors()
    print("Calibrating...")

    ir = readQuat(ser)
    iRotMat = quatToMat( ir )
    yaw = 0
    print("Data flowing...")

    calibrate = 20
    precal = 400

    for i in range(precal):
        ir = readQuat(ser)
        print(".",end='')
    
    print("!")
    print("Done calibrating.")


    scale = (0.1, 0.1, 2)
    elbow = (0,0,1)
    vs = ( ( 1,-1, 1),
           ( 1, 1, 1),
           (-1, 1, 1),
           (-1,-1, 1),
           ( 1,-1, 0),
           ( 1, 1, 0),
           (-1,-1, 0),
           (-1, 1, 0) )
    es = (    (0,1),
              (0,3),
              (0,4),
              (2,1),
              (2,3),
              (2,7),
              (6,3),
              (6,4),
              (6,7),
              (5,1),
              (5,4),
              (5,7) )
    
    


    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    
    gluPerspective(65, (display[0]/display[1]), 0.1, 50.0)
    glTranslatef(0,-1,-7)


    for i in range(calibrate):
        ir = readQuat(ser)
        iRotMat = quatToMat( ir )
        print("|",end='')
        
    yaw = getYaw(iRotMat)
    

    
    print(yaw)
    #print(ir)
    print( '\n'.join([ "[" + ', '.join([("%0.4f" % c).rjust(7) for c in r]) + "]" for r in iRotMat]))
    
    print("Done")

    glRotate(yaw + 180, 0, 1, 0)

    
    
    print("Beginning main loop...")
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glColor3f(1,1,1)
        
        #glPushMatrix()
        glPushMatrix()
        
        rotMat = quatToMat(readQuat(ser))
        glMultMatrixf(rotMat)
        glPushMatrix()
        glScalef(scale[0],scale[1], scale[2])
        renderCube(vs, es)
        glPopMatrix()

        glTranslatef(scale[0] * elbow[0], scale[1] * elbow[1], scale[2] *  elbow[2])        
        glMultMatrixf(invertRotMat(rotMat))
        glRotatef(-90,1,0,0)
        glPushMatrix()
        glScalef(scale[0],scale[1],scale[2])
        renderCube(vs, es)
        glPopMatrix()

        
        glTranslatef(scale[0] * elbow[0], scale[1] * elbow[1], scale[2] * elbow[2])
        glRotatef(90,1,0,0)
        glMultMatrixf(rotMat)
        glPushMatrix()
        glScalef(scale[0],scale[1],scale[2])
        renderCube(vs, es)
        glPopMatrix()
    
        glPopMatrix()


        
        glPushMatrix()
        glScalef(0.1 ,0.1 ,2)
        glColor3f(1,0,1)
        renderCube(vs, es)
        glPopMatrix()

        glPushMatrix()
        glRotate(-getYaw(rotMat), 0, 1, 0)
        glScalef(0.1 ,0.1 ,2)
        glColor3f(1,0,0)
        renderCube(vs, es)
        glPopMatrix()




        
        #glPopMatrix()

        
        pygame.display.flip()
        pygame.time.wait(5)


def invertRotMat(mat):
    return [row[:] for row in transpose(mat)]

def transpose(mat):
    m = [row[:] for row in mat]
    return [[m[j][i] for j in range(len(m))] for i in range(len(m[0]))]


def quatToMat(vec4):
    d = vec4[0]
    a = vec4[1]
    b = vec4[2]
    c = vec4[3]

    mat = [[ a*a+b*b-c*c-d*d,     2*b*c-2*a*d,     2*b*d+2*a*c, 0],
          [      2*b*c+2*a*d, a*a-b*b+c*c-d*d,     2*c*d-2*a*b, 0],
          [      2*b*d-2*a*c,     2*c*d+2*a*b, a*a-b*b-c*c+d*d, 0],
          [                0,               0,               0, 1]]
    #return transpose(mat)
    return align(mat)



def getYaw( mat ):
    w = [mat[2][i] for i in range(3)]    
    return math.degrees(math.acos( w[2] / math.sqrt( w[0]*w[0]  + w[2]*w[2] ) )) * (-w[0] / abs(w[0]))
    
    



def align(mat):
    negCol(0,mat)
    negCol(2,mat)
    swapCol(0,2,mat)
    swapCol(1,2,mat)
    return mat

def negCol(c, mat):
    for i in range(0, len(mat)):
        mat[i][c] = -mat[i][c]
    return mat

def swapCol(c1, c2, mat):
    for i in range(0,len(mat)):
        temp = mat[i][c1]
        mat[i][c1] = mat[i][c2]
        mat[i][c2] = temp
    return mat


def setupSensors():
    port = 'COM4'
    print("Setting up serial connection to: " + port)
    ser = serial.Serial( port , 115200,timeout=0.1)
    return ser


def readQuat(serialConnection):
    sline = ""
    while (len(sline) != 33 or sline[0] != ord('0')):
        sline = serialConnection.readline()[:33]
    fData = bytearray()
    for i in range(4):
        for j in range(4):
            k = 2 * (4 * i + j) + 1
            fData.append( (sline[k] & 15 ) | ((sline[k + 1] & 15) << 4) )
    return struct.unpack('4f', fData)

def renderCube(vertices, edges):
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

main()


