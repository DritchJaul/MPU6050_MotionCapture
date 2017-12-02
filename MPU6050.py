
import time
import struct
import sys
#import numpy as np
import serial
import math

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *



def main():
        
    ser = setupSensors('COM6')
    print("Calibrating...")

    #
    # [ arm1,
    #   arm2,
    #  [pot,   0,0,0],
    #  [button,0,0,0] ]
    mpuData = readSerialRigData(ser)


    print("Data flowing...")

    calibrate = 20
    precal = 100

    for i in range(precal):
        mpuData = readSerialRigData(ser)
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

    fs = (  (7,5,4,6),
            (2,1,5,7),
            (3,0,1,2),
            (6,4,0,3),
            (4,5,1,0),
            (2,7,6,3) )




    
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    glEnable(GL_DEPTH_TEST)
    gluPerspective(65, (display[0]/display[1]), 0.1, 50.0)
    glTranslatef(0,-1,-7)
    

    mpu1yaw = 0
    mpu2yaw = 0
    ijoint = 0;

    for i in range(calibrate):
        mpuData = readSerialRigData(ser)
        print("|",end='')
        
    mpu1yaw = getYaw(quatToMat( mpuData[0] ))
    mpu2yaw = getYaw(quatToMat( mpuData[1] ))
    ijoint = mpuData[2][0]    

    #print(yaw)
    #print(ir)
    #print( '\n'.join([ "[" + ', '.join([("%0.4f" % c).rjust(7) for c in r]) + "]" for r in iRotMat]))
    
    print("Done")

    glRotate(mpu1yaw + 180, 0, 1, 0)

    print("Beginning main loop...")
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        glPushMatrix()

        mpuData = readSerialRigData(ser)
        mpu1Mat = quatToMat(mpuData[0])
        mpu2Mat = quatToMat(mpuData[1])
        joint = mpuData[2][0]
        button = mpuData[3][0]

        glMultMatrixf(mpu1Mat)
        glPushMatrix()
        glScalef(scale[0],scale[1], scale[2])
        glColor3f(1,0,0)
        renderCube(vs, fs)
        renderWireCube(vs, es)
        glPopMatrix()

        glTranslatef(scale[0] * elbow[0], scale[1] * elbow[1], scale[2] * elbow[2])
        glRotate( -(ijoint - joint) / 2.4, 1, 0, 0)
        glPushMatrix()
        glScalef(scale[0],scale[1],scale[2])
        glColor3f(0,1,0)
        renderCube(vs, fs)
        renderWireCube(vs, es)
        glPopMatrix()

        glTranslatef(scale[0] * elbow[0], scale[1] * elbow[1], scale[2] * elbow[2])
        glRotate( (ijoint - joint) / 2, 1, 0, 0)
        glMultMatrixf(transpose(mpu1Mat)
        glRotate( mpu2yaw + 180, 0, 1, 0)
        glMultMatrixf(mpu2Mat)
        randerHand(vs, es, fs, button, (0,0,1))

        glPopMatrix()

        pygame.display.flip()
        pygame.time.wait(5)


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


def setupSensors(com):
    print("Setting up serial connection to: " + com)
    ser = serial.Serial( com , 115200,timeout=0.01)
    return ser


# Button and Pot: 48
# Upper  arm MPU: 49
# Lower  arm MPU: 50
def readSerialRigData(serialConnection):
    arm1 = [-1,0,0,0]
    arm2 = [-1,0,0,0]
    pot = -1
    button = -1
    sline = ""
    serialConnection.flushInput()
    while ((arm1[0] == -1) | (arm2[0] == -1) | (pot == -1) | (button == -1)):
        sline = serialConnection.readline()
        if len(sline) > 0:
            header = sline[0]
            sline = sline[1:-1]
            if ( (header == 49) & (len(sline) == 32)):
                arm1 = decodeData( sline )
            if ((header     == 50) & (len(sline) == 32)):
                arm2 = decodeData( sline )
            if ((header == 48) & (len(sline) == 4)):
                high = ((sline[0] & 15) << 4) | (sline[1] & 15)
                low  = ((sline[2] & 15) << 4) | (sline[3] & 15)
                
                button = (4 & high) >> 2
                pot = low + ((3 & high) << 8)
                
    return [arm1,arm2,[pot,0,0,0],[button,0,0,0]]



def decodeData( line ):
    fData = bytearray()
    for i in range(4):
        for j in range(4):
            k = 2 * (4 * i + j)
            fData.append( (line[k] & 15 ) | ((line[k + 1] & 15) << 4) )
    return struct.unpack('4f', fData)



def renderCube(vertices, faces):
    glBegin(GL_QUADS)
    for face in faces:
        for vertex in face:
            glVertex3fv(vertices[vertex])
    glEnd()
    

def renderWireCube(vertices, edges):
    glColor3f(1,1,1)
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()



def renderWireCubeC(vertices, edges, color):
    glColor3f(color[0],color[1],color[2])
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()


def renderFinger(vertices, edges, faces, button, color, segments, side):
    openAngle = 215
    closeAngle = 150

    angle = openAngle - button * (openAngle - closeAngle)

    hs = (0.1, 0.1, 0.5)
    hj = (0,0,0.9)

    glPushMatrix()
    glRotate( -side * angle/2, 1, 0, 0)

    glPushMatrix()
    glScalef(hs[0],hs[1],hs[2])
    glColor3f(color[0],color[1],color[2])
    renderCube(vertices, faces)
    renderWireCube(vertices, edges)
    glPopMatrix()

    glPushMatrix()
    for i in range(segments):
        glTranslatef(hs[0] * hj[0], hs[1] * hj[1], hs[2] * hj[2])
        glRotate( side * closeAngle / segments * 0.95, 1, 0, 0)
        glPushMatrix()
        glScalef(hs[0],hs[1],hs[2])

        glColor3f(color[0],color[1],color[2])
        renderCube(vertices, faces)
        renderWireCube(vertices, edges)
        glPopMatrix()
        
    glPopMatrix()
    glPopMatrix()

def randerHand(vertices, edges, faces, button, color):
    renderFinger(vertices, edges, faces, button, color, 3, 1)
    renderFinger(vertices, edges, faces, button, color, 3, -1)
    
    
    



main()
