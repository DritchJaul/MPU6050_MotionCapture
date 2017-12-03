
import time
import struct
import sys
#import numpy as np
import serial
import math
import os

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *



def main():
    display = (1200,900)
    ser = setupSensors('COM6')
    
    print("Calibrating...")

    #
    # [ arm1,
    #   arm2,
    #  [pot,   0,0,0],
    #  [button,0,0,0] ]
    mpuData = readSerialRigData(ser)

    print("Data flowing...")
    
    calibrate = 40
    precal = 200

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
    
    fs = (  (7,5,4,6),  # (0,0,-1)
            (2,1,5,7),  # (0,1,0)
            (3,0,1,2),  # (0,0,1)
            (6,4,0,3),  # (0,-1,0)
            (4,5,1,0),  # (1,0,0)
            (2,7,6,3) ) # (-1,0,0)

    pygame.init()
    
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    
    gluPerspective(75, (display[0]/display[1]), 0.1, 150.0)

    glEnable(GL_DEPTH_TEST)

    #glColorMaterial ( GL_FRONT_AND_BACK, GL_EMISSION )
    glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE )
    glEnable ( GL_COLOR_MATERIAL ) 
    glEnable(GL_LIGHTING)


    glLightfv(GL_LIGHT0, GL_POSITION,  (1,1,0,0.1))
    glLightfv(GL_LIGHT0, GL_AMBIENT, (0,0,0,1))
    glLightfv(GL_LIGHT0, GL_DIFFUSE, (1,1,1,1))
    glLightfv(GL_LIGHT0, GL_SPECULAR, (1,1,1,1))
    glEnable(GL_LIGHT0)

    
    glTranslatef(0,-2,-4)
    

    objs = [ Shape((-2,0,-4), (0.35,1,0.35), (1,0,1)),
             Shape((0,4,-2), (0.35,1,0.35), (1,1,0)),
             Shape((2,0,-4), (0.35,1,0.35), (0,1,1)) ]

    terrain = [ Shape((0,-2,0), (100,1,100), (0.6,0.6,0.6)),
                Shape((0,-2,-30), (100,100,1), (0.6,0.6,0.6))]

    

    for i in range(calibrate):
        mpuData = readSerialRigData(ser)
        print("|",end='')

    mpu1yaw = getYaw(quatToMat( mpuData[0] ))
    mpu2yaw = getYaw(quatToMat( mpuData[1] ))
    mpu2Init = quatToMat( mpuData[1] )
    ijoint = mpuData[2][0]    
    button = mpuData[3][0]
    buttonLastFrame = 0
    shapeGrabbed = -1


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
        glPopMatrix()

        glTranslatef(scale[0] * elbow[0], scale[1] * elbow[1], scale[2] * elbow[2])
        glRotate( -(ijoint - joint) / 2.4, 1, 0, 0)
        glPushMatrix()
        glScalef(scale[0],scale[1],scale[2])
        glColor3f(0,1,0)
        renderCube(vs, fs)
        glPopMatrix()

        glTranslatef(scale[0] * elbow[0], scale[1] * elbow[1], scale[2] * elbow[2])
        glRotate( (ijoint - joint) / 2, 1, 0, 0)
        glMultMatrixf(transpose(mpu1Mat))
        glRotatef( (-mpu1yaw) - (-getYaw(mpu2Init)) , 0, 1, 0)
        glMultMatrixf(mpu2Mat)
        randerHand(vs, fs, button, (0,0,1))


        glPushMatrix()
        glTranslatef(0,0,0.60)
        glRotate(90,0,0,1)
        endM = glGetFloatv(GL_MODELVIEW_MATRIX)
        endP = getPos(endM)
        if button == 1 and buttonLastFrame == 0:
            closestShape = -1
            for obj in objs:
                if dist(obj.p, endP) < 1:
                    if closestShape == -1:
                        closestShape = obj
                    elif dist(obj.p, endP) < dist(closestShape.p, endP):
                        closestShape = obj
            shapeGrabbed = closestShape
        elif button == 0 and buttonLastFrame == 1:
            shapeGrabbed = -1

        if shapeGrabbed != -1:
            shapeGrabbed.setMatrix(endM)

        buttonLastFrame = button
        glPopMatrix()


        glPopMatrix()

        
        for obj in objs:
            obj.render()

        for obj in terrain:
            obj.render()
        

        pygame.display.flip()
        pygame.time.wait(5)

def getPos( mat ):
    return [ mat[3][0], mat[3][1], mat[3][2] ]


def dist( a, b ):
    return math.sqrt( (a[0]- b[0])**2 + (a[1]- b[1])**2 + (a[2]- b[2])**2)


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



def getCurrentYaw():
    a = glGetFloatv(GL_MODELVIEW_MATRIX)
    return getYaw(a)


    
def align(mat):
    negCol(0,mat)
    negCol(2,mat)
    swapCol(0,2,mat)
    swapCol(1,2,mat)
    return mat


def negRow(r, mat):
    for i in range(len(mat)):
        mat[r][i] = -mat[r][i]
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
    timer = time.time()
    while ((arm1[0] == -1) | (arm2[0] == -1) | (pot == -1) | (button == -1)):
        sline = serialConnection.readline()
        
        if timer + 2 < time.time():
            print( "Arduino not responding." )
            timer = time.time()
        
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

    fn = (  ( 0, 0,-1),
            ( 0, 1, 0),
            ( 0, 0, 1),
            ( 0,-1, 0),
            ( 1, 0, 0),
            (-1, 0, 0) )

    for face in range(len(faces)):
        glNormal3f(fn[face][0], fn[face][1], fn[face][2])
        for vertex in faces[face]:
            glVertex3fv(vertices[vertex])


    #for face in faces:
    #    for vertex in face:
    #        glVertex3fv(vertices[vertex])
    glEnd()
    




def renderFinger(vertices, faces, button, color, segments, side):
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
    glPopMatrix()

    glPushMatrix()
    for i in range(segments):
        glTranslatef(hs[0] * hj[0], hs[1] * hj[1], hs[2] * hj[2])
        glRotate( side * closeAngle / segments * 0.95, 1, 0, 0)
        glPushMatrix()
        glScalef(hs[0],hs[1],hs[2])
        glColor3f(color[0],color[1],color[2])
        renderCube(vertices, faces)
        glPopMatrix()
        
    glPopMatrix()
    glPopMatrix()

def randerHand(vertices, faces, button, color):
    renderFinger(vertices, faces, button, color, 3, 1)
    renderFinger(vertices,  faces, button, color, 3, -1)
    
    
class Shape:
    s = (1,1,1)
    m = [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
    p = (0,0,0)
    c = (1,1,1)

    vs = ( ( 1,-1, 1),   
           ( 1, 1, 1),
           (-1, 1, 1),
           (-1,-1, 1),
           ( 1,-1, -1),
           ( 1, 1, -1),
           (-1,-1, -1),
           (-1, 1, -1) )
    

    fs = (  (7,5,4,6),
            (2,1,5,7),
            (3,0,1,2),
            (6,4,0,3),
            (4,5,1,0),
            (2,7,6,3) )

    def __init__(self, position, scale, color):
        self.s = scale
        self.c = color
        glPushMatrix()
        glTranslatef(position[0], position[1], position[2])
        self.m = glGetFloatv(GL_MODELVIEW_MATRIX)
        self.p = getPos(self.m)
        glPopMatrix()

    def render(self):
        glPushMatrix()
        glLoadMatrixf(self.m)
        glScalef(self.s[0], self.s[1], self.s[2])
        glColor3f(self.c[0], self.c[1], self.c[2])
        renderCube(self.vs, self.fs)
        glPopMatrix()


    def setMatrix(self, mat):
        self.m = mat
        self.p = getPos(self.m)




main()
