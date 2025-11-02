#!/home/plaaspadda/skripsie/bin/python3

import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Rectangle
import FreeSimpleGUI as sg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import random

TREEAMOUNTX = 1 
TREEAMOUNTY = 1 

TREEDISTANCE = 3 # y distance between trees in a row (in meters)
HALFTREEAREA = 0.8 # half the length of a tree block (in meters)
ROADWIDTH = 2 # width of space between tree blocks (in meters)
ROVERWIDTH = 1 # width of rover (in meters)

LIGHTGREEN_THRESH = 100
YELLOW_THRESH = 60
ORANGE_THRESH = 30
RED_THRESH = 10

class Boom():
    def __init__(self, TLx, TLy, BRx, BRy):
        self.topLeft = (TLx, TLy)
        self.bottomRight = (BRx, BRy)  

        center = 0.5 * (TLy - BRy)

        self.treeLeftPos = (TLx, BRy+center) 
        #self.treeLeftCount = 0
        #self.treeLeftCount = random.randint(0, 150)
        self.treeLeftCount = 80 


        self.treeRightpos = (BRx, BRy+center)
        #self.treeRightCount = 0
        #self.treeRightCount = random.randint(0, 150)
        self.treeRightCount = 80 

    def inArea(self, RoverX, RoverY):
        if (self.topLeft[0] < RoverX) and (RoverX < self.bottomRight[0]):
            if (self.bottomRight[1] < RoverY) and (RoverY < self.topLeft[1]):
                return True
        
        return False

    def updateCount(self, Count, Heading):
        # Under assumption that camera points right
        if (90 < Heading) and (Heading < 270):    # ONTHOU OM ARDUINO SEND PYTHON RECEIVE TE VERANDER VIR HEADING
            self.treeLeftCount = self.treeLeftCount + Count  
            # ONTHOU OM APPLECOUNTER N RETURN COUNT IN TE SIT IN DETECTOBJECTS FUNCTION (maak n areaCount, UPDATE               aCount en return areacount)
        else:
            self.treeRightCount = self.treeRightCount + Count

    def givePlots(self):
        return self.treeLeftPos, self.treeRightpos

    def returnColours(self):
        if (self.treeLeftCount >= LIGHTGREEN_THRESH):
            leftColour = "forestgreen"

        elif ((self.treeLeftCount >= YELLOW_THRESH) and (self.treeLeftCount < LIGHTGREEN_THRESH)):
            leftColour = "lawngreen"

        elif ((self.treeLeftCount >= ORANGE_THRESH) and (self.treeLeftCount < YELLOW_THRESH)):
            leftColour = "yellow"

        elif ((self.treeLeftCount >= RED_THRESH) and (self.treeLeftCount < ORANGE_THRESH)):
            leftColour = "orange"

        elif ((self.treeLeftCount > 0) and (self.treeLeftCount < RED_THRESH)):
            leftColour = "red"

        elif (self.treeLeftCount == 0):
            leftColour = "darkseagreen"

        if (self.treeRightCount >= LIGHTGREEN_THRESH):
            rightColour = "forestgreen"

        elif ((self.treeRightCount >= YELLOW_THRESH) and (self.treeRightCount < LIGHTGREEN_THRESH)):
            rightColour = "lawngreen"

        elif ((self.treeRightCount >= ORANGE_THRESH) and (self.treeRightCount < YELLOW_THRESH)):
            rightColour = "yellow"

        elif ((self.treeRightCount >= RED_THRESH) and (self.treeRightCount < ORANGE_THRESH)):
            rightColour = "orange"

        elif ((self.treeRightCount > 0) and (self.treeRightCount < RED_THRESH)):
            rightColour = "red"

        elif (self.treeRightCount == 0):
            rightColour = "darkseagreen"

        return leftColour, rightColour

if __name__=='__main__':
    # set up area parameters
    tree_list = []
    topleftx = (-HALFTREEAREA - 0.5*ROVERWIDTH) - (ROADWIDTH + (2*HALFTREEAREA))
    toplefty = (TREEDISTANCE) - TREEDISTANCE
    bottomrightx = topleftx + (ROADWIDTH + (2*HALFTREEAREA))    
    bottomrighty = 0 - TREEDISTANCE

    topleftyBASE = toplefty
    bottomrightyBASE = bottomrighty

    for i in range(TREEAMOUNTX):
        topleftx = topleftx + (ROADWIDTH + (2*HALFTREEAREA))
        bottomrightx = bottomrightx + (ROADWIDTH + (2*HALFTREEAREA))
        for j in range(TREEAMOUNTY):
            toplefty = toplefty + TREEDISTANCE
            bottomrighty = bottomrighty + TREEDISTANCE 
            
            # add boom object to list of trees
            tree_list.append(Boom(TLx=topleftx, TLy=toplefty, BRx=bottomrightx, BRy=bottomrighty))

            if (j == TREEAMOUNTY-1):
                toplefty = topleftyBASE
                bottomrighty = bottomrightyBASE

    # set up plot parameters 
    fig, ax = plt.subplots()
    radius = 0.8 * (HALFTREEAREA) 
    
    for i in range(TREEAMOUNTX*TREEAMOUNTY):
        treeLeftPos, treeRightpos = tree_list[i].givePlots()

        bottomLeft = (treeLeftPos[0]+HALFTREEAREA, treeLeftPos[1]-(0.5*TREEDISTANCE))
        road = Rectangle(bottomLeft, ROADWIDTH, TREEDISTANCE, facecolor="navajowhite", edgecolor="navajowhite")
        ax.add_patch(road)

        leftColour, rightColour = tree_list[i].returnColours()

        leftWedge = Wedge(treeLeftPos, radius, 270, 90, fc=leftColour, ec=leftColour)
        ax.add_patch(leftWedge)

        rightWedge = Wedge(treeRightpos, radius, 90, 270, fc=rightColour, ec=rightColour)
        ax.add_patch(rightWedge)

    # Set axis limits so wedges are visible
    ax.set_xlim(-2, 5)
    ax.set_ylim(-1, 5)
    ax.set_aspect('equal')
    ax.set_facecolor("darkseagreen")

    plot = plt.gcf()

    layout = [[sg.Push(),sg.Canvas(size=(1000,1000), key='-CANVAS-'), sg.Push(), sg.Image('/home/plaaspadda/Pictures/Legend.png')]]
    window = sg.Window('CanvasTest', layout, finalize=True, element_justification='center')
    canvas = window['-CANVAS-'].TKCanvas

    figure_canvas_agg = FigureCanvasTkAgg(plot, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    print('Done')

    while True:
        event,value = window.read(timeout=1)
        if event == sg.WIN_CLOSED:
            break

