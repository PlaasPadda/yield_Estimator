#!/home/plaaspadda/skripsie/bin/python3

import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Rectangle
import FreeSimpleGUI as sg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

TREEAMOUNTX = 3
TREEAMOUNTY = 3

TREEDISTANCE = 2 # y distance between trees in a row (in meters)
HALFTREEAREA = 0.5 # half the length of a tree block (in meters)
ROADWIDTH = 1 # width of space between tree blocks (in meters)
ROVERWIDTH = 0.6 # width of rover (in meters)

class Boom():
    def __init__(self, TLx, TLy, BRx, BRy):
        self.topLeft = (TLx, TLy)
        self.bottomRight = (BRx, BRy)  

        center = 0.5 * (TLy - BRy)

        self.treeLeftPos = (TLx, BRy+center) 
        self.treeLeftCount = 0

        self.treeRightpos = (BRx, BRy+center)
        self.treeRightCount = 0

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

        leftWedge = Wedge(treeLeftPos, radius, 270, 90, fc='green', ec='green')
        ax.add_patch(leftWedge)

        rightWedge = Wedge(treeRightpos, radius, 90, 270, fc='blue', ec='blue')
        ax.add_patch(rightWedge)

    # Set axis limits so wedges are visible
    ax.set_xlim(-1, 5.5)
    ax.set_ylim(0, 6)
    ax.set_aspect('equal')
    ax.set_facecolor("lightgreen")

    plot = plt.gcf()

    layout = [[sg.Push(),sg.Canvas(size=(1000,1000), key='-CANVAS-'), sg.Push(), sg.Image('/home/plaaspadda/Pictures/Untitled.png')]]
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

