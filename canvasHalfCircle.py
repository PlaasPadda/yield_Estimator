#!/home/plaaspadda/skripsie/bin/python3

import matplotlib.pyplot as plt
from matplotlib.patches import Wedge
import FreeSimpleGUI as sg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

fig,ax = plt.subplots()
theta1, theta2 = 0, 180
radius = 0.2 
center =(0.5,0.5)

wedge = Wedge(center, radius, theta1, theta2, fc='green', ec='green')
ax.add_artist(wedge)


wedge = Wedge(center, radius, theta2, theta1, fc='blue', ec='blue')
ax.add_artist(wedge)

plot = plt.gcf()

layout = [[sg.Push(),sg.Canvas(size=(400,400), key='-CANVAS-'), sg.Push()]]

window = sg.Window('CanvasTest', layout, finalize=True, element_justification='center')

canvas = window['-CANVAS-'].TKCanvas

figure_canvas_agg = FigureCanvasTkAgg(plot, canvas)
figure_canvas_agg.draw()
figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)

while True:
    event,value = window.read(timeout=1)
    if event == sg.WIN_CLOSED:
        break

