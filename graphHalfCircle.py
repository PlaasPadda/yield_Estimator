#!/home/plaaspadda/skripsie/bin/python3
import FreeSimpleGUI as sg

def draw_semicircle(graph, center_x, center_y, radius, color='black', start_angle=0, extent_angle=180):
    """Draws a semicircle on the graph element.

    Args:
        graph: The PySimpleGUI Graph element.
        center_x: The x-coordinate of the semicircle's center.
        center_y: The y-coordinate of the semicircle's center.
        radius: The radius of the semicircle.
        color: The color of the semicircle (default: black).
        start_angle: The starting angle of the semicircle in degrees (default: 0).
        extent_angle: The extent angle of the semicircle in degrees (default: 180).
    """
    #graph.DrawArc(
    #    (center_x - radius, center_y - radius),
    #    (center_x + radius, center_y + radius),
    #    style='arc',
    #    start_angle=start_angle,
    #    extent=extent_angle,
    #    arc_color ='black'
    #)

    graph.draw_image(filename='/home/plaaspadda/Pictures/GreenHalfCirc.png',location=(center_x,center_y))

layout = [
    [sg.Graph(canvas_size=(400, 400), graph_bottom_left=(0, 0), graph_top_right=(400, 400), key='graph', enable_events=True, drag_submits=True)]
]

window = sg.Window('Semicircle Example', layout, finalize=True)

graph = window['graph']

hemisphere = graph.draw_image(filename='/home/plaaspadda/Pictures/GreenHalfCirc.png',location=(200,200))
while True:
    event, values = window.read(timeout=1)

    if event == sg.WIN_CLOSED:
        break

    if event == 'graph':
        x,y = values['graph']
        window['graph'].relocate_figure(hemisphere, x, y)
window.close()
