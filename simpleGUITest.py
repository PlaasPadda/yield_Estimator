#!/home/plaaspadda/skripsie/bin/python3
import FreeSimpleGUI as sg

aCount = 0
coorLong = 0
coorLat = 0

colRover = [[sg.Text('Driving UI')],
            [sg.Text('Count:'), sg.Text(aCount,key='aCount')]]

colMap = [[sg.Push(),sg.Text('Coordinates: '),sg.Push()],
          [sg.Text('Longitude: '), sg.Text(coorLong, key='coorLong'), sg.Push(), sg.Text('Latitude: '), sg.Text(coorLat, key='coorLat')],
          [sg.Push(), sg.Text('Orientation: '), sg.Push()],
          [sg.Button('Update')]]

layout = [[sg.Push(),sg.Text('O-Count'),sg.Push()],
          [sg.Image('/home/plaaspadda/Pictures/PythExecute.png',subsample=2),sg.Push(),sg.Column(colRover),sg.Push()],
          [sg.Image('/home/plaaspadda/Pictures/PythExecute.png', subsample=2), sg.Column(colMap)]]

window = sg.Window('O-Count', layout)

while True:

    event, values = window.read()
    
    if (event == sg.WIN_CLOSED):
        break
    
    for i in range(2):
        aCount+=1
        window['aCount'].update(aCount)

window.close()
