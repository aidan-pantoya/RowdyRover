import PySimpleGUI as sg

layout = [[sg.Text("Test")], [sg.Button("Button")]]

window = sg.Window("Test", layout)

while True:
    event, values = window.read()
    if event == "Button" or event == sg.WIN_CLOSED:
        break

window.close()

#sg.Window(title="Test", layout=[[]], margins=(100,50)).read()
