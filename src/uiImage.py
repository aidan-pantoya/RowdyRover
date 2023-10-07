#!/usr/bin/python
import PySimpleGUI as sg
import os.path
import subprocess

# First the window layout in 2 columns
sg.theme("Reds")
file_list_column = [
    [
        sg.Text("Image Folder"),
        sg.In(size=(25, 1), enable_events=True, key="-FOLDER-"),
        sg.FolderBrowse(),
    ],
    [
        sg.Listbox(
            values=[], enable_events=True, size=(40, 20),
             key="-FILE LIST-"
        )
    ],
    [
        sg.Button("Take Photo")
    ],
    [
        sg.Button("Camera Control")
    ],
    [
        sg.Button("Close Photo")
    ],
]

# For now will only show the name of the file that was chosen
image_viewer_column = [
    [sg.Text("Choose an image from list on left:")],
    [sg.Text(size=(40, 1), key="-TOUT-")],
    [sg.Image(key="-IMAGE-")],
]

# ----- Full layout -----
layout = [
    [
        sg.Column(file_list_column),
        sg.VSeperator(),
        sg.Column(image_viewer_column),
    ]
]

window = sg.Window("Image Viewer", layout)

# Run the Event Loop
while True:
    event, values = window.read()
    
    if event == "Exit" or event == sg.WIN_CLOSED:
        break
        
    if event == "Close Photo":
        window["-TOUT-"].update("")
        window["-IMAGE-"].update()
    
    if event == "Take Photo":
        if not os.path.exists("takePhoto.out"):
            subprocess.call(["g++", "rs-save-to-disk.cpp",
             "-lrealsense2", "-o", "takePhoto.out"])
        subprocess.call("./takePhoto.out")
    
    if event == "Camera Control":
        sg.Window("Continue in terminal",[ 
        [sg.Text("Continue in terminal")],
        [sg.OK()]]).read(close=True)
        if not os.path.exists("cameraControl.out"):
            subprocess.call(["g++", "rs-sensor-control.cpp", 
                             "-pthread", "-lGL", "-lGLU",
                             "-lglfw", "-lrealsense2", "-o", "cameraControl.out"])
        os.system("gnome-terminal -e")
        output, error = subprocess.Popen(["lxterminal", "-e", "./cameraControl.out"]).communicate()
    
    # Folder name was filled in, make a list of files in the folder
    if event == "-FOLDER-":
        folder = values["-FOLDER-"]
        try:
            # Get list of files in folder
            file_list = os.listdir(folder)
        except:
            file_list = []

        fnames = [
            f
            for f in file_list
            if os.path.isfile(os.path.join(folder, f))
            and f.lower().endswith((".png", ".gif"))
        ]
        window["-FILE LIST-"].update(fnames)
    
    elif event == "-FILE LIST-":  # A file was chosen from the listbox
        try:
            filename = os.path.join(
                values["-FOLDER-"], values["-FILE LIST-"][0]
            )
            window["-TOUT-"].update(filename)
            window["-IMAGE-"].update(filename=filename)
        except:
            pass

window.close()
