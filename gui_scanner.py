from threading import Thread
import PySimpleGUI as sg
from os import getcwd
from os.path import join
from online import RealsenseRecorder
from test_realsense import test_main


sg.ChangeLookAndFeel("LightGreen")


def gui_choose_version():
    layout = [
        [sg.T()],
        [sg.Text("Please choose the version you wanna use", font=("Helvetica", 14))],
        [sg.T(size=(1, 1))],
        [
            sg.T(size=(10, 1)),
            sg.Button("online", font=("Helvetica", 13)),
            sg.T(size=(4, 1)),
            sg.Button("offline", font=("Helvetica", 13))
        ]
    ]

    window = sg.Window("Choose version", layout, size=(410, 150))
    event, values = window.read()
    window.Close()
    return event


def gui_ouput(exec_function):
    layout = [
        [sg.Output(size=(88, 40), font=('', 11))]
    ]

    window = sg.Window('Output', layout, font=("Helvetica", 13))
    thread = Thread(target=exec_function)
    thread.start()

    while True:
        event, values = window.read()
        if event == None or event == 'Exit':      
            break    
    window.Close()


def gui_online_version():
    layout = [      
        [sg.T()],
        [
            sg.T(size=(2, 1)),
            sg.Text('Output folder:', size=(11, 1)), 
            sg.Input(size=(43, 1), font=('', 11), default_text=join(getcwd(), "dataset")), 
            sg.FolderBrowse()
        ],
        [sg.T(font=('', 5))],
        [
            sg.T(size=(2, 1)),
            sg.Text('images:'), sg.Input(size=(6, 1), default_text="30"), sg.T(size=(2, 1)),
            sg.Text('voxel size:'), sg.Input(size=(8, 1), default_text="0.0025"),sg.T(size=(2, 1)),
            sg.Text('max depth:'), sg.Input(size=(4, 1), default_text="1.0"), sg.Text('m'),
        ],
        [sg.T(font=('', 5))],
        [
            sg.T(size=(2, 1)),
            sg.Text('Register method:'),
            sg.InputCombo(('point to plane ICP', 'color ICP'), readonly=True, size=(15, 1), 
                          default_value='point to plane ICP',font=('Helvetica', 10)),
            sg.T(size=(8, 1)),
            sg.Checkbox('only scan body', default=True)
        ],
        [sg.T(font=('', 7))],
        [
            sg.T(size=(13, 1)),
            sg.Button('test camera', size=(12, 1)),
            sg.T(size=(8, 1)),
            sg.Button('start capturing', size=(12, 1))
        ]
    ]      

    window = sg.Window(
        "ForestScanner(Online version)", layout, size=(648, 255), font=("Helvetica", 13)
    )      

    while True:      
        event, values = window.read()
        if event == None or event == 'Exit':      
            break          
        if event == 'test camera':
            window.Hide()
            gui_ouput(exec_function=test_main)
            window.UnHide()


def gui_offline_version():
    layout = [      
        [sg.T()],
        [
            sg.T(size=(2, 1)),
            sg.Text('Output folder:', size=(11, 1)), 
            sg.Input(size=(43, 1), font=('', 11), default_text=join(getcwd(), "dataset")), 
            sg.FolderBrowse()
        ],
        [sg.T(font=('', 5))],
        [
            sg.T(size=(2, 1)),
            sg.Text('images:'), sg.Input(size=(6, 1), default_text="30"), sg.T(size=(2, 1)),
            sg.Text('voxel size:'), sg.Input(size=(8, 1), default_text="0.0025"),sg.T(size=(2, 1)),
            sg.Text('max depth:'), sg.Input(size=(4, 1), default_text="1.0"), sg.Text('m'),
        ],
        [sg.T(font=('', 5))],
        [
            sg.T(size=(2, 1)),
            sg.Text('Register method:'),
            sg.InputCombo(('point to plane ICP', 'color ICP'), readonly=True, size=(15, 1), 
                          default_value='point to plane ICP',font=('Helvetica', 10)),
            sg.T(size=(1, 1)),
            sg.Checkbox('only scan body', default=True),
            sg.Button('start', size=(7, 1))
        ],
    ]      

    window = sg.Window(
        "ForestScanner(Offline version)", layout, size=(648, 225), font=("Helvetica", 13)
    )      

    while True:      
        event, values = window.read()
        if event == None or event == 'Exit':      
            break          
        if event == 'test camera':
            window.Hide()
            gui_ouput(exec_function=test_main)
            window.UnHide()


if __name__ == "__main__":
    VERSION = gui_choose_version()
    if VERSION == "online":
        gui_online_version()
    elif VERSION == "offline":
        gui_offline_version()