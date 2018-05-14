#!/usr/bin/python
import roomba_500
import threading
import RPi.GPIO as GPIO
import time

robot = roomba_500.Roomba500Interface("/dev/ttyAMA0")

def someFunc():
    print "yeh yeh"

robot.set_buttons(Clean=someFunc)

def keep_alive(robot, parent):
    #try:
    while(parent.is_alive()):
        robot.signal_on()
        time.sleep(1)
    #except:
     #   print "keep_alive: Parent thread likely closed"

def start_terminal():
    input = "" 

    while input != "exit":
        input = raw_input("Please type a command: ").rstrip()
        if input == "clean":
            robot.clean()
            continue

        if input == "dock":
            robot.dock()
            continue

        if input == "is_docked":
            print("is_docked: " + str(robot.is_docked()))
   
        if input == "is_cleaning":
            print("is_cleaning: " + str(robot.is_cleaning()))
       
        if input == "pause":
            robot.pause()   

        if input == "dustbin_open":
            print("dusbin_open: " + robot.dustbin_open()) 
    robot.terminate()

#keep_alive_thread = threading.Thread(target=keep_alive, args=(robot, threading.current_thread(),))
#keep_alive_thread.start()
start_terminal()
