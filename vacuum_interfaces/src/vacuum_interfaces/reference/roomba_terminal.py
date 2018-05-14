#!/usr/bin/python
import roomba_dirtdog
import threading
import RPi.GPIO as GPIO
import time

robot = roomba_dirtdog.RoombaDirtdogInterface("/dev/ttyAMA0")

def keep_alive(robot, parent):
    #try:
    while(parent.is_alive()):
        robot.signal_on()
        time.sleep(1)
    #except:
     #   print "keep_alive: Parent thread likely closed"

def printThings():
    print("hello dickface")

def start_terminal():
    input = "" 
    robot.set_buttons(power=printThings)
    
    while input != "exit":
        input = raw_input("Please type a command: ").rstrip()

        if input == "clean":
            print("Sending clean")
            robot.clean()

        if input == "dock":
            robot.dock()
            continue

        if input == "pause":
            robot.pause()

        if input == "is_docked":
            print("is_docked: " + str(robot.is_docked()))
   
        if input == "is_cleaning":
            print("is_cleaning: " + str(robot.is_cleaning()))
             
    robot.terminate()

#keep_alive_thread = threading.Thread(target=keep_alive, args=(robot, threading.current_thread(),))
#keep_alive_thread.start()
start_terminal()
