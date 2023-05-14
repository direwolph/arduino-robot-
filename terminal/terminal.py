#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May  8 22:01:17 2023

@author: julius
"""

import curses
import time
import serial
import serial.tools.list_ports  
def draw_square(stdscr,x, y, color):
    color_pair = None
    if color == -1:
        color_pair = curses.color_pair(1)
    elif color == 0:
        color_pair = curses.color_pair(2)
    elif color == 100:
        color_pair = curses.color_pair(3)
    elif color < -10:
        color_pair = curses.color_pair(4)
    stdscr.attron(color_pair)
    for i in range(square_size):
        for j in range(square_size):
            stdscr.addch(y + i, x + j, " ")
    stdscr.attroff(color_pair)
    
    
    

def main(stdscr):
    # Set up the curses environment
    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()
    curses.cbreak()
    curses.noecho()
    stdscr.keypad(True)
    curses.curs_set(0)
    
    curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_BLACK) #unknown cell
    curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_GREEN) #free cell
    curses.init_pair(3, curses.COLOR_RED, curses.COLOR_RED) #obstacle in this cell
    curses.init_pair(4, curses.COLOR_BLUE, curses.COLOR_BLUE) #robot path
    stdscr.nodelay(True)

    # Initialize the data

    # Main loop
    while True:
        # Clear the screen
        global list_data
        data = str(arduino.readline()[:-2])[2:-1]
        list_data = data.split(";")
        # Update the data
        
        stdscr.clear()
        # Display the data
        test(stdscr)
        stdscr.addstr(0, 0, "---- position ----")
        stdscr.addstr(0, 1, f"postion from servo X: {list_data[0]}")
        stdscr.addstr(0, 2, f"postion from servo Y: {list_data[1]}")
        stdscr.addstr(0, 3, f"postion from servo angle: {list_data[2]}")
        stdscr.addstr(0, 4, f"postion from IMU X: {list_data[3]}")
        stdscr.addstr(0, 5, f"postion from IMU Y: {list_data[4]}")
        stdscr.addstr(0, 6, f"postion from IMU angle: {list_data[5]}")
        stdscr.addstr(0, 7, f"postion from fusion X: {list_data[6]}")
        stdscr.addstr(0, 8, f"postion from fusion Y: {list_data[7]}")
        stdscr.addstr(0, 9, f"postion from fusion angle: {list_data[8]}")
        stdscr.addstr(0, 10, f"distance from flag: {list_data[9]}")
        stdscr.addstr(0, 11, f"delta angle: {list_data[10]}")
        
        stdscr.addstr(15, 0, "---- movement ----")
        stdscr.addstr(15, 1, f"velocity : {list_data[11]}")
        stdscr.addstr(15, 2, f"angular velocity : {list_data[12]}")
        stdscr.addstr(15, 3, f"targeted angle : {list_data[13]}")
        
        stdscr.addstr(15, 4, "---- sensor ----")
        stdscr.addstr(15, 5, f"last mes dist : {list_data[14]}")
        stdscr.addstr(15, 6, f"last mes angle : {list_data[15]}")
        
        stdscr.addstr(15, 7, "---- path ----")
        stdscr.addstr(15, 8, f"nbr of pts : {list_data[16]}")
        
        stdscr.addstr(15, 9, "---- flag ----")
        stdscr.addstr(15, 10, f"flag X : {list_data[19]}")
        stdscr.addstr(15, 11, f"flag Y : {list_data[20]}")
        list_grid = list_data[21].join(":")
        for i in range(len(list_grid)):
            x = i//list_data[22]
            y = i%list_data[22]
            draw_square(stdscr,x, y,list_grid[i])
        list_path_x = list_data[17].join(":")
        list_path_y = list_data[18].join(":")
        for i in range(len(list_path_y)):
            x = list_path_x[i]
            y = list_path_y[i]
            draw_square(stdscr,x, y,-10)
        # Refresh the screen
        stdscr.refresh()

        # Check for input (to allow the user to exit the program)
        c = stdscr.getch()
        if c == ord('q'):
            break

# Start the program

print("Recherche d'un port serie...")

ports = serial.tools.list_ports.comports(include_links=False)

if (len(ports) != 0): # on a trouvé au moins un port actif

    if (len(ports) > 1):     # affichage du nombre de ports trouvés
        print (str(len(ports)) + " ports actifs ont ete trouves:") 
    else:
        print ("1 port actif a ete trouve:")

    ligne = 1
    
    for port in ports :  # affichage du nom de chaque port
        print(str(ligne) + ' : ' + port.device)
        ligne = ligne + 1

    portChoisi = int(input("\nPort de la carte : "))
    
    # on établit la communication série
    arduino = serial.Serial(ports[portChoisi - 1].device, 115200, timeout=1)
    
    print('Connexion a ' + arduino.name)

    # si on reçoit un message, on l'affiche
    while True:
        data = arduino.readline()[:-2]
        if data:
            curses.wrapper(main)

else: # on n'a pas trouvé de port actif
    print("Aucun port actif n'a ete trouve")


