import pygame as pg
import serial

curr_pos = [0,0,0,0,0,0,0]

xyz_delta = 0.0001
rot_delta = 0.0001
gripper_delta = 0.0001
delta = [xyz_delta, xyz_delta, xyz_delta, rot_delta, rot_delta, rot_delta, gripper_delta]

SCREENRECT = pg.Rect(0, 0, 640, 480)

def main():
    # Set up serial
    ser = serial.Serial('/dev/ttyACM0', baudrate=115200)
    print(ser.readline())

    # Set up pygame
    pg.init()
    fullscreen = False
    # Set the display mode
    winstyle = 0  # |FULLSCREEN
    bestdepth = pg.display.mode_ok(SCREENRECT.size, winstyle, 32)
    screen = pg.display.set_mode(SCREENRECT.size, winstyle, bestdepth)
    
    # Keep track of which direction
    directions = [0,0,0,0,0,0,0]
    print("Controls: \n XDOF: W & S \n YDOF: D & A \n ZDOF: J & K \n 1ROT: UP & DOWN \n 2ROT: LEFT & RIGHT \n 3ROT: L & H \n GRIPPER: N & C")
    while True:
        send_message = False
        for event in pg.event.get():
            if event.type == pg.QUIT:
                return
            if event.type == pg.KEYDOWN or event.type == pg.KEYUP:
                send_message = True
                #print("setting send_message to true")
                mult = 1 if event.type == pg.KEYDOWN else -1
                #W/S to control X DOF
                if event.key == pg.K_w:
                    directions[0] += mult
                if event.key == pg.K_s:
                    directions[0] -= mult
                #A/D to control Y DOF
                if event.key == pg.K_d:
                    directions[1] += mult
                if event.key == pg.K_a:
                    directions[1] -= mult
                #J/K to control vertical DOF
                if event.key == pg.K_j:
                    directions[2] += mult
                if event.key == pg.K_k:
                    directions[2] -= mult
                #W/S to control first rotational DOF 
                if event.key == pg.K_UP:
                    directions[3] += mult
                if event.key == pg.K_DOWN:
                    directions[3] -= mult
                #A/D to control second rotational DOF 
                if event.key == pg.K_LEFT:
                    directions[4] += mult
                if event.key == pg.K_RIGHT:
                    directions[4] -= mult
                #L/H to control third rotational DOF 
                if event.key == pg.K_l:
                    directions[5] += mult
                if event.key == pg.K_h:
                    directions[5] -= mult
                #N/C to control gripper
                if event.key == pg.K_n:
                    directions[6] += mult
                if event.key == pg.K_c:
                    directions[6] -= mult
        for i in range(7):
            curr_pos[i] += directions[i] * delta[i]
        # ser.flush()
        # print("0.05,0,0\n".encode("ascii"))
        bytes = ser.read_all()
        #if (len(bytes) > 0):
            #print(str(bytes))
        if (send_message):
            ser.write((direction_to_message(directions)).encode("ascii"))
            #print((direction_to_message(directions)).encode("ascii"))

def direction_to_message(directions):

    return format_coord(directions[0]) + "," +format_coord(directions[1]) + "," + format_coord(directions[2]) + "," + format_coord(directions[3]) + "," + format_coord(0.5 * directions[4] + directions[5]) + "," + format_coord(-(0.5 * directions[4] - directions[5])) + "," + format_coord(directions[6]) + "\n"

def format_coord(coord):
    if coord == 0:
        return "x"
    return str(coord)

if __name__ == "__main__":
    main()
    pg.quit()
