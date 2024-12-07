import pygame as pg
import RPi.GPIO as GPIO

curr_pos = [0,0,0,0,0,0,0]

xyz_delta = 0.01
rot_delta = 0.01
delta = [xyz_delta, xyz_delta, xyz_delta, rot_delta, rot_delta, rot_delta, 0.01]

GPIO.setmode(GPIO.BCM)
SCREENRECT = pg.Rect(0, 0, 640, 480)

def main():
    pg.init()
    fullscreen = False
    # Set the display mode
    winstyle = 0  # |FULLSCREEN
    bestdepth = pg.display.mode_ok(SCREENRECT.size, winstyle, 32)
    screen = pg.display.set_mode(SCREENRECT.size, winstyle, bestdepth)
    directions = [0,0,0,0,0,0,0]
    while True:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                return
            if event.type == pg.KEYDOWN or event.type == pg.KEYUP:
                mult = 1 if event.type == pg.KEYDOWN else -1
                #W/S to control X DOF
                if event.key == pg.K_w:
                    directions[0] += mult
                    print("w pressed")
                if event.key == pg.K_s:
                    directions[0] -= mult
                #A/D to control Y DOF
                if event.key == pg.K_a:
                    directions[1] += mult
                if event.key == pg.K_d:
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
        print(curr_pos)

if __name__ == "__main__":
    main()
    pg.quit()
