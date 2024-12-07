import pygame as pg
import RPi.GPIO as GPIO

curr_pos = [0,0,0,0,0,0,0]

xyz_delta = 0.01
rot_delta = 0.01
delta = [xyz_delta, xyz_delta, xyz_delta, rot_delta, rot_delta, rot_delta, 0.01]

def main():
    pg.init()
    while True:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                return
            if event.type == pg.KEYDOWN:
                #J/K to control vertical DOF
                if event.key == pg.K_j:
                    curr_pos[2] += delta[2]
                if event.key == pg.K_k:
                    curr_pos[2] -= delta[2]
                #W/S to control first rotational DOF 
                if event.key == pg.K_w:
                    curr_pos[3] += delta[3]
                if event.key == pg.K_s:
                    curr_pos[3] -= delta[3]
                #A/D to control second rotational DOF 
                if event.key == pg.K_a:
                    curr_pos[4] += delta[4]
                if event.key == pg.K_d:
                    curr_pos[4] -= delta[4]
                #L/H to control third rotational DOF 
                if event.key == pg.K_l:
                    curr_pos[5] += delta[5]
                if event.key == pg.K_h:
                    curr_pos[5] -= delta[5]
                #N/C to control gripper
                if event.key == pg.K_n:
                    curr_pos[5] += delta[5]
                if event.key == pg.K_c:
                    curr_pos[5] -= delta[5]

if __name__ == "__main__":
    main()
    pg.quit()
