import serial
import random
import time

def main():
    ser = serial.Serial('/dev/ttyACM0', baudrate=115200)
    print(ser.readline())
    ranges = [(0,0.2), (0,0.2), (-0.1,0.1), (-1.57, 1.57)]
    while True:
        directions = [random.uniform(ranges[i][0], ranges[i][1]) for i in range(len(ranges))] 
        directions += [0,0,0]
        ser.write(direction_to_message(directions).encode("ascii"))
        #wait for 5 seconds
        time.sleep(10)

def direction_to_message(directions):
    return format_coord(directions[0]) + "," +format_coord(directions[1]) + "," + format_coord(directions[2]) + "," + format_coord(directions[3]) + "," + format_coord(0.5 * directions[4] + directions[5]) + "," + format_coord(-(0.5 * directions[4] - directions[5])) + "," + format_coord(directions[6]) + "\n"
        
def format_coord(coord):
    if coord == 0:
        return "x"
    return str(coord)

if __name__ == "__main__":
    main()