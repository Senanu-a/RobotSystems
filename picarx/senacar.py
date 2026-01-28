import time
from picarx_improved import Picarx

px = Picarx()
px.set_dir_servo_angle(0)
def fwd_back_turn():
    # move forward at a speed of 100 for 3s
    px.forward(100)
    time.sleep(3)
    px.stop()
    time.sleep(0.5)

    #move back at a speed of 50 for 5s
    px.backward(50)
    time.sleep(5)

    #turn at an angle of 25deg and move forward at a speed of 40 for 3s
    px.set_dir_servo_angle(25)
    px.forward(40)
    time.sleep(3)

    #turn at an angle of -25 and reverse at a speed of 30 for 4s
    px.set_dir_servo_angle(-25)
    px.backward(30)
    time.sleep(4)

    #straighten your wheels
    px.set_dir_servo_angle(0)
    time.sleep(1)
    px.stop()

def parallel_parking():

    #inital forward motion AT 100 FOR 3S
    px.forward(100)
    time.sleep(3)

    #reverse turn into parking space 
    px.set_dir_servo_angle(25)
    px.backward(30)
    time.sleep(2)

    #oppose the reverse turn earlier done
    px.set_dir_servo_angle(-25)
    px.backward(30)
    time.sleep(2)

    #straigthen and park
    px.set_dir_servo_angle(0)
    px.forward(30)
    time.sleep(1)
    px.stop()

def three_point_turn():

    #initial motion of car
    px.forward(100)
    time.sleep(3)

    #stop for a second
    px.forward(0)
    time.sleep(1)

    #first turn in the 3 turn process(forward + turn)
    px.set_dir_servo_angle(-30)
    px.forward(30)
    time.sleep(2)

    #second turn(reverse +opp turn)
    px.set_dir_servo_angle(30)
    px.backward(30)
    time.sleep(2)

    #final turn(forward +turn)
    px.set_dir_servo_angle(-30)
    px.forward(40)
    time.sleep(2)

    #straighten tires and proceed
    px.set_dir_servo_angle(0)
    px.forward(40)
    time.sleep(3)
    px.stop()

running = True
while running:

    user_input = int(input("What Cool Stuff do you want to see the car do?\nFor Forward,Reverse and Turning display; \nPress 1 \nFor Parallel parking;\nPress 2 \nFor A K-turn;\nPress 3\nPlease ender your selection here:--"))
    if user_input == 1:
        fwd_back_turn()
    elif user_input == 2:
        parallel_parking()
    elif user_input == 3:
        three_point_turn()
    elif user_input == 0:
        print("exiting Program")
        px.stop()
        running = False
    else:
        print("Please select a valid input")
        
