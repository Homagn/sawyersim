import sys
import os
import copy
import traceback
import numpy as np

os.environ['CLIENT'] = '/home/homagni/sawyer/'
sys.path.append(os.path.join(os.environ['CLIENT']))

from client import *

    
#=================================== EXAMPLE FUNCTIONS =========================================    
def request_template():
    request =  {'action':{'delta_joint_pos':[], 'gripper':'', 'ikpos':[], 'preset_pos':'', 'move_object':{}}, 
               'observation':[]}
    return request

def left2right(gc, x1, y1, x2,y2, drag_height, drag_pose):
    '''
    variables:
    gc - gazebo client
    x1,y1- initial position of first cube (cube)
    x2,y2- final dragged position of the first cube (cube)
    '''
    request = request_template()
    #request['observation'] = ['rgb', 'rgb_side', 'end_pos','joint_angles','grasp_success_cube', 'grasp_success_cube2']
    request['observation'] = ['rgb', 'end_pos','joint_angles','grasp_success_cube', 'grasp_success_cube2']
    obs_seq = []
    act_seq = {}

    #1. Position default spawned cubes according to x, y
    #   Make sure the table is set to the default position too (might have been disturbed due to arm motion)
    request['action']['move_object']['cube']=[x1,y1,0.8+ drag_height,     drag_pose[0], drag_pose[1], drag_pose[2]]
    request['action']['move_object']['table']=[0.63,0.20,0] #default table position

    #reset camera position to remove previous disturbances
    request['action']['move_object']['camera_platform']=[1.0,5.2,1.5,       -1.2,0.0,1.57] #can pass last 3 variables for picth,yaw,roll
    request['action']['move_object']['camera_platform_side']=[0.67,5.71,1.469,   -0.94,0.0,3.14]

    request['action']['preset_pos']='neutral'
    act_seq['1'] = copy.copy(request['action'])
    o1 = gc.step(request)
    
    #nullify move request
    request['action']['move_object']={}

    #2. Teleport the cube to the final position (assume some IK function is there to actually do it)
    request['action']['move_object']['cube']=[x2,y2,0.8+ drag_height,    drag_pose[0], drag_pose[1], drag_pose[2]]
    act_seq['2'] = copy.copy(request['action'])
    o2 = gc.step(request) #passed values are global positions wrt start

    act_seq['drag_vector'] = [x2-x1,y2-y1] #to be used as action embedding during training
    

    print("act seq ",act_seq)
    


    obs_seq = [o1,o2]
    return obs_seq, act_seq



if __name__ == '__main__':
    import cv2
    import json
    import random


    #grab an object and drag it to some position
    gc = gazebo_client()
    labels = {}
    rez = 100 #resolution for the random position generator
    num_demo = 3

    #task variation parameters
    x_range = 0.98-0.82
    y_range = 0.28-0.14
    z_range = 1.25-1.0
    shift_range = 0.1
    drag_pose_range = [1.5,1.5,1.5]


    def ran(rez):
        return random.choice(list(range(rez)))

    i=0

    try: #extension append labels
        with open('data/labels.json') as f:
            data = json.load(f)
        nums = int(max(np.array(list(data.keys())))) #somehow numpy can max over string rep of numbers too

        i = nums+1 #set starting counter to the number of labels we already have
    except: #labels file doesnt exist then
        print("labels file doesnt exist yet")
        traceback.print_exc()

    

    for n in range(num_demo):

        print("generating sample ",n)

        rx,ry, rdh, rdp, rs = ran(rez),ran(rez), ran(rez), ran(rez), ran(rez)

        x1 = 0.82 + (x_range/rez)*rx
        y1 = 5.0 + 0.14 + (y_range/rez)*ry

        x2 = x1 + 0.2*(shift_range/rez)*rs
        y2 = y1 + (shift_range/rez)*rs
        
        drag_height = 0.2 + (z_range/rez)*rdh #this is added to table height of 0.8
        drag_pose = [0+(drag_pose_range[0]/rez)*rdp, 0.0+(drag_pose_range[1]/rez)*rdp ,0 +(drag_pose_range[2]/rez)*rdp]
        

        observations, action_sequence = left2right(gc, x1,y1,x2,y2, drag_height, drag_pose)

        print(dt.now())



        if True: #no specific checks needed here
            
            cv2.imwrite('data/images/'+repr(i)+'_front1.png', observations[0]['rgb']) 
            cv2.imwrite('data/images/'+repr(i)+'_front2.png', observations[1]['rgb']) 



            
            labels[repr(i)] = {}
            labels[repr(i)]['x1'] = x1
            labels[repr(i)]['x2'] = x2

            labels[repr(i)]['y1'] = y1
            labels[repr(i)]['y2'] = y2

            labels[repr(i)]['action_sequence'] = action_sequence

            


            try: #extension append labels
                with open('data/labels.json') as f:
                    data = json.load(f)
                nums = int(max(np.array(list(data.keys())))) #somehow numpy can max over string rep of numbers too
                data[repr(i)] = labels[repr(i)]
                with open('data/labels.json', 'w') as fp:
                    json.dump(data, fp, indent = 4)

            except: #labels file doesnt exist then
                print("json file doesnt exist yet")
                traceback.print_exc()
                with open('data/labels.json', 'w') as fp:
                    json.dump(labels, fp, indent = 4)

            i+=1 #increment counter

        else:
            print("WARNING ! Task check returned unsuccessful, so not saving data !")




    '''
    with open('data/labels.json', 'w') as fp:
        json.dump(labels, fp, indent = 4)
    '''

    gc.stop() #After this the server code would stop with an error
