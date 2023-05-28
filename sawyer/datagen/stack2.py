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

def stack2(gc, x1, y1, x2,y2, xt, yt, stall_height = 0.4):
    '''
    variables:
    gc - gazebo client
    x1,y1- initial position of first cube (cube)
    x2,y2- initial position of the second cube (cube2)
    xt,yt- Final position of the 2 stacked cubes
    '''
    request = request_template()
    #request['observation'] = ['rgb', 'rgb_side', 'end_pos','joint_angles','grasp_success_cube', 'grasp_success_cube2']
    request['observation'] = ['rgb', 'end_pos','joint_angles','grasp_success_cube', 'grasp_success_cube2']
    obs_seq = []
    act_seq = {}

    #1. Position default spawned cubes according to x, y
    #   Make sure the table is set to the default position too (might have been disturbed due to arm motion)
    request['action']['move_object']['cube']=[x1,y1,0.8]
    request['action']['move_object']['cube2']=[x2,y2,0.8]
    request['action']['move_object']['table']=[0.63,0.20,0] #default table position

    #reset camera position to remove previous disturbances
    request['action']['move_object']['camera_platform']=[3.75,0.0,0, 0,0,1.57] #can pass last 3 variables for picth,yaw,roll
    request['action']['move_object']['camera_platform_side']=[0.75,3.5,0, 0,0,3.14]

    request['action']['preset_pos']='neutral'
    act_seq['1'] = copy.copy(request['action'])
    o1 = gc.step(request)
    
    #nullify move request
    request['action']['move_object']={}

    #2. Open the gripper and move it approximately close to default spawned cube
    request['action']['gripper']='open'
    request['action']['preset_pos']=''
    request['action']['ikpos']=[0.45,0.16,stall_height] #raise the end effector a bit higher from the starting neutral position
    gc.step(request)
    request['action']['ikpos']=[x1,y1,stall_height]
    act_seq['2'] = copy.copy(request['action'])
    o2 = gc.step(request) #passed values are global positions wrt start
    

    #3. Move the end effector further downward near to the object
    request['action']['gripper']=''
    request['action']['ikpos']=[x1,y1,-0.14]
    act_seq['3'] = copy.copy(request['action'])
    o3 = gc.step(request) #passed values are global positions wrt start
    

    #4. Finally close the gripper and grab the object
    request['action']['gripper']='close'
    request['action']['ikpos']=[]
    act_seq['4'] = request['action']
    o4 = gc.step(request) #passed values are global positions wrt start
    

    #5. Pick the grabbed object to the stall height
    request['action']['ikpos']=[x1,y1,stall_height]
    act_seq['5'] = copy.copy(request['action'])
    o5 = gc.step(request) 
    

    #6. Move the grabbed object to the stack location
    request['action']['ikpos']=[xt,yt,stall_height]
    act_seq['6'] = copy.copy(request['action'])
    o6 = gc.step(request) 
    


    #7. Drop the grabbed object to the stack position
    request['action']['ikpos']=[xt,yt,-0.14]
    gc.step(request) #passed values are global positions wrt start
    request['action']['gripper']='open'
    request['action']['ikpos']=[xt,yt,stall_height]
    request['action']['preset_pos']='neutral'
    act_seq['7'] = copy.copy(request['action'])
    o7 = gc.step(request)
    


    #8. Move the gripper over to the 2nd object location
    request['action']['preset_pos']=''
    request['action']['gripper']=''
    request['action']['ikpos']=[x2,y2,stall_height]
    act_seq['8'] = copy.copy(request['action'])
    o8 = gc.step(request) #passed values are global positions wrt start
    


    #9. Drop the gripper
    request['action']['ikpos']=[x2,y2,-0.14]
    o9 = gc.step(request) #passed values are global positions wrt start
    request['action']['gripper']='close'
    act_seq['9'] = copy.copy(request['action'])
    gc.step(request)
    

    #10. Close the gripper and grab the object
    request['action']['gripper']=''
    request['action']['ikpos']=[xt,yt,stall_height]
    act_seq['10'] = copy.copy(request['action'])
    o10 = gc.step(request) #passed values are global positions wrt start
    
    request['action']['ikpos']=[xt,yt,-0.1]
    gc.step(request) #passed values are global positions wrt start
    t.sleep(0.5)#slight sleep to nullify the velocity of the block itself which might be sent flying after gripper opens


    #11. Drop the object on the stack
    request['action']['gripper']='open'
    request['action']['ikpos']=[]
    act_seq['11'] = copy.copy(request['action'])
    o11 = gc.step(request) #passed values are global positions wrt start

    print("act seq ",act_seq)
    


    obs_seq = [o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11]
    return obs_seq, act_seq



if __name__ == '__main__':
    import cv2
    import json
    import random


    #grab an object and drag it to some position
    gc = gazebo_client()
    labels = {}
    rez = 100
    num_demo = 30000

    #task variation parameters
    start_x_range = 0.2
    start_y_range = 0.35


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

        rx,ry = ran(rez),ran(rez)
        x1,x2 = 0.6 + (start_x_range/rez)*rx,  0.6 - (start_x_range/rez)*rx
        y1,y2 = 0.2 + (start_y_range/rez)*ry, 0.2 - (start_y_range/rez)*ry
        
        xt,yt = 0.4 + (start_x_range/rez)*ran(rez), -0.05 + (start_y_range/rez)*ran(rez)

        observations, action_sequence = stack2(gc, x1,y1,x2,y2,xt,yt)

        print(dt.now())



        if float(observations[4]['grasp_success_cube'])>0.78 and float(observations[10]['grasp_success_cube2'])>0.78: #means cubes has been lifted off table
            
            cv2.imwrite('data/images/'+repr(i)+'_front1.png', observations[0]['rgb']) 
            cv2.imwrite('data/images/'+repr(i)+'_front2.png', observations[1]['rgb']) 
            cv2.imwrite('data/images/'+repr(i)+'_front3.png', observations[2]['rgb']) 
            cv2.imwrite('data/images/'+repr(i)+'_front4.png', observations[4]['rgb']) 
            cv2.imwrite('data/images/'+repr(i)+'_front5.png', observations[5]['rgb']) 
            cv2.imwrite('data/images/'+repr(i)+'_front6.png', observations[6]['rgb']) 
            cv2.imwrite('data/images/'+repr(i)+'_front7.png', observations[7]['rgb']) 
            cv2.imwrite('data/images/'+repr(i)+'_front8.png', observations[8]['rgb']) 
            cv2.imwrite('data/images/'+repr(i)+'_front9.png', observations[9]['rgb']) 
            cv2.imwrite('data/images/'+repr(i)+'_front10.png', observations[10]['rgb']) 


            '''
            cv2.imwrite('data/images/'+repr(i)+'_side1.png', observations[0]['rgb_side']) 
            cv2.imwrite('data/images/'+repr(i)+'_side2.png', observations[1]['rgb_side']) 
            cv2.imwrite('data/images/'+repr(i)+'_side3.png', observations[2]['rgb_side']) 
            cv2.imwrite('data/images/'+repr(i)+'_side4.png', observations[4]['rgb_side']) 
            cv2.imwrite('data/images/'+repr(i)+'_side5.png', observations[5]['rgb_side']) 
            cv2.imwrite('data/images/'+repr(i)+'_side6.png', observations[6]['rgb_side']) 
            cv2.imwrite('data/images/'+repr(i)+'_side7.png', observations[7]['rgb_side']) 
            cv2.imwrite('data/images/'+repr(i)+'_side8.png', observations[8]['rgb_side']) 
            cv2.imwrite('data/images/'+repr(i)+'_side9.png', observations[9]['rgb_side']) 
            cv2.imwrite('data/images/'+repr(i)+'_side10.png', observations[10]['rgb_side']) 
            '''


            
            labels[repr(i)] = {}

            labels[repr(i)]['x1'] = x1
            labels[repr(i)]['x2'] = x2

            labels[repr(i)]['y1'] = y1
            labels[repr(i)]['y2'] = y2

            labels[repr(i)]['xt'] = xt
            labels[repr(i)]['yt'] = yt

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
            print("WARNING ! Grasp was not successful, so not saving data !")




    '''
    with open('data/labels.json', 'w') as fp:
        json.dump(labels, fp, indent = 4)
    '''

    gc.stop() #After this the server code would stop with an error
