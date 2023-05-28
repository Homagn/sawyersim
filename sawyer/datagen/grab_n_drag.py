import sys
import os

os.environ['CLIENT'] = '/home/homagni/sawyer/'
sys.path.append(os.path.join(os.environ['CLIENT']))

from client import *

    
#=================================== EXAMPLE FUNCTIONS =========================================    
def request_template():
    request =  {'action':{'delta_joint_pos':[], 'gripper':'', 'ikpos':[], 'preset_pos':'', 'move_object':{}}, 
               'observation':[]}
    return request

def simple_pick(gc, x, y):
    #assuming the block to pick is at the position (x,y)=(0.75,0.0)
    request = request_template()
    obs_seq = []

    #1. Position default spawned cube according to x, y
    request['action']['move_object']['cube']=[x,y,0.8]
    request['observation'] = ['joint_angles']
    o1 = gc.step(request)
    #nullify move request
    request['action']['move_object']={}

    #2. Open the gripper and move it approximately close to default spawned cube
    request['action']['gripper']='open'
    request['action']['ikpos']=[0.45,0.16,0.4] #raise the end effector a bit higher from the starting neutral position
    request['observation'] = ['rgb','end_pos','joint_angles']
    gc.step(request)
    request['action']['ikpos']=[x,y,0.4]
    o2 = gc.step(request) #passed values are global positions wrt start

    #3. Move the end effector further downward near to the object
    request['action']['gripper']=''
    request['action']['ikpos']=[x,y,-0.14]
    request['observation'] = ['end_pos','joint_angles']
    o3 = gc.step(request) #passed values are global positions wrt start

    #4. Finally close the gripper and grab the object
    request['action']['gripper']='close'
    request['action']['ikpos']=[]
    o4 = gc.step(request) #passed values are global positions wrt start

    #5. Take the object with the arm and lift it to neutral position
    request['action']['gripper']=''
    request['action']['preset_pos']='neutral'
    o5 = gc.step(request) #passed values are global positions wrt start

    obs_seq = [o1,o2,o3,o4,o5]
    return obs_seq

def reach(gc, sp, syaw, sr, sx, sy, sz):
    request = request_template()
    obs_seq = []

    #1. Position default spawned cube according to x, y
    request['action']['move_object']['cube']=[sx,sy,0.8]
    request['observation'] = ['joint_angles']
    o1 = gc.step(request)
    #nullify move request
    request['action']['move_object']={}

    #2. Open the gripper and move it approximately close to default spawned cube
    request['action']['gripper']='open'
    request['action']['ikpos']=[0.45,0.16,0.4] #raise the end effector a bit higher from the starting neutral position
    request['observation'] = ['rgb','end_pos','joint_angles']
    gc.step(request)
    request['action']['ikpos']=[sx,sy,sz,  sp, syaw, sr]
    o2 = gc.step(request) #passed values are global positions wrt start

    obs_seq = [o1,o2]
    return obs_seq

def grab_n_drag(gc, sp, syaw, sr, sx, sy, sz, tp, tyaw, tr, tx, ty, tz):
    '''
    variables
    gc- gazebo client
    p- pitch value of end effector quaternion while grasping the object
    y- yaw value
    r- roll value
    
    sx- starting x position of the cube
    sy- starting y position of the cube

    tx- target x position of the end effector where it moves to drag the grabbed object along with it
    ty- target y
    tx- target z
    '''
    request = request_template()
    obs_seq = []

    #1. Position default spawned cube according to x, y
    #   Make sure the table is set to the default position too (might have been disturbed due to arm motion)
    request['action']['move_object']['cube']=[sx,sy,0.8]
    request['action']['move_object']['table']=[0.63,0.20,0] #default table position
    request['observation'] = ['joint_angles']
    o1 = gc.step(request)
    #nullify move request
    request['action']['move_object']={}

    #2. Open the gripper and move it approximately close to default spawned cube
    request['action']['gripper']='open'
    request['action']['ikpos']=[0.45,0.16,0.4] #raise the end effector a bit higher from the starting neutral position
    request['observation'] = ['end_pos','joint_angles']
    gc.step(request)
    request['action']['ikpos']=[sx,sy,0.4]
    o2 = gc.step(request) #passed values are global positions wrt start


    #3. Move the end effector further downward near to the object
    request['action']['gripper']=''
    request['action']['ikpos']=[sx,sy,sz, sp, syaw, sr] #sz=-0.14 always for this table setup
    request['observation'] = ['end_pos','joint_angles']
    o3 = gc.step(request) #passed values are global positions wrt start

    #4. Finally close the gripper and grab the object
    request['action']['gripper']='close'
    request['action']['ikpos']=[]
    request['observation'] = ['rgb', 'rgb_side', 'end_pos','joint_angles']
    o4 = gc.step(request) #passed values are global positions wrt start

    #5. (different from example pick func) drag the object by specifying an IK target of end effector
    request['action']['ikpos']=[tx,ty,tz, sp, syaw, sr]
    o5 = gc.step(request)
    request['action']['ikpos']=[tx,ty,tz, tp, tyaw, tr] #chenge the final end effector angles in mid air
    request['observation'] = ['rgb', 'rgb_side', 'end_pos','joint_angles','grasp_success']
    o5 = gc.step(request)

    #6. Finally get back the arm to neutral position to be ready for next random block 
    request['action']['preset_pos']='neutral'
    request['observation'] = ['end_pos','joint_angles']
    o6 = gc.step(request)
    

    obs_seq = [o1,o2,o3,o4,o5,o6]
    return obs_seq


if __name__ == '__main__':
    import cv2
    import json
    import random


    #grab an object and drag it to some position
    gc = gazebo_client()
    labels = {}
    rez = 100
    start_x_range = 0.2
    start_y_range = 0.45

    final_pitch_range = 0.4
    final_yaw_range = 0.4

    disp_range = 0.2


    def ran(rez):
        return random.choice(list(range(rez)))

    i=0

    for n in range(10):

        print("generating sample ",n)
        

        sp = -3.138
        syaw = 0.005
        sr = -3.138

        sx = 0.4 + (start_x_range/rez)*ran(rez)
        sy = -0.05 + (start_y_range/rez)*ran(rez)
        sz = -0.14

        tp = -3.4 + (final_pitch_range/rez)*ran(rez)
        tyaw = -0.2 + (final_yaw_range/rez)*ran(rez)
        tr = -3.138

        tx = sx + (disp_range/rez)*ran(rez)
        ty = sy + (disp_range/rez)*ran(rez)
        tz = sz + (disp_range/rez)*ran(rez)


        #observations = grab_n_drag(gc, -3.138, 0.005, -3.138,    0.5, 0.25,-0.14,   -3.138, -0.2, -3.138,    0.6, 0.35, -0.08) #safe working range (0.4,-0.15) -> (0.65,0.4)
        observations = grab_n_drag(gc, sp, syaw, sr, sx, sy, sz, tp, tyaw, tr, tx, ty, tz)

        print(dt.now())



        if float(observations[4]['grasp_success'])>0.78: #means cube has been lifted off table
            cv2.imwrite('data/images/'+repr(i)+'_front1.png', observations[3]['rgb']) 
            cv2.imwrite('data/images/'+repr(i)+'_front2.png', observations[4]['rgb']) 

            cv2.imwrite('data/images/'+repr(i)+'_side1.png', observations[3]['rgb_side']) 
            cv2.imwrite('data/images/'+repr(i)+'_side2.png', observations[4]['rgb_side']) 


            
            labels[repr(i)] = {}

            labels[repr(i)]['sp'] = sp
            labels[repr(i)]['syaw'] = syaw
            labels[repr(i)]['sr'] = sr

            labels[repr(i)]['sx'] = sx
            labels[repr(i)]['sy'] = sy
            labels[repr(i)]['sz'] = sz

            labels[repr(i)]['tp'] = tp
            labels[repr(i)]['tyaw'] = tyaw
            labels[repr(i)]['tr'] = tr

            labels[repr(i)]['tx'] = tx
            labels[repr(i)]['ty'] = ty
            labels[repr(i)]['tz'] = tz

            i+=1

        else:
            print("WARNING ! Grasp was not successful, so not saving data !")




    with open('data/labels.json', 'w') as fp:
        json.dump(labels, fp, indent = 4)

    gc.stop() #After this the server code would stop with an error
