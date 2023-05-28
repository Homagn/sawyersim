# client.py
#(main pthon3 ML codes use this for interacting with sawyer gazebo running on docker containers)
#(this code itself writen in python 3)

import socket                   # Import socket module
from datetime import datetime as dt
from zipfile import ZipFile 
import time as t
import struct
import pickle
import json

class gazebo_client(object):
    def __init__(self, host = '127.0.0.1', port = 60000): #10.24.250.119
        self.s = socket.socket()             # Create a socket object
        #host = socket.gethostname()     # Get local machine name
        self.host = host
        self.port = port                   # Reserve a port for your service.
        
        self.s.connect((self.host, self.port))

        print("Initiated connection")

    def send_msg(self,msg):
        msg = struct.pack('>I',len(msg)) + msg
        self.s.sendall(msg)

    def send_json_msg(self,msg):
        self.s.sendall(msg)

    def recv_msg(self):
        # Read message length and unpack it into an integer
        raw_msglen = self.recvall(4)
        if not raw_msglen:
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]
        # Read the message data
        return self.recvall(msglen)
    def recvall(self, n):
        # Helper function to recv n bytes or return None if EOF is hit
        data = bytearray()
        while len(data) < n:
            packet = self.s.recv(n - len(data))
            if not packet:
                return None
            data.extend(packet)
        return data
    def step(self,request, sleep_sec = -1.0): #send actions over to gazebo and receive observations from remote host gazebo
        #actions = request['action']
        #obs = request['obs']
        req = pickle.dumps(request, protocol=2) #python 2 protocol because gazebo server in docker is using python2 

        #self.send_msg(actions.encode()) #Thing sent must be a string otherwise broken pipe error
        self.send_msg(req) #Thing sent must be a string otherwise broken pipe error
        #blocksize= int(self.s.recv(1024))
        #print("Got file blocksize ",blocksize)
        self.byte_data = self.recv_msg()
        #self.byte_data = self.byte_data.decsode()
        self.data = pickle.loads(self.byte_data, encoding="bytes")
        #self.data = json.loads(self.byte_data)
        #print("Got data from server ",self.data)
        observations = {}

        if 'depth' in request['observation']:
            depth = self.data[b'depth']
            print("Got depth data shape ",depth.shape)
            observations['depth'] = depth
        if 'rgb' in request['observation']:
            rgb = self.data[b'rgb']
            print("Got rgb data shape ",rgb.shape)
            observations['rgb'] = rgb
        if 'rgb_side' in request['observation']:
            rgb_side = self.data[b'rgb_side']
            print("Got rgb_side data shape ",rgb_side.shape)
            observations['rgb_side'] = rgb_side
        if 'joint_angles' in request['observation']:
            jangles = self.data[b'jangles']
            print("Got joint angles ",jangles)
            observations['joint_angles'] = jangles
        if 'end_pos' in request['observation']:
            end_pos = self.data[b'end_pos']
            print("Got end effector position ",end_pos)
            observations['end_pos'] = end_pos
        if 'grasp_success' in request['observation']:
            s = self.data[b'grasp_success']
            print("Got height of the cube ",s)
            gs = False
            if float(s)>0.78: #the current height of the table
                gs = True
            print("Grasp success ? ",gs)
            observations['grasp_success'] = s

        if 'grasp_success_cube' in request['observation']:
            s = self.data[b'grasp_success_cube']
            print("Got height of the cube ",s)
            gs = False
            if float(s)>0.78: #the current height of the table
                gs = True
            print("Grasp success cube ? ",gs)
            observations['grasp_success_cube'] = s

        if 'grasp_success_cube2' in request['observation']:
            s = self.data[b'grasp_success_cube2']
            print("Got height of the cube ",s)
            gs = False
            if float(s)>0.78: #the current height of the table
                gs = True
            print("Grasp success cube2 ? ",gs)
            observations['grasp_success_cube2'] = s

        if sleep_sec>0.0:
            t.sleep(sleep_sec)

        return observations

    def stop(self):
        self.s.close()
        print('connection closed')
    
#=================================== EXAMPLE FUNCTIONS =========================================    
def request_template():
    request =  {'action':{'delta_joint_pos':[], 'gripper':'', 'ikpos':[], 'preset_pos':'', 'move_object':{}}, 
               'observation':[]}
    return request

def example_pick(gc, x, y):
    #assuming the block to pick is at the position (x,y)=(0.75,0.0)
    request = request_template()
    obs_seq = []

    #1. Position default spawned cube according to x, y
    request['action']['move_object']['cube']=[x,y,0.8]
    #move table to the default position in case it was disturbed by arm
    request['action']['move_object']['table']=[0.63,0.20,0] #dafeult table position
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
    request['action']['move_object']['cube']=[sx,sy,0.8]
    request['action']['move_object']['table']=[0.63,0.20,0] #dafeult table position
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
    request['observation'] = ['rgb', 'rgb_side', 'end_pos','joint_angles', 'grasp_success']
    o5 = gc.step(request)

    #6. Finally get back the arm to neutral position to be ready for next random block 
    request['action']['preset_pos']='neutral'
    request['observation'] = ['end_pos','joint_angles']
    o6 = gc.step(request)

    obs_seq = [o1,o2,o3,o4,o5]
    return obs_seq


if __name__ == '__main__':
    import cv2

    '''
    #pick up an object and move to neutral position
    gc = gazebo_client()
    print(dt.now())
    observations = example_pick(gc,0.4,-0.15) #safe working range (0.4,-0.15) -> (0.65,0.4)
    print(dt.now())
    cv2.imwrite('sample_img.png', observations[1]['rgb']) 
    '''

    
    #grab an object and drag it to some position
    gc = gazebo_client()
    print(dt.now())
    observations = grab_n_drag(gc, -3.138, 0.005, -3.138,    0.5, 0.25,-0.14,   -3.138, -0.2, -3.138,    0.6, 0.35, -0.08) #safe working range (0.4,-0.15) -> (0.65,0.4)
    print(dt.now())
    t.sleep(5)


    cv2.imwrite('1_front1.png', observations[3]['rgb']) 
    cv2.imwrite('1_front2.png', observations[4]['rgb']) 

    cv2.imwrite('2_side1.png', observations[3]['rgb_side']) 
    cv2.imwrite('2_side2.png', observations[4]['rgb_side']) 

    import json

    labels = {}
    labels[repr(1)] = {}

    labels[repr(1)]['sp'] = -3.138
    labels[repr(1)]['syaw'] = 0.005
    labels[repr(1)]['sr'] = -3.138

    labels[repr(1)]['sx'] = 0.5
    labels[repr(1)]['sy'] = 0.25
    labels[repr(1)]['sz'] = -0.14

    labels[repr(1)]['tp'] = -3.138
    labels[repr(1)]['tyaw'] = -0.2
    labels[repr(1)]['tr'] = -3.138

    labels[repr(1)]['tx'] = 0.6
    labels[repr(1)]['ty'] = 0.35
    labels[repr(1)]['tz'] = -0.08

    with open('data/labels.json', 'w') as fp:
        json.dump(labels, fp, indent = 4)



    

    '''
    #reach to a specified end effector pose (pitch, yaw, roll, x, y, z)
    gc = gazebo_client()
    print(dt.now())
    #possible pitch values are [-3.5,3.5]
    #possible yaw values [-0.5,0.5]
    #possible roll  values are 1.57 or 0.0 for cube
    observations = reach(gc, -2.0, 0.005, -1.57,    0.5, 0.25,0.1) #possible r  values are 1.57 and 0.0 for cube
    print(dt.now())
    t.sleep(5)
    '''


    gc.stop() #After this the server code would stop with an error

#NOTE ! - def recvall(self, n): is the slowest function taking avg 0.45 secs everytime to read. Maybe try parallelize