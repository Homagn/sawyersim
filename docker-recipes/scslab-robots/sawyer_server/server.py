# server.py
# needs modification for sawyer functionalities

#import cPickle as pickle
import sys
import os

os.environ['SAWYER_SERVER'] = '/root/ros_ws/sawyer_server'
sys.path.append(os.path.join(os.environ['SAWYER_SERVER']))

import camera
import sawyer_camera
import pointcloud
import move_sim_objects
import arm_motion

import pickle
import json
import rospy
import time as t
import cv2

import struct
import cPickle

joint_presets = {u'neutral':[0.0,-1.1798,0.0,2.18,0.0,0.569,3.14]}

def gazebo_work(am, m, actions):
    if actions[u'delta_joint_pos']!=[]:
        am.move_arm(actions[u'delta_joint_pos'])
    if actions[u'gripper']==u'open':
        t.sleep(0.5)
        am.gripper.open()
        t.sleep(0.5)
    if actions[u'gripper']==u'close':
        t.sleep(0.5)
        am.gripper.close()
        t.sleep(0.5)
    
    if actions[u'ikpos']!=[]:
        tx,ty,tz = actions[u'ikpos'][0],actions[u'ikpos'][1],actions[u'ikpos'][2]
        if len(actions[u'ikpos'])>3: #means pitch,yaw,roll of end effector has also been specified 
            p,y,r = actions[u'ikpos'][3],actions[u'ikpos'][4],actions[u'ikpos'][5]
            am.move_arm_coord(tx,ty,tz,p=p,y=y,r=r)
        else:
             am.move_arm_coord(tx,ty,tz)
    '''
    if actions[u'preset_pos']==u'neutral':
        print("Moving arm to neutral position ")
        am.limb.move_to_neutral()
        print("Done moving arm to neutral position")
    '''

    if actions[u'preset_pos']!='':
        am.move_arm_to_pos(joint_presets[actions[u'preset_pos']])

    #force move objects in the simulator for interventions
    if actions[u'move_object']!={}:
        for k in actions[u'move_object'].keys():
            print("Moving simulator object ",k)
            m.teleport_to(unicode(k), actions[u'move_object'][unicode(k)])

def reset_sim(am):
    print("Moving arm to neutral position ")
    #am.limb.move_to_neutral()
    am.move_arm_to_pos(joint_presets[u'neutral'])
    print("Done moving arm to neutral position")
    am.angles = am.limb.joint_angles()

def record_obs(sc, c,p, am, m, observation_request):
    obs_cam = c[0].see()
    obs_cam_side = c[1].see()

    obs_cam_head = sc.see("head")
    obs_cam_hand = sc.see("hand")

    obs_depth = p.see()
    jangles = am.joint_angles()
    obs = {}
    if u'rgb' in observation_request:
        obs['rgb'] = obs_cam
    if u'rgb_side' in observation_request:
        obs['rgb_side'] = obs_cam_side
    if u'head_rgb' in observation_request:
        obs['head_rgb'] = obs_cam_head
    if u'hand_rgb' in observation_request:
        obs['hand_rgb'] = obs_cam_hand
    if u'depth' in observation_request:
        obs['depth'] = obs_depth
    if u'joint_angles' in observation_request:
        obs['jangles'] = jangles
    if u'end_pos' in observation_request:
        cx,cy,cz = am.get_xyz()
        obs['end_pos'] = [cx,cy,cz]

    if u'grasp_success' in observation_request:
        s, _ = m.object_status('cube')
        obs['grasp_success'] = s

    if u'grasp_success_cube' in observation_request:
        s, _ = m.object_status('cube')
        obs['grasp_success_cube'] = s

    if u'grasp_success_cube2' in observation_request:
        s, _ = m.object_status('cube2')
        obs['grasp_success_cube2'] = s

    #write_file('obs.obj',obs)
    return pickle.dumps(obs)
    #return json.dumps(obs) #with json currently encounter some serialization problems
    
def mirror_motion(m, shift, copied = 'cube', mirrored = 'cube_m1'):
    _, p = m.object_status(copied)
    m.teleport_to(mirrored, [p[0],p[1]+shift,p[2],p[3],p[4],p[5]] )


def parse_client(x):
    result = [float(i.strip()) for i in x.split(',')]
    return result

def send_msg(sock, msg):
    # Prefix each message with a 4-byte length (network byte order)
    msg = struct.pack('>I', len(msg)) + msg
    sock.sendall(msg)

def recv_msg(sock):
    # Read message length and unpack it into an integer
    raw_msglen = recvall(sock, 4)
    if not raw_msglen:
        return None
    msglen = struct.unpack('>I', raw_msglen)[0]
    # Read the message data
    return recvall(sock, msglen)

def recvall(sock, n):
    # Helper function to recv n bytes or return None if EOF is hit
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data


if __name__ == '__main__':
    import thread
    thread.start_new_thread(os.system, ("roslaunch sawyer_gazebo sawyer_world.launch electric_gripper:=true",))
    #wait about 40 seconds till sawyer simulation is setup
    t.sleep(40)
    print 'Staring post gazebo script'

    #need to initialize a node for all subsequent ros subscribers
    rospy.init_node('Hello_Sawyer', anonymous=True)
    #set initial sawyer arm position
    print 'Setting initial arm position '
    am=arm_motion.arm()
    am.reset()


    mirror_shift = 5.0 # a 5.0m shift mirrors all the object transformations


    
    print 'Spawning camera manipulation platform '
    move_sim_objects.spawn('/root/.gazebo/models/float_camera/model.sdf','camera_platform','/foo',[3.75,0.0+mirror_shift,0],[0,0,1.57]) #filename, position and quaternion
    print 'spawned camera platform'

    
    #(optional extra view if you want)
    print 'Spawning pendulum camera (side) '
    move_sim_objects.spawn('/root/.gazebo/models/float_camera_side/model.sdf','camera_platform_side','/bar',[0.75,3.5+mirror_shift,0],[0,0,3.14]) #filename, position and quaternion
    print 'spawned camera platform side'
    
    

    print 'Spawning table '
    move_sim_objects.spawn('/root/.gazebo/models/table/model.sdf','table','/fo',[0.63,0.20,0],[0,0,0.0]) #filename, position and quaternion
    print 'spawned table'

    print 'Spawning mirror table '
    move_sim_objects.spawn('/root/.gazebo/models/table/model.sdf','table_mirror','/fom',[0.63,0.20+mirror_shift,0],[0,0,0.0]) #filename, position and quaternion
    print 'spawned table'

    
    
    

    #====================
    #spawn the manipulation objects below the table
    #spawn the manipulation object - cube

    #non 0g version
    '''
    print 'Spawning manipulation object - wooden cube '
    move_sim_objects.spawn('/root/.gazebo/models/cube2/model.sdf','cube','/foo1',[0.75,0.0,0.5],[0,0,0]) #filename, position and quaternion
    print 'spawned wooden cube'
    '''

    #0 g version
    print 'Spawning manipulation object - wooden cube '
    move_sim_objects.spawn('/root/.gazebo/models/cube2_0g/model.sdf','cube','/foo1',[0.75,0.0,0.5],[0,0,0]) #filename, position and quaternion
    print 'spawned wooden cube'

    #spawn the manipulation object - cube
    print 'Spawning manipulation object - wooden cube 2'
    move_sim_objects.spawn('/root/.gazebo/models/cube2/model.sdf','cube2','/foo2',[0.75,0.0,0.5],[0,0,0]) #filename, position and quaternion
    print 'spawned wooden cube'

    #spawn the manipulation object - cube having 0 gravity supposed to mirror movements of 'cube'
    print 'Spawning manipulation object - wooden cube 2 0g'
    move_sim_objects.spawn('/root/.gazebo/models/cube2_0g/model.sdf','cube_m1','/foo3',[0.75,0.0+mirror_shift,2.0],[0,0.0,0]) #filename, position and quaternion
    print 'spawned wooden cube'



    print 'Starting environment observation nodes '
    #extra floating cameras
    c = [camera.Camera("raw"), camera.Camera("raw", topic_desc = "color_side")]
    #cameras on sawyer
    sc = sawyer_camera.camera()
    p = pointcloud.pointcloud(viz_save = True)
    t.sleep(1)
    print 'subscribed to sensors, waiting for client to connect'


    import socket                   # Import socket module
    port = 60000                    # Reserve a port for your service.
    s = socket.socket()             # Create a socket object
    s.settimeout(1000.0)            # Set a long timeout useful for debugging client codes and turn them in again keeping socker running
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #allow reuse so that dont get the socket already in use error
    #s.setblocking(0)
    #host = socket.gethostname()     # Get local machine name
    #s.bind((host, port))            # Bind to the port
    s.bind(('', port))            # Bind to the port
    s.listen(5)                     # Now wait for client connection.

    print 'Server listening....'


    while True:
        #client can connect and disconnect indefinitely server should always keep running
        conn, addr = s.accept()     # Establish connection with client.
        print 'Got connection from', addr
        while True:
            byte_data = recv_msg(conn)
            #print("Got byte data ",byte_data)
            if byte_data!=None:
                data = pickle.loads(str(byte_data))

                print('Server received request to act',data[u'action'])
                print('Server received request to provide observations',data[u'observation'])

                gazebo_work(am, move_sim_objects, data[u'action'])
                obs_data = record_obs(sc, c,p,am, move_sim_objects, data[u'observation'])

                #mirror motion to another duplicate object for clean observation without the clutter created by the arm
                mirror_motion(move_sim_objects, mirror_shift, copied = 'cube', mirrored = 'cube_m1')

                print("sending file")
                send_msg(conn,obs_data)
                print('Done sending')
            else:
                conn.close()
                reset_sim(am)
                print("closed connection waiting for a new client")
                break

#==========================================================
'''
some challenges to solve
-arm movement is very fast (IK) thats why because of inertia, there is noisy movements sometimes destroying object positions
-socket address already in use problem
'''