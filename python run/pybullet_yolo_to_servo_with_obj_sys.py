# -*- coding: utf-8 -*-
import serial
import serial.tools.list_ports
import sys
# //"C:/Users/student/Desktop/MBS4541/test_rot"
import test_rot
print(serial.tools.list_ports.comports()[0])
UNO=serial.Serial(serial.tools.list_ports.comports()[0][0],115200)
UNO_pose=[90,90]
# import cProfile
# from pstats import SortKey
import pyrealsense2 as rs
import os
import time
import math
import numpy as np
# from PIL import Image
import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import cv2
from ultralytics import YOLO
model = YOLO('C:/Users/student/Desktop/MBS4541/test_pybullet/best.pt')
# import imageio_ffmpeg
# from base64 import b64encode
# from IPython.display import HTML
print()
p.connect(p.DIRECT) #or p.GUI for graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
scale=0.1
spheres_cols=[]
for i in range(64):
    spheres_rows=[]
    for j in range(48):
        sphere = p.loadURDF("cube.urdf",[5,(6.4/2)-scale*i,(4.8)-scale*j],globalScaling=scale)
        p.setCollisionFilterGroupMask(sphere, -1, 0, 0)
        p.changeVisualShape(sphere, -1, rgbaColor=[i/64*255, 0, j/48*255, 1])
        spheres_rows.append(sphere)
    spheres_cols.append(spheres_rows)
    print(i)
# plane_id = p.loadURDF("plane.urdf")
# kuka_id = p.loadURDF("kuka_iiwa/model.urdf")
print(f"{os.path.dirname(__file__)}\cus.urdf"+"-"*100)
kuka_id = p.loadURDF(f"{os.path.dirname(__file__)}\cus.urdf",globalScaling=7)
# cube1_id = p.loadURDF(f"{os.path.dirname(__file__)}\\bottle.urdf", globalScaling=2)
# p.changeVisualShape(cube1_id, -1, rgbaColor=[255,255,0, 1])
depth_offset=1

board_scaling=5
board_pos=[5,0,board_scaling//2]
# board_id = p.loadURDF("cube.urdf", basePosition=board_pos, globalScaling=board_scaling)
# p.setCollisionFilterGroupMask(cube1_id, -1, 0, 0)
num_joints = p.getNumJoints(kuka_id)
kuka_end_effector_idx = 2

# camera parameters
cam_target_pos = [3,0,2]
cam_distance = 15.0
cam_yaw, cam_pitch, cam_roll = -70, -10.0, 0
cam_width, cam_height = 480, 360

cam_up, cam_up_axis_idx, cam_near_plane, cam_far_plane, cam_fov = [0, 0, 1], 2, 0.01, 100, 60
# Initialize the pipeline
pipeline = rs.pipeline()
# Configure the pipeline to stream depth and color
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)
target_pos=None
position_tolerance = 0.05
pos_list=[[],[],[]]
sin_cos_x=0
fig3d, ax3d = plt.subplots(subplot_kw=dict(projection='3d'))
fig2d, ax2d = plt.subplots()
t=0
# tick=time.time()
# cap=cv2.VideoCapture(0,cv2.CAP_DSHOW)
cv2.namedWindow('Color Image')
mouseX,mouseY=640//2,480//2
gX,gY=640//2,480//2
cv2.namedWindow("Aligned Depth Image")
def main():
    global t,UNO_pose,gX,gY
    joint_poses=[0,0,0]
    tardepth=10
    w,h=0,0
    servo_angle=-10,162
    normal_angle=0,180
    # COCO class index for "bottle"
    # bottle_index = 39
    bottle_index = 0
    # Access the bounding boxes and class labels
    # boxes = results[0].boxes.xywh  # Get the bounding boxes in [x, y, width, height] format
    # labels = results[0].boxes.cls   # Get the class labels for each box
    unknown_photo_name=photo_name("C:/Users/student/Desktop/MBS4541/test_pybullet/unknown_photo","jpg")
    while True:
        if (waited_key:=cv2.waitKey(1))==27 or waited_key==ord('q'):break
        # Wait for a new frame
        frames = pipeline.wait_for_frames()
        # for i in dir(rs):
        #     print(i)
        # exit()
        # Get depth frame
        depth_frame = frames.get_depth_frame()
        
        temporal = rs.hole_filling_filter()
        depth_frame = temporal.process(depth_frame)
        depth_frame = rs.spatial_filter().process(depth_frame)
        hole_filling = rs.hole_filling_filter(0)
        depth_frame = hole_filling.process(depth_frame)
        
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        Limg=cv2.Laplacian(color_image, cv2.CV_64F)
        print(f"{Limg.var()=}")
        
        if Limg.var()<100:
            pybullet_cam_to_cv()
            cv2.imshow("Limg",Limg)
            cv2.imshow("img",color_image)
            continue
            
        #vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        aligned_depth_frame = rs.align(rs.stream.color).process(frames).get_depth_frame()
        aligned_depth_image = np.asanyarray(aligned_depth_frame.get_data())
        
        
        
        
        # aligned_depth_image=fill_depth_blind_spots(aligned_depth_image)
        aligned_depth_image[aligned_depth_image == 0]=65535
        # aligned_depth_image = cv2.morphologyEx(aligned_depth_image, cv2.MORPH_OPEN, (100,100))
        # aligned_depth_image=cv2.blur(aligned_depth_image,(10,10))



        
        depth_image = aligned_depth_image
        # 將深度值轉換為可視化格式（例如，歸一化）
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(aligned_depth_image, alpha=0.03), cv2.COLORMAP_JET)

        cv2.setMouseCallback("Aligned Depth Image",
                            lambda e,x,y,f,a:
                                print(
                                    (aligned_depth_image[y][x])
                                    if e==cv2.EVENT_LBUTTONDOWN 
                                    else "",
                                    end="\n"
                                    if e==cv2.EVENT_LBUTTONDOWN 
                                    else "")
                                )
        #^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        
        # Stack both images horizontally
        blur=~cv2.cvtColor(cv2.blur(color_image,(10,10)),cv2.COLOR_RGB2BGR)
        depth_10=cv2.blur(depth_image,(10,10))
        depth_20=cv2.blur(depth_image,(20,20))
        # Create a colorized depth image
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        img = cv2.cvtColor(color_image.copy(), cv2.COLOR_BGR2RGB)
        results = model.predict(img)
        # cv2.imshow("results",cv2.cvtColor(results[0].plot(),cv2.COLOR_RGB2BGR))
        if (waited_key:=cv2.waitKey(1))==27 or waited_key==ord('q'):break
        # Get the confidence scores
        conf_scores = results[0].boxes.conf
        
        # Check if there are any scores available
        if conf_scores.size(0) > 0:
            # Find the first box with label "bottle"
            bottle_indexs=[i for i in range(len(results[0].boxes.cls))if results[0].boxes.cls[i].item() == bottle_index]
            bottle_objs=[]
            for i in bottle_indexs:
                if (CONF:=conf_scores[i].item())<0.75:
                    print((CONF,type(CONF)),(0.75,type(0.75)))
                    continue
                bottle_obj=dict(
                    xywh=results[0].boxes.xywh[i],
                    conf=CONF
                )
                # bottle_obj.setdefault("xywh",results[0].boxes.xywh[i])
                # bottle_obj.setdefault("conf",conf_scores[i])
                bottle_objs.append(bottle_obj)
            
            
            
            for bottle_obj in bottle_objs:
                bottle_box = bottle_obj["xywh"].cpu().numpy()  # Convert to NumPy array
                x,y,w,h=[int(i)for i in bottle_box]
                croped_color_image=color_image.copy()[y-h//2:y+h//2,x-w//2:x+w//2].copy()
                next_unknown_photo_name=next(unknown_photo_name)
                print(next_unknown_photo_name)
                cv2.imwrite(next_unknown_photo_name,croped_color_image)
            # continue
            
            
            bottle_objs = sorted(bottle_objs,key=lambda o:o["conf"],reverse=True)
            for bottle_obj in bottle_objs:
                bottle_box = bottle_obj["xywh"].cpu().numpy()  # Convert to NumPy array
                
                
                
                print(bottle_box)
                x,y,w,h=[int(i)for i in bottle_box]
                croped_depth_20=depth_20.copy()[y-h//2:y+h//2,x-w//2:x+w//2].copy()
                # print(f"{type(croped_depth_20)=}")
                # print(f"{(croped_depth_20*255/32767)=}")
                # croped_depth_20 = cv2.adaptiveThreshold(cv2.convertScaleAbs(croped_depth_20), 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
                # croped_depth_20 = cv2.convertScaleAbs(croped_depth_20.copy())
                
                
                cv2.imshow("croped_depth_20",cv2.applyColorMap(cv2.convertScaleAbs(croped_depth_20.copy(), alpha=0.03), cv2.COLORMAP_JET))
                # print("="*10,croped_depth_20)
                tardepth=(croped_depth_20.copy()).reshape(-1).tolist()
                # print("="*10,croped_depth_20.shape,len(tardepth))
                # print("="*10,croped_depth_20)
                if len(tardepth)>0:
                    # if 65535 in tardepth:tardepth.remove(65535)
                    tardepth=list(filter(lambda a: a <= 4000, tardepth))
                    if len(tardepth)>0:
                        print(max(tardepth))
                        print("-"*10,min(tardepth),"|",max(tardepth))
                        median_depth=np.median(tardepth)
                        # tardepth_list=[]
                        # for d20y in range(len(croped_depth_20)):
                        #     # print(type(croped_depth_20[d20y][0]))
                        #     xlist=[]
                        #     for d20x in range(len(croped_depth_20[d20y])):
                        #         # print(croped_depth_20[d20y][d20x], np.uint16(round(median_depth)))
                        #         if int(croped_depth_20[d20y][d20x])==int(round(median_depth)):
                        #             xlist.append(d20x)
                        #     # print(len(xlist),xlist)
                        #     if len(xlist)>0:
                        #         print(xlist)
                        #         tardepth_list+=[list(z) for z in (zip(xlist,[d20y]*len(xlist)))]
                        # print(tardepth_list)
                        # if len(tardepth_list)>0:
                        #     tar_bbox=cv2.boundingRect(np.int32([tardepth_list]))
                        #     bbimg=np.zeros((480,640),np.uint8)
                        #     cv2.rectangle(bbimg,tar_bbox,255,1)
                        #     bbox_x,bbox_y,bbox_w,bbox_h=tar_bbox
                        #     cv2.circle(bbimg,(bbox_x+bbox_w//2,bbox_y+bbox_h//2),10,255,-1)
                        #     x,y=(bbox_x+bbox_w//2,bbox_y+bbox_h//2)
                        #     cv2.imshow("tar",bbimg)
                        #     print()
                            # input()#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                        # else:
                        # print(">"*10,tardepth,tardepth_list)
                        # tardepth=(median_depth/1000)+depth_offset
                        tardepth=(median_depth/100)+depth_offset
                        break
                # x=round((x+w//2))
                # y=round((y+h//2))
                gX,gY=x,y
                
                
                break  # Exit the loop after finding the first bottle
            else:
                # print("No 'bottle' detected.")
                x,y=gX,gY
            # Get the index of the highest confidence score
            # highest_confidence_index = np.argmax(conf_scores)
            
            # # Get the corresponding bounding box
            # best_box = results[0].boxes.xywh[highest_confidence_index]
            
        else:x,y=gX,gY
        # x,y=mouseX,mouseY
        point=(round(x), round(y))
        # distance = depth_frame.get_distance(*point)
        distance=tardepth*10
        cv2.circle(color_image,point,50,(0,255,0),-1)#<----------------------------------------------------------------------------
        if x>=640:x=639
        if y>=480:y=479
        # tardepth=((depth_20[y][x])/200)+depth_offset
        tarx=(6.4/2)-(scale**2)*x
        tary=(4.8)-(scale**2)*y
        UNO.write("\r".encode())
        read=UNO.readline().decode().split(',')
        UNO_pose=[int(r)for r in read]
        # print("V"*50)
        # print(num_to_range(UNO_pose[0],*normal_angle,*servo_angle)-90)
        # print(UNO_pose[0]-90)
        # print("^"*50)
        target_pos=[tardepth,tarx,tary]
        print(f"{target_pos=}")
        target_pos=\
            np.rot90(test_rot.rot(
                [
                    [1,0,0,target_pos[0]],
                    [0,1,0,target_pos[1]],
                    [0,0,1,target_pos[2]],
                    [0,0,0,1],
                # ],UNO_pose[0]-90,"z",[0,0,0]
                # ],num_to_range(UNO_pose[0],*normal_angle,*servo_angle)-90,"z",[0,0,0]
                ],math.degrees(joint_poses[0]),"z",[0,0,0]
                # ],num_to_range(joint_poses[0],math.radians(0),math.radians(360),math.radians(0+90),math.radians(360-120)),"z",[0,0,0]
            ))[0][0:3]
        target_pos=\
            np.rot90(test_rot.rot(
                [
                    [1,0,0,target_pos[0]],
                    [0,1,0,target_pos[1]],
                    [0,0,1,target_pos[2]],
                    [0,0,0,1],
                # ],360-(math.degrees(joint_poses[1])-90),"y",[0,0,0]
                ],(UNO_pose[1]-90),"y",[0,0,0]
            ))[0][0:3]
        # else:
        #     distance=0
        #     target_pos=[0,0,0]
        # Display the distance
        cv2.putText(color_image, f'Distance: {distance:.2f} mm', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        
        if not (UNO_pose[0] == 0 or UNO_pose[0] == 180 or\
            UNO_pose[1] == 0 or UNO_pose[1] == 180):
            joint_poses = p.calculateInverseKinematics(kuka_id, kuka_end_effector_idx, target_pos)
        tard=1#3
        for i in range(64):
            # break#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            for j in range(48):
                color=(blur[j*10][i*10])
                depth=((depth_10[j*10][i*10])/200)+depth_offset
                # ⁅𝑥=sin(𝑑)𝑦/cos(𝑑) ⁆
                # ⁅𝑦=cos(𝑑)𝑥/sin(𝑑) ⁆
                position = [
                    depth,
                    ((6.4/2)-scale*i)   *(1+math.sin(math.radians(tard))*depth/math.cos(math.radians(tard))),
                    ((4.8)-scale*j)     #*(1+math.cos(math.radians(tard))*depth/math.sin(math.radians(tard)))
                    ]  # x=0, y=0, z=1 (above the plane)
                # print(position)
                # print(
                position=\
                    np.rot90(test_rot.rot(
                        [
                            [1,0,0,position[0]],
                            [0,1,0,position[1]],
                            [0,0,1,position[2]],
                            [0,0,0,1],
                        # ],num_to_range(UNO_pose[0],*normal_angle,*servo_angle)-90,
                        # ],UNO_pose[0]-90,
                        ],math.degrees(joint_poses[0]),
                        "z",
                        [0,0,0]
                    ))[0][0:3]
                position=\
                    np.rot90(test_rot.rot(
                        [
                            [1,0,0,position[0]],
                            [0,1,0,position[1]],
                            [0,0,1,position[2]],
                            [0,0,0,1],
                        ],
                        # UNO_pose[1]-90,
                        360-(math.degrees(joint_poses[1])-90),
                        "y",
                        [0,0,0]
                    ))[0][0:3]
                # )
                # print()
                orientation = [0, 0, 0, 1]  # No rotation
                # Reset cube position and orientation
                p.resetBasePositionAndOrientation(spheres_cols[i][j], position, orientation)
                p.changeVisualShape(spheres_cols[i][j], -1, rgbaColor=[*(color), 1])
                # print(p.getVisualShapeData(spheres_cols[i][j]))
    
    
        # Set the initial position of the cube (x, y, z) and orientation (quaternion)
        position = [0, 0, 1]  # x=0, y=0, z=1 (above the plane)
        orientation = [0, 0, 0, 1]  # No rotation
        # Reset cube position and orientation
        # p.resetBasePositionAndOrientation(cube1_id, [target_pos[0],target_pos[1]+w//200,target_pos[2]-h//200], orientation)
        # draw_local_axes(cube1_id)
        for j in range (num_joints):
            p.setJointMotorControl2(bodyIndex=kuka_id, jointIndex=j, controlMode=p.POSITION_CONTROL, targetPosition=joint_poses[j])
        
        end_effector_state = p.getLinkState(kuka_id, kuka_end_effector_idx)
        # print(end_effector_state)
        current_pos = end_effector_state[0]  # 当前末端位置
        # 检查末端执行器是否接近目标位置
        if (abs(current_pos[0] - target_pos[0]) < position_tolerance and
            abs(current_pos[1] - target_pos[1]) < position_tolerance and
            abs(current_pos[2] - target_pos[2]) < position_tolerance):
            # end_effector_state = p.getLinkState(kuka_id, kuka_end_effector_idx)
            # # print(end_effector_state)
            # current_pos = end_effector_state[0]  # 当前末端位置
            t+=1
        for _ in range(10):
            p.stepSimulation()
        # print(f'timestep {t}...,____{round(sin_cos_x,4)=},____{target_pos=}........', end='\r')
        print(f'timestep {t}..,___{target_pos=}.{list(map(math.degrees,joint_poses))=}', end='\r')
        if t % 8 == 0 or 1: # PyBullet default simulation time step is 240fps.
            # num_to_range(UNO_pose[0],*normal_angle,*servo_angle)
            
            # f"x={90+num_to_range(math.degrees(joint_poses[0]),*servo_angle,*normal_angle)}\r"
            # f"x={90+math.degrees(joint_poses[0])}\r"
            to_write_x=constrain(math.floor(90+math.degrees(joint_poses[0])),0,180)
            to_write_y=constrain(math.floor(180-math.degrees(joint_poses[1])),0,180)
            to_write= \
            f"x={to_write_x}\r"+\
            f"y={to_write_y}\r"
            print(f"{to_write=}")
            
            UNO.write((to_write).encode("utf-8"))
            while True:#make sure the last angle of servo is equal to pybullet one
                read=UNO.readline().decode().split(',')
                UNO_pose=[int(r)for r in read]                                                             #make it same format to compare
                to_write_int=[int(I)for I in [i.strip('x=').strip('y=')for i in to_write.split("\r")][0:2]]#make it same format to compare
                print(UNO_pose,to_write_int,end="\r")
                if UNO_pose == to_write_int:break
                UNO.write(("\r").encode())
            # 顯示 RGB 和對齊的深度圖像
            cv2.imshow("results",cv2.cvtColor(results[0].plot(),cv2.COLOR_RGB2BGR))
            cv2.imshow('Color Image', color_image)
            cv2.imshow('Aligned Depth Image', depth_colormap)
            pybullet_cam_to_cv()
            


    # plt.imshow(Image.fromarray(image)) # show the last frame

    # vid.close()
    #video.release()
    p.disconnect()
    exit()
    # Play recorded video

    #os.system(f"ffmpeg -y -i vid.avi -vcodec libx264 vidc.mp4") # convert to mp4 to show in browser
    # mp4 = open('vid.mp4', 'rb').read()
    # data_url = "data:video/mp4;base64," + b64encode(mp4).decode()
    # HTML('<video width=480 controls><source src="%s" type="video/mp4"></video>' % data_url)
    mp4=cv2.VideoCapture("vid.mp4")
    s=1
    while s:
        if (waited_key:=cv2.waitKey(1))==27 or waited_key==ord('q'):break
        s,frame=mp4.read()
        cv2.imshow("frame",frame)
def pybullet_cam_to_cv():
    cam_view_matrix = p.computeViewMatrixFromYawPitchRoll(cam_target_pos, cam_distance, cam_yaw, cam_pitch, cam_roll, cam_up_axis_idx)
    cam_projection_matrix = p.computeProjectionMatrixFOV(cam_fov, cam_width*1./cam_height, cam_near_plane, cam_far_plane)
    images = p.getCameraImage(cam_width, cam_height, cam_view_matrix, cam_projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    #video.write(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    # Access the color and depth images
    color_image = np.array(images[2],np.uint8)  # Color image (RGB)
    color_image = color_image.reshape((cam_height, cam_width, 4))
    # depth_image = color_image[:, :, 3] 
    color_image = color_image[:, :, :3]  # Keep only R, G, B channels
    color_image = cv2.cvtColor(color_image,cv2.COLOR_RGB2BGR)
    # print(f"{color_image.shape=}")
    # print(f"{depth_image.shape=}")
    f=2
    cv2.imshow('pybullet_cam', cv2.resize(color_image,(0,0),fx=f,fy=f))# vid.send(np.ascontiguousarray(image))
    # cv2.imshow("depth_image",depth_image)


# Function to draw local axes
def draw_local_axes(object_id):
    # Get the object's position and orientation
    position, orientation = p.getBasePositionAndOrientation(object_id)
    
    # Convert orientation to a rotation matrix
    rot_matrix = p.getMatrixFromQuaternion(orientation)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    
    # Define the length of the axes
    length = 0.2
    
    # Define the local axes based on the rotation matrix
    x_axis = position + rot_matrix[:, 0] * length
    y_axis = position + rot_matrix[:, 1] * length
    z_axis = position + rot_matrix[:, 2] * length
    
    # Draw the axes using drawLine
    p.addUserDebugLine(position, x_axis, lineColorRGB=[1, 0, 0], lineWidth=2)  # Red for X
    p.addUserDebugLine(position, y_axis, lineColorRGB=[0, 1, 0], lineWidth=2)  # Green for Y
    p.addUserDebugLine(position, z_axis, lineColorRGB=[0, 0, 1], lineWidth=2)  # Blue for Z


def num_to_range(num, inMin, inMax, outMin, outMax):
    return outMin + (float(num - inMin) / float(inMax - inMin) * (outMax- outMin))

def photo_name(dir:str,jpg_png:str):
    index=0
    while True:
        yield f"{dir}/{round(time.time())}{index}.{jpg_png}"
        index+=1

def constrain(amt,low,high):
    return((low)if(amt)<(low)else((high)if(amt)>(high)else(amt)))


def xy(e,x,y,f,p):
    global mouseX,mouseY
    if e==cv2.EVENT_LBUTTONDOWN:
        mouseX,mouseY=x,y
cv2.setMouseCallback('Color Image',xy)


if __name__ == "__main__":
    # cProfile.run("main()",sort=SortKey.CUMULATIVE) # yolo have so many data at the end
    main()