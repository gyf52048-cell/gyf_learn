"""Existing PX4 SITL only: record raw telemetry/images and visual target, take off 3m, hover, land."""
import json, math, time, pathlib, traceback
import cv2,numpy as np,rclpy,rosbag2_py
from rclpy.serialization import serialize_message
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from px4_msgs.msg import VehicleAttitude,VehicleLocalPosition,VehicleStatus,VehicleLandDetected,VehicleGlobalPosition,VehicleCommand,VehicleCommandAck
ROOT=pathlib.Path('/home/drone/week4_bags'); run=ROOT/('complete_flight_'+time.strftime('%Y%m%d_%H%M%S')); run.mkdir()
rclpy.init(); n=rclpy.create_node('week4_flight_capture'); w=rosbag2_py.SequentialWriter();w.open(rosbag2_py.StorageOptions(uri=str(run/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
counts={}; latest={}; stage='initial'; events=[]; start=time.monotonic(); last_image=None; detections=0
base='/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/'
def register(topic,typ):
 w.create_topic(rosbag2_py.TopicMetadata(id=len(counts),name=topic,type=typ,serialization_format='cdr'));counts[topic]=0
def write(topic,msg):
 w.write(topic,serialize_message(msg),time.time_ns());counts[topic]+=1
def event(kind,**kw):
 e=dict(wall_elapsed=time.monotonic()-start,kind=kind,**kw);events.append(e);print(json.dumps(e),flush=True)
def sub(topic,cls,key):
 register(topic,'px4_msgs/msg/'+cls.__name__)
 def cb(m):latest[key]=m;write(topic,m)
 n.create_subscription(cls,topic,cb,qos_profile_sensor_data)
for topic,cls,key in [('/fmu/out/vehicle_attitude',VehicleAttitude,'att'),('/fmu/out/vehicle_local_position_v1',VehicleLocalPosition,'pos'),('/fmu/out/vehicle_status_v4',VehicleStatus,'status'),('/fmu/out/vehicle_land_detected',VehicleLandDetected,'land'),('/fmu/out/vehicle_global_position',VehicleGlobalPosition,'global'),('/fmu/out/vehicle_command_ack_v1',VehicleCommandAck,'ack')]:sub(topic,cls,key)
register(base+'image','sensor_msgs/msg/Image');register('/week4/target/pose_camera','geometry_msgs/msg/PoseStamped');register('/week4/target/detection','std_msgs/msg/String');register('/week4/camera_info','sensor_msgs/msg/CameraInfo');register('/fmu/in/vehicle_command','px4_msgs/msg/VehicleCommand')
pubs={t:n.create_publisher(c,t,10) for t,c in [('/week4/target/pose_camera',PoseStamped),('/week4/target/detection',String),('/week4/camera_info',CameraInfo)]}
cmdpub=n.create_publisher(VehicleCommand,'/fmu/in/vehicle_command',10)
dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50);f=1280/(2*math.tan(1.74/2));K=np.array([[f,0,640],[0,f,480],[0,0,1]],float)
def publish(t,m):pubs[t].publish(m);write(t,m)
def image(m):
 global last_image,detections
 write(base+'image',m);last_image=time.monotonic()
 a=np.asarray(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
 gray=cv2.cvtColor(a,cv2.COLOR_RGB2GRAY);corners,ids,_=cv2.aruco.detectMarkers(gray,dictionary)
 info=CameraInfo();info.header=m.header;info.header.frame_id='camera_optical_frame';info.width=m.width;info.height=m.height;info.distortion_model='plumb_bob';info.d=[0.]*5;info.k=K.flatten().tolist();info.r=np.eye(3).flatten().tolist();info.p=[f,0.,640.,0.,0.,f,480.,0.,0.,0.,1.,0.];publish('/week4/camera_info',info)
 detail={'stamp_sec':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'valid':False,'dictionary':'DICT_4X4_50','id':0,'marker_size_m':.5,'frame':'camera_optical_frame','source':'visual_estimate','pixel_min':int(a.min()),'pixel_max':int(a.max())}
 if ids is not None and 0 in ids.flatten():
  c=corners[list(ids.flatten()).index(0)];rv,tv,_=cv2.aruco.estimatePoseSingleMarkers(c,.5,K,np.zeros(5));v=tv[0,0];R,_=cv2.Rodrigues(rv[0,0]);# quaternion from rotation matrix via robust eigen formulation
  from scipy.spatial.transform import Rotation
  q=Rotation.from_matrix(R).as_quat();p=PoseStamped();p.header=m.header;p.header.frame_id='camera_optical_frame';p.pose.position.x=float(v[0]);p.pose.position.y=float(v[1]);p.pose.position.z=float(v[2]);p.pose.orientation.x=float(q[0]);p.pose.orientation.y=float(q[1]);p.pose.orientation.z=float(q[2]);p.pose.orientation.w=float(q[3]);publish('/week4/target/pose_camera',p)
  detail.update(valid=True,translation_m=v.tolist());detections+=1
  if detections==1 or detections%50==0:cv2.imwrite(str(run/'camera_target.png'),cv2.cvtColor(a,cv2.COLOR_RGB2BGR))
 s=String();s.data=json.dumps(detail);publish('/week4/target/detection',s)
n.create_subscription(Image,base+'image',image,qos_profile_sensor_data)
def command(code,**params):
 m=VehicleCommand();m.timestamp=int(time.time()*1e6);m.command=code;m.target_system=1;m.target_component=1;m.source_system=255;m.source_component=190;m.from_external=True
 for k,v in params.items():setattr(m,k,float(v))
 cmdpub.publish(m);write('/fmu/in/vehicle_command',m);event('command',command=code,params=params)
def spin():rclpy.spin_once(n,timeout_sec=.1)
def wait(pred,timeout):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  spin()
  if pred():return True
 return False
try:
 if not wait(lambda:all(k in latest for k in ['pos','status','land','global']) and counts[base+'image']>=5,30):raise RuntimeError('Missing live sensor/telemetry data')
 if latest['status'].arming_state!=1 or not latest['land'].landed:raise RuntimeError('Expected disarmed landed SITL before flight')
 if not latest['status'].pre_flight_checks_pass:raise RuntimeError('Preflight checks not passed')
 event('baseline',z=latest['pos'].z);z0=latest['pos'].z
 command(400,param1=1.)
 if not wait(lambda:latest['status'].arming_state==2,15):raise RuntimeError('Arm not confirmed')
 command(22,param4=float('nan'),param5=float('nan'),param6=float('nan'),param7=latest['global'].alt+3.)
 if not wait(lambda:z0-latest['pos'].z>2.5 and abs(latest['pos'].vz)<.35,100):raise RuntimeError('Takeoff/hover altitude not reached')
 event('hover_start',z=latest['pos'].z);stamp=latest['pos'].timestamp
 if not wait(lambda:latest['pos'].timestamp-stamp>20_000_000,100):raise RuntimeError('Hover clock timeout')
 event('hover_end',detections=detections)
 command(21,param4=float('nan'),param5=float('nan'),param6=float('nan'))
 if not wait(lambda:latest['land'].landed and latest['status'].arming_state==1,120):raise RuntimeError('Landing/disarm not confirmed')
 event('landed_disarmed',z=latest['pos'].z);wait(lambda:False,5);event('complete')
except BaseException as e:
 event('error',error=str(e));traceback.print_exc()
 if 'status' in latest and latest['status'].arming_state==2:
  command(21,param4=float('nan'),param5=float('nan'),param6=float('nan'));wait(lambda:latest.get('land') and latest['land'].landed and latest['status'].arming_state==1,120)
finally:
 (run/'capture_summary.json').write_text(json.dumps(dict(events=events,counts=counts,detections=detections,duration_wall=time.monotonic()-start,intrinsics=K.tolist(),marker_size_m=.5,bag_timestamp='host Unix reception time ns',image_timestamp='Gazebo simulation time',pose_frame='camera optical: x right, y down, z forward; target relative to camera; visual estimate'),indent=2));del w;n.destroy_node();rclpy.shutdown();print('OUTPUT='+str(run),flush=True)
