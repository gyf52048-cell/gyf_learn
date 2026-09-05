import sys,json,collections,pathlib,math
import numpy as np,cv2,rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
p=pathlib.Path(sys.argv[1]);r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(p/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));types={t.name:get_message(t.type) for t in r.get_all_topics_and_types()};counts=collections.Counter();ts=collections.defaultdict(list);positions=[];states=[];lands=[];images=[];poses=[];valid=0;acks=[]
while r.has_next():
 t,b,stamp=r.read_next();m=deserialize_message(b,types[t]);counts[t]+=1;ts[t].append(stamp)
 if t.endswith('vehicle_local_position_v1'):positions.append([stamp,m.timestamp,m.x,m.y,m.z,m.vx,m.vy,m.vz])
 elif t.endswith('vehicle_status_v4'):states.append([stamp,m.arming_state,m.nav_state,m.failsafe])
 elif t.endswith('vehicle_land_detected'):lands.append([stamp,m.landed])
 elif t.endswith('vehicle_command_ack_v1'):acks.append([m.command,m.result])
 elif t.endswith('/image'):
  a=np.asarray(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3);images.append([stamp,m.header.stamp.sec+m.header.stamp.nanosec/1e9,int(a.min()),int(a.max()),float(a.std())])
  if a.std()>15 and not (p/'verified_bag_image.png').exists():cv2.imwrite(str(p/'verified_bag_image.png'),cv2.cvtColor(a,cv2.COLOR_RGB2BGR))
 elif t.endswith('pose_camera'):poses.append([m.pose.position.x,m.pose.position.y,m.pose.position.z])
 elif t.endswith('/detection'):valid+=json.loads(m.data)['valid']
a=np.array(positions);im=np.array(images);st=np.array(states);po=np.array(poses)
report={'counts':dict(counts),'duration_wall_s':(max(v[-1] for v in ts.values())-min(v[0] for v in ts.values()))/1e9,'topic_timestamp_regressions':{t:int(sum(np.diff(v)<0)) for t,v in ts.items()},'position_first':positions[0],'position_last':positions[-1],'max_height_relative_start_m':float(a[0,4]-a[:,4].min()),'position_velocity_all_finite':bool(np.isfinite(a).all()),'arming_states':sorted(set(st[:,1].tolist())),'nav_states':sorted(set(st[:,2].tolist())),'any_failsafe':bool(st[:,3].any()),'land_first_last':[lands[0],lands[-1]],'airborne_samples':sum(not x[1] for x in lands),'image_frames':len(images),'nonuniform_image_frames':sum(x[4]>1 for x in images),'image_sim_duration_s':float(im[-1,1]-im[0,1]),'image_mean_wall_fps':(len(images)-1)/((im[-1,0]-im[0,0])/1e9),'image_max_wall_gap_s':float(np.diff(im[:,0]).max()/1e9),'valid_visual_detections':valid,'target_translation_min_m':po.min(axis=0).tolist() if len(po) else [],'target_translation_max_m':po.max(axis=0).tolist() if len(po) else [],'command_acks':acks}
report['complete_flight_verified']=bool(1 in st[:,1] and 2 in st[:,1] and st[-1,1]==1 and lands[0][1] and lands[-1][1] and report['airborne_samples']>0 and report['max_height_relative_start_m']>2.5 and valid>0 and not report['any_failsafe'])
(p/'validation.json').write_text(json.dumps(report,indent=2));np.savetxt(p/'position_velocity.csv',a,delimiter=',',header='bag_unix_ns,px4_us,x_north_m,y_east_m,z_down_m,vx_mps,vy_mps,vz_mps');print(json.dumps(report,indent=2))
