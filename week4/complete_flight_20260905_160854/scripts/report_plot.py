import pathlib,json,numpy as np,rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=pathlib.Path('/home/drone/week4_bags/complete_flight_20260905_160854');a=np.loadtxt(p/'position_velocity.csv',delimiter=',');t=(a[:,0]-a[0,0])/1e9
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(p/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));types={v.name:get_message(v.type) for v in r.get_all_topics_and_types()};poses=[];images=[];att=[]
while r.has_next():
 topic,b,s=r.read_next()
 if topic.endswith('pose_camera'):
  m=deserialize_message(b,types[topic]);poses.append([(s-a[0,0])/1e9,m.pose.position.x,m.pose.position.y,m.pose.position.z])
 elif topic.endswith('/image'):
  m=deserialize_message(b,types[topic]);images.append([(s-a[0,0])/1e9,m.header.stamp.sec+m.header.stamp.nanosec/1e9])
 elif topic.endswith('vehicle_attitude'):
  m=deserialize_message(b,types[topic]);att.append([m.timestamp,*m.q])
b=np.array(poses);im=np.array(images);q=np.array(att);summary=json.load(open(p/'capture_summary.json'));ev={e['kind']:e['wall_elapsed'] for e in summary['events']};first_delay=summary['duration_wall']-t[-1]
hover_im=im[(im[:,0]>=ev['hover_start'])&(im[:,0]<=ev['hover_end'])]
extra={'quaternion_norm_min':float(np.linalg.norm(q[:,1:],axis=1).min()),'quaternion_norm_max':float(np.linalg.norm(q[:,1:],axis=1).max()),'attitude_source_timestamp_regressions':int(sum(np.diff(q[:,0])<0)),'position_source_timestamp_regressions':int(sum(np.diff(a[:,1])<0)),'image_source_timestamp_regressions':int(sum(np.diff(im[:,1])<0)),'hover_wall_s':ev['hover_end']-ev['hover_start'],'hover_image_sim_span_s':float(hover_im[-1,1]-hover_im[0,1]),'overall_image_sim_to_wall_ratio':float((im[-1,1]-im[0,1])/(im[-1,0]-im[0,0]))}
(p/'timing_validation.json').write_text(json.dumps(extra,indent=2));np.savetxt(p/'target_camera.csv',b,delimiter=',',header='bag_elapsed_wall_s,target_right_m,target_down_m,target_forward_m')
fig,axes=plt.subplots(3,1,figsize=(10,9),sharex=True);axes[0].plot(t,a[0,4]-a[:,4],label='PX4 height above initial estimate');axes[0].plot(b[:,0],b[:,3],'.',ms=3,label='Visual target camera depth');axes[0].set_ylabel('Distance (m)');axes[0].legend();axes[1].plot(t,a[:,5:8],label=['v North','v East','v Down']);axes[1].set_ylabel('Velocity (m/s)');axes[1].legend();axes[2].plot(b[:,0],b[:,1:3],'.',label=['Target camera right','Target camera down']);axes[2].set_ylabel('Offset (m)');axes[2].set_xlabel('Bag reception elapsed wall time (s)');axes[2].legend()
for ax in axes:ax.grid(True,alpha=.3)
fig.suptitle('Week 4 SITL flight: takeoff, hover, landing\nVisual and PX4 samples use receipt time; not hardware-synchronized');fig.tight_layout();fig.savefig(p/'flight_overview.png',dpi=150);print(json.dumps(extra,indent=2))
