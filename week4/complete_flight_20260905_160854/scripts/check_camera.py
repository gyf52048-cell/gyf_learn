import time,json,pathlib
import numpy as np,cv2,rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleLocalPosition,VehicleStatus,VehicleLandDetected
rclpy.init(); n=rclpy.create_node('week4_camera_check'); start=time.monotonic(); rows=[]; state={}
out=pathlib.Path('/home/drone/week4_bags/checks'); out.mkdir(exist_ok=True)
def image(m):
 a=np.asarray(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
 rows.append({'wall':time.monotonic()-start,'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'min':int(a.min()),'max':int(a.max()),'std':float(a.std())})
 if len(rows)==1 or len(rows)%30==0: cv2.imwrite(str(out/'camera_headless_latest.png'),cv2.cvtColor(a,cv2.COLOR_RGB2BGR))
def status(m): state['status']={k:getattr(m,k) for k in ['arming_state','nav_state','failsafe']}
def pos(m): state['position']={k:getattr(m,k) for k in ['x','y','z','vx','vy','vz','xy_valid','z_valid']}
def land(m):state['landed']=m.landed
n.create_subscription(Image,'/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image',image,qos_profile_sensor_data)
n.create_subscription(VehicleStatus,'/fmu/out/vehicle_status_v4',status,qos_profile_sensor_data)
n.create_subscription(VehicleLocalPosition,'/fmu/out/vehicle_local_position_v1',pos,qos_profile_sensor_data)
n.create_subscription(VehicleLandDetected,'/fmu/out/vehicle_land_detected',land,qos_profile_sensor_data)
while time.monotonic()-start<30:rclpy.spin_once(n,timeout_sec=.2)
r={'duration':time.monotonic()-start,'frames':len(rows),'state':state,'samples':rows}
if len(rows)>1:r.update(fps=(len(rows)-1)/(rows[-1]['wall']-rows[0]['wall']),max_gap=max(np.diff([x['wall'] for x in rows])))
(out/'camera_headless_check.json').write_text(json.dumps(r,indent=2));print(json.dumps({k:v for k,v in r.items() if k!='samples'},indent=2));n.destroy_node();rclpy.shutdown()
