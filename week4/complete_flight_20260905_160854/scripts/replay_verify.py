import os,time,json,pathlib,subprocess,collections
os.environ['ROS_DOMAIN_ID']='74'
import rclpy,rosbag2_py
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosbag2_interfaces.srv import GetRate
replay_qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE)
from rosidl_runtime_py.utilities import get_message
p=pathlib.Path('/home/drone/week4_bags/complete_flight_20260905_160854');r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(p/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));topics=r.get_all_topics_and_types();del r
rclpy.init();n=rclpy.create_node('week4_replay_verifier');counts=collections.Counter();images={'nonuniform':0};subscriptions=[]
def cb(t,m):
 counts[t]+=1
 if t.endswith('/image'):
  import numpy as np
  a=np.asarray(m.data,dtype=np.uint8);images['nonuniform']+=int(a.max()!=a.min())
for t in topics:
 if t.name.startswith('/fmu/in/'):continue
 subscriptions.append(n.create_subscription(get_message(t.type),t.name,lambda m,t=t.name:cb(t,m),replay_qos))
f=open(p/'replay.log','w');proc=subprocess.Popen(['ros2','bag','play',str(p/'bag'),'--exclude-topics','/fmu/in/vehicle_command','--disable-keyboard-controls','--delay','3','--rate','1','--wait-for-all-acked','5000'],stdout=f,stderr=subprocess.STDOUT)
start=time.monotonic();graph=None;client=n.create_client(GetRate,'/rosbag2_player/get_rate');future=None
while proc.poll() is None and time.monotonic()-start<130:
 rclpy.spin_once(n,timeout_sec=.1)
 if future is None and client.service_is_ready():future=client.call_async(GetRate.Request())
 if graph is None and time.monotonic()-start>6:graph={'nodes':n.get_node_names(),'services':n.get_service_names_and_types()}
if proc.poll() is None:proc.terminate()
proc.wait();end=time.monotonic()+2
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
report={'domain_id':74,'excluded_control_topic':'/fmu/in/vehicle_command','player_exit_code':proc.returncode,'received_counts':dict(counts),'images':images,'graph':graph,'get_rate_service_result':future.result().rate if future is not None and future.done() and future.result() is not None else None,'subscription_qos':'RELIABLE depth 100'};expected={k:v for k,v in json.load(open(p/'validation.json'))['counts'].items() if not k.startswith('/fmu/in/')};report['all_expected_counts_match']=expected==dict(counts);(p/'replay_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown();f.close()
