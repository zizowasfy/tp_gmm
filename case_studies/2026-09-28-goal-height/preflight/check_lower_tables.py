from pathlib import Path
from types import SimpleNamespace
import json,sys
sys.path.insert(0,'src/tp_gmm/scripts')
import rclpy
from moveit_msgs.msg import PlanningScene,RobotState
from moveit_msgs.srv import GetStateValidity
from run_sampling_study import StudyRunner,message,asdict,save
from replay_sampling_study import restore_frozen_scene
rclpy.init();r=StudyRunner(SimpleNamespace(task='franka_pick_cube',ref_robot='native',clearance=.5,visualize=False,sampler_config=None))
preflight=json.loads(Path('sampling_results/2026-09-28-goal-height-preflight/results.json').read_text())['rows']
out=[]
try:
 r.save_scene()
 for top in (0.,-.02,-.05):
  rows=[]
  for old in preflight:
   root=Path(f'sampling_results/2026-09-28-clearance-{old["phase"]}')
   m=json.loads((root/'manifest.json').read_text());content=json.loads((root/m['environments'][old['environment']]['file']).read_text())
   scene=message(PlanningScene,content['scene'])
   for obj in scene.world.collision_objects:
    if obj.id=='tpgmm_benchmark_table':obj.pose.position.z=top-obj.primitives[0].dimensions[2]/2
   verification=restore_frozen_scene(r,scene)
   response=r.service(r.validity_client,GetStateValidity.Request(robot_state=message(RobotState,old['goal_state']),group_name=r.group_name))
   contacts=asdict(response)['contacts']
   rows.append(dict(phase=old['phase'],environment=old['environment'],valid=response.valid,contacts=contacts,scene_verification=verification))
  table=sum(any('tpgmm_benchmark_table' in (c['contact_body_1'],c['contact_body_2']) for c in row['contacts']) for row in rows)
  entry=dict(table_top_z=top,valid=sum(row['valid'] for row in rows),table_contacts=table,rows=rows);out.append(entry)
  save(Path('sampling_results/2026-09-28-goal-height-preflight/lower_table_candidates.json'),out)
  print(top,'valid',entry['valid'],'table contacts',table,flush=True)
  if table==0:break
finally:r.restore_scene();r.destroy_node();rclpy.shutdown()
