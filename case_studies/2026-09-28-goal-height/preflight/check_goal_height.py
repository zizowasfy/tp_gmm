from pathlib import Path
from types import SimpleNamespace
import json,sys,hashlib
sys.path.insert(0,'src/tp_gmm/scripts')
import rclpy
from moveit_msgs.msg import PlanningScene,RobotState
from moveit_msgs.srv import GetPositionIK,GetStateValidity
from run_sampling_study import StudyRunner,message,asdict,save
from replay_sampling_study import restore_frozen_scene
from compare_sampling_approaches import pose
rclpy.init();runner=StudyRunner(SimpleNamespace(task='franka_pick_cube',ref_robot='native',clearance=.5,visualize=False,sampler_config=None))
results=[]
try:
 runner.save_scene()
 for phase in ('primary','confirmation'):
  root=Path(f'sampling_results/2026-09-28-clearance-{phase}')
  manifest=json.loads((root/'manifest.json').read_text())
  for env in manifest['environments']:
   content=json.loads((root/env['file']).read_text())
   verification=restore_frozen_scene(runner,message(PlanningScene,content['scene']))
   target=list(content['case']['goal']);target[2]=.1+(target[2]-.45)/(.54-.45)*.2
   req=GetPositionIK.Request();req.ik_request.group_name=runner.group_name;req.ik_request.ik_link_name=runner.ee_link
   req.ik_request.pose_stamped=pose(target,runner.frame_id);req.ik_request.avoid_collisions=False
   req.ik_request.timeout.sec=2;req.ik_request.robot_state=message(RobotState,content['goal'])
   ik=runner.service(runner.ik_client,req)
   row=dict(phase=phase,environment=env['id'],clearance=env['clearance'],original_goal=content['case']['goal'],goal=target,ik_error=ik.error_code.val,scene_verification=verification)
   if ik.error_code.val==1:
    valid=runner.service(runner.validity_client,GetStateValidity.Request(robot_state=ik.solution,group_name=runner.group_name))
    row.update(valid=valid.valid,contacts=asdict(valid)['contacts'],goal_state=asdict(ik.solution))
   else:row.update(valid=False,contacts=[])
   results.append(row)
   save(Path('sampling_results/2026-09-28-goal-height-preflight/results.json'),dict(goal_z_range=[.1,.3],mapping='Preserve old z quantile from [.45,.54] in [.1,.3]; all other Cartesian geometry unchanged',rows=results))
   print(phase,env['id'],round(target[2],4),row['valid'],[(c['contact_body_1'],c['contact_body_2']) for c in row['contacts']],flush=True)
finally:
 runner.restore_scene();runner.destroy_node();rclpy.shutdown()
