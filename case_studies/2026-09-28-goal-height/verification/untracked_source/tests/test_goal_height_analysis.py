"""Guard paired scene identity and environment weighting in the height analysis."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_goal_height_study import verify_pair,summarize,MODES


class HeightAnalysis(unittest.TestCase):
    def test_rejects_unintended_scene_and_case_changes(self):
        control=dict(start={'joints':[1,2,3]},case=dict(goal=[.5,0.,.495],obstacle=[.4,0.,.4]),
            scene=dict(world=dict(collision_objects=[dict(id='tpgmm_benchmark_table',
                pose=dict(position=dict(x=.6,y=0.,z=.125)),primitives=[dict(dimensions=[.75,1.,.25])]),
                dict(id='cylinder',position=[.4,0.,.4])])),response=dict(policy_sha256='checkpoint',action_scale=.15))
        training=deepcopy(control)
        training['case']['goal'][2]=.2
        training['scene']['world']['collision_objects'][0]['pose']['position']['z']=-.145
        intervention=dict(source_goal_z_range=[.45,.54],training_goal_z_range=[.1,.3],table_top_z=-.02)
        saved=deepcopy(training)
        verify_pair(control,training,intervention)
        self.assertEqual(training,saved)
        for key in ('cylinder','goal_xy','policy','start'):
            changed=deepcopy(training)
            if key=='cylinder':changed['scene']['world']['collision_objects'][1]['position'][0]+=.01
            elif key=='goal_xy':changed['case']['goal'][0]+=.01
            elif key=='policy':changed['response']['policy_sha256']='different'
            else:changed['start']['joints'][0]+=.01
            with self.assertRaises(ValueError):verify_pair(control,changed,intervention)

    def test_equal_environment_weight_and_design_failures(self):
        def rows(n,success,attempted=True):
            row=dict(success=success,failure_stage=None if success else 'goal_ik_design')
            if attempted:row['error_code']=1
            return [row.copy() for _ in range(n)]
        cases=[dict(stratum='a',rows={m:(rows(10,False,False),rows(10,True)) for m in MODES}),
               dict(stratum='b',rows={m:(rows(5,True),rows(5,False,False)) for m in MODES})]
        for r in summarize(cases,np.random.default_rng(9)):
            self.assertEqual((r['control_successes'],r['training_successes'],r['requests']),(5,10,15))
            self.assertEqual((r['difference'],r['p']),(0.,1.))
            self.assertEqual((r['control_action_requests'],r['training_action_requests']),(5,10))
            self.assertEqual((r['recovered_pairs'],r['lost_pairs']),(10,5))
            self.assertEqual(r['training_failures'],{'goal_ik_design':5})


if __name__=='__main__':unittest.main()
