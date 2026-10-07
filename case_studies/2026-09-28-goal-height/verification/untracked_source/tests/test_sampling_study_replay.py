"""Guard the clearance-only intervention and original paired schedule."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from replay_sampling_study import clearance_mapping, check_replay, model_payload, restore_frozen_scene, PURE, MODES
from moveit_msgs.msg import PlanningScene, CollisionObject

class ReplayTests(unittest.TestCase):
    def test_scene_restoration_verifies_live_geometry(self):
        target=PlanningScene(robot_model_name='panda')
        target.world.collision_objects=[CollisionObject(id='obstacle')]
        stale=deepcopy(target);stale.world.collision_objects[0].pose.position.x=1.
        class Runner:
            def __init__(self,accept_diff):
                self.accept_diff=accept_diff;self.observed=deepcopy(stale)
            def apply_scene(self,scene):
                # Reproduce a full-scene service that reports success but leaves
                # the monitored child stale. The explicit diff must correct it.
                if scene.is_diff and self.accept_diff:self.observed=deepcopy(scene)
            def scene_snapshot(self):return deepcopy(self.observed)
        result=restore_frozen_scene(Runner(True),target)
        self.assertEqual(result['expected_sha256'],result['actual_sha256'])
        with self.assertRaisesRegex(RuntimeError,'Live collision scene differs'):
            restore_frozen_scene(Runner(False),target)

    def test_two_levels_preserve_source_assignment(self):
        self.assertEqual(clearance_mapping([.2,.8],[.5,.7]),{.2:.5,.8:.7})
        self.assertEqual(clearance_mapping([.2,.8],[.6]),{.2:.6,.8:.6})
        for targets in ([],[.5,.6,.7],[float('nan')],[-.1],[1.1]):
            with self.assertRaises(ValueError):clearance_mapping([.2,.8],targets)

    def test_prior_identity_ignores_only_header_time(self):
        model={'header':{'frame_id':'world','stamp':{'sec':1}},'gaussians':[{'means':[1,2,3,4]}],'weights':[1]}
        other=deepcopy(model);other['header']['stamp']['sec']=2
        self.assertEqual(model_payload(model),model_payload(other))
        other['gaussians'][0]['means'][1]=3
        self.assertNotEqual(model_payload(model),model_payload(other))

    def test_resume_requires_complete_matching_schedule_and_settings(self):
        source=dict(status='complete',settings=PURE,modes=MODES,runtime={'strict':False},design={'repeats':2},environments=[{'id':0}],
                    schedule=[dict(environment=0,repeat=i,mode=m) for i in range(2) for m in MODES])
        check_replay(source,source['runtime'])
        for changed in (source | {'schedule':source['schedule'][:-1]},source | {'status':'running'},
                        source | {'settings':PURE | {'uniform_fraction':.1}}):
            with self.assertRaises(ValueError):check_replay(changed,source['runtime'])

if __name__=='__main__':unittest.main()
