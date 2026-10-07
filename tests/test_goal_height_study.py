from copy import deepcopy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_goal_height_study import mapped_goal_z,lower_table

class HeightStudyGeometry(unittest.TestCase):
    def test_quantile_mapping_changes_only_height_range(self):
        for before,after in ((.45,.1),(.495,.2),(.54,.3)):
            self.assertAlmostEqual(mapped_goal_z(before,[.45,.54],[.1,.3]),after)
            self.assertEqual(mapped_goal_z(before,[.45,.54],[.45,.54]),before)
        for before,old,new in ((.6,[.45,.54],[.1,.3]),(.5,[.5,.5],[.1,.3]),(.5,[.45,.54],[.3,.1])):
            with self.assertRaises(ValueError):mapped_goal_z(before,old,new)

    def test_clone_lowers_only_table_and_preserves_source(self):
        table=dict(id='tpgmm_benchmark_table',pose=dict(position=dict(x=.6,y=0.,z=.125),orientation=dict(x=0.,y=0.,z=0.,w=1.)),
                   primitives=[dict(type=1,dimensions=[.75,1.,.25])],primitive_poses=[dict(position=dict(x=0.,y=0.,z=0.))])
        scene=dict(world=dict(collision_objects=[table,dict(id='cylinder',pose=dict(x=.4,y=.1,z=.3))]))
        original=deepcopy(scene);changed=lower_table(scene,-.02)
        self.assertEqual(scene,original)
        self.assertEqual(changed['world']['collision_objects'][1],scene['world']['collision_objects'][1])
        self.assertEqual(changed['world']['collision_objects'][0]['pose']['position'],dict(x=.6,y=0.,z=-.145))
        changed['world']['collision_objects'][0]['pose']['position']['z']=.125
        self.assertEqual(changed,original)

if __name__=='__main__':unittest.main()
