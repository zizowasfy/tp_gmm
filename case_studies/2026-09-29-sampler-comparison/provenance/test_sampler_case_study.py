"""Guard equal environment weights, matched successes, and fixed controls."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from summarize_sampler_case_study import MODES,environment_values,paired_values,verify_configuration,verify_row


class SamplerCaseStudy(unittest.TestCase):
    def test_unequal_repeats_do_not_change_environment_weights(self):
        cases=[dict(stratum='a',rows={m:[dict(success=True,repeat=j) for j in range(10)] for m in MODES}),
               dict(stratum='b',rows={m:[dict(success=False,repeat=j) for j in range(5)] for m in MODES})]
        values,strata,count=environment_values(cases,MODES[0],'success')
        self.assertEqual(float(values.mean()),.5)
        self.assertEqual(count,15)
        self.assertEqual(strata,['a','b'])

    def test_quality_uses_only_matched_successes(self):
        a=[dict(repeat=0,success=True,audit={'joint_path_length':4.}),
           dict(repeat=1,success=True,audit={'joint_path_length':100.}),dict(repeat=2,success=False)]
        b=[dict(repeat=0,success=True,audit={'joint_path_length':3.}),dict(repeat=1,success=False),
           dict(repeat=2,success=True,audit={'joint_path_length':200.})]
        cases=[dict(stratum='a',rows={MODES[0]:a,MODES[1]:b})]
        values,_,count=paired_values(cases,*MODES[:2],'audit.joint_path_length')
        np.testing.assert_allclose(values,[1.]);self.assertEqual(count,1)
        b[0]['repeat']=7
        with self.assertRaisesRegex(ValueError,'Repeat order'):paired_values(cases,*MODES[:2],'success')

    def test_rejects_mixed_cohort_configurations(self):
        old=dict(modes=MODES,settings=dict(proposal='gmm',corridor_mode='covariance',cutoff=3.,uniform_fraction=0.,cartesian_fraction=0.),
                 runtime={'ompl.panda_arm.allow_constraint_sampler_fallback':{'bool_value':False}},
                 policy_sha256='policy',environment_ranges={},design={'planning_time':3.})
        verify_configuration(old)
        for key,value in [('cutoff',2.),('covariance_floor',.001)]:
            other=deepcopy(old);other['settings'][key]=value
            with self.assertRaisesRegex(ValueError,'Different cohort'):verify_configuration(other,old)
        other=deepcopy(old);other['runtime']['ompl.panda_arm.allow_constraint_sampler_fallback']['bool_value']=True
        with self.assertRaisesRegex(ValueError,'fallback'):verify_configuration(other)

    def test_purity_and_audit_define_success(self):
        base=dict(mode='joint_projected',repeat=0,sampling=dict(proposal='gmm',corridor_mode='covariance',component_cutoffs=[3.]),
                  success=False,planner_success=False,par2_s=6.,action_wall_s=3.,end_to_end_wall_s=3.1)
        verify_row(base,3.)
        for change in ({'uniform_attempts':1},{'online_ik_calls':1},{'missing_anchor_fallbacks':1},{'component_cutoffs':[2.]}):
            with self.assertRaises(ValueError):verify_row(base | {'sampling':base['sampling'] | change},3.)
        with self.assertRaisesRegex(ValueError,'dense path audit'):
            verify_row(base | dict(success=True,planner_success=True,audit={'audit_invalid_samples':1}),3.)


if __name__=='__main__':unittest.main()
