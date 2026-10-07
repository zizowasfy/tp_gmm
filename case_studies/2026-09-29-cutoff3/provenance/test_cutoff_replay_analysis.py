"""Cutoff attribution requires unchanged inputs and equal environment weights."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from compare_cutoff_replays import MODES,verify_controls,verify_environment,verify_purity,summarize


class CutoffReplayAnalysis(unittest.TestCase):
    def test_rejects_changes_beyond_cutoff(self):
        old={k:[] for k in ('schedule','design','runtime','modes','environment_ranges')}
        old.update(policy_sha256='policy',settings=dict(cutoff=2.,uniform_fraction=0.))
        new=deepcopy(old)
        new['settings']['cutoff']=3.
        new['replay']=dict(kind='cutoff',source_cutoff=2.,cutoff=3.,clearance_map={'0.2':.2,'0.8':.8})
        verify_controls(old,new)
        for key,value in (('uniform_fraction',.1),('covariance_floor',1e-3)):
            bad=deepcopy(new);bad['settings'][key]=value
            with self.assertRaisesRegex(ValueError,'beyond cutoff'):verify_controls(old,bad)
        bad=deepcopy(new);bad['replay']['clearance_map']['0.2']=.5
        with self.assertRaisesRegex(ValueError,'clearance changed'):verify_controls(old,bad)
        msg=dict(header=dict(frame_id='world',stamp=1),values=[1.,2.])
        case=dict(case={},start={},goal={},scene={},response={k:deepcopy(msg) for k in
                  ('original_gmm','deformed_gmm','original_trajectory','deformed_trajectory')})
        case['response'].update(policy_sha256='policy',action_scale=.15)
        replay=deepcopy(case);replay['response']['deformed_gmm']['header']['stamp']=2
        verify_environment(case,replay)
        for key in ('case','start','goal','scene'):
            bad=deepcopy(replay);bad[key]['changed']=True
            with self.assertRaisesRegex(ValueError,'frozen'):verify_environment(case,bad)
        for key in ('original_gmm','deformed_gmm','original_trajectory','deformed_trajectory'):
            bad=deepcopy(replay);bad['response'][key]['values'][0]=3.
            with self.assertRaisesRegex(ValueError,'model/reference'):verify_environment(case,bad)

    def test_forbids_fallback_and_wrong_recorded_cutoff(self):
        base=dict(proposal='gmm',corridor_mode='covariance',component_cutoffs=[3.])
        for mode in MODES[:2]:
            verify_purity([dict(mode=mode,sampling=base)],3.)
            changes=[dict(uniform_attempts=1),dict(missing_anchor_fallbacks=1),
                     dict(component_cutoffs=[2.]),dict(component_cutoffs=[]),dict(gmr_attempts=1),
                     {('online_ik_calls' if mode=='joint_projected' else 'projected_attempts'):1}]
            for change in changes:
                with self.assertRaises(ValueError):verify_purity([dict(mode=mode,sampling=base | change)],3.)
        verify_purity([dict(mode='ompl_uniform',sampling={})],3.)
        with self.assertRaises(ValueError):verify_purity([dict(mode='ompl_uniform',sampling=base)],3.)

    def test_environment_weights_do_not_pool_unequal_repeats(self):
        def rows(n,success):return [dict(success=success,failure_stage=None if success else 'planning') for _ in range(n)]
        cases=[dict(stratum='primary',rows={m:(rows(10,False),rows(10,True)) for m in MODES}),
               dict(stratum='second',rows={m:(rows(5,True),rows(5,False)) for m in MODES})]
        for result in summarize(cases,np.random.default_rng(9)):
            self.assertEqual((result['before_successes'],result['after_successes'],result['requests']),(5,10,15))
            self.assertEqual((result['before_rate'],result['after_rate'],result['difference']),(.5,.5,0.))
            self.assertEqual(result['p'],1.)
            self.assertEqual((result['recovered_pairs'],result['lost_pairs']),(10,5))


if __name__=='__main__':unittest.main()
