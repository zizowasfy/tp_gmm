"""The two replay cohorts have different repeat counts; environments stay equal-weighted."""
from pathlib import Path
import hashlib
import json
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from compare_clearance_replays import load, summarize_transition, MODES

class PairedReplayAnalysis(unittest.TestCase):
    def test_rejects_changed_environment_and_trial_provenance(self):
        with tempfile.TemporaryDirectory() as name:
            path=Path(name)
            content={'case':{'goal':[.6,0,.5]}}
            checksum=hashlib.sha256(json.dumps(content,sort_keys=True).encode()).hexdigest()
            trial=dict(environment=0,repeat=0,mode=MODES[0],environment_sha256=checksum)
            manifest=dict(status='complete',schedule=[trial],environments=[dict(id=0,file='case.json',sha256=checksum)])
            (path/'manifest.json').write_text(json.dumps(manifest))
            (path/'case.json').write_text(json.dumps(content))
            (path/'results.jsonl').write_text(json.dumps(trial)+'\n')
            load(path)
            (path/'case.json').write_text('{}')
            with self.assertRaisesRegex(ValueError,'content changed'):load(path)
            (path/'case.json').write_text(json.dumps(content))
            (path/'results.jsonl').write_text(json.dumps(trial | {'environment_sha256':'other'})+'\n')
            with self.assertRaisesRegex(ValueError,'different frozen environment'):load(path)

    def test_environment_weighting_differs_from_attempt_pooling(self):
        def rows(n,success):
            return [dict(success=success,failure_stage=None if success else 'planning',sampling={}) for _ in range(n)]
        # Environment A improves over ten runs; B deteriorates over five runs.
        # Average environment effect is zero, although pooled successes rise 5 -> 10.
        cases=[dict(cluster_stratum='a',rows={m:(rows(10,False),rows(10,True)) for m in MODES}),
               dict(cluster_stratum='b',rows={m:(rows(5,True),rows(5,False)) for m in MODES})]
        result=summarize_transition(cases,.8,.7,np.random.default_rng(9))
        for comparison in result:
            self.assertEqual(comparison['old_successes'],5)
            self.assertEqual(comparison['new_successes'],10)
            self.assertEqual(comparison['plans'],15)
            self.assertEqual(comparison['success_difference'],0.)
            self.assertEqual(comparison['p'],1.)
            self.assertEqual(comparison['recovered_pairs'],10)
            self.assertEqual(comparison['lost_pairs'],5)

if __name__=='__main__':unittest.main()
