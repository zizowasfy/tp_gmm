"""ROS message-level checks: baseline bypasses GMM constraints and never executes."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import RobotState
from run_clearance_sweep import ClearanceRunner


class BaselineContractTest(unittest.TestCase):
    def test_default_ompl_request_has_no_gmm_path_constraints(self):
        captured=[]
        client=SimpleNamespace(send_goal_async=lambda req:captured.append(req))
        fake=SimpleNamespace(group_name='panda_arm',move_action_client=client)
        state=RobotState()
        state.joint_state.name=['panda_joint1']
        state.joint_state.position=[0.]
        result=MoveGroup.Result()
        result.error_code.val=result.error_code.SUCCESS
        handle=SimpleNamespace(accepted=True,get_result_async=lambda:None)
        with patch('run_clearance_sweep.wait',side_effect=[handle,SimpleNamespace(result=result)]):
            ClearanceRunner.uniform_plan(fake,state,state,2.)
        request=captured[0]
        self.assertTrue(request.planning_options.plan_only)
        self.assertFalse(request.planning_options.replan)
        self.assertEqual(request.request.path_constraints.name,'')
        self.assertEqual(len(request.request.path_constraints.position_constraints),0)
        self.assertEqual(len(request.request.path_constraints.orientation_constraints),0)
        self.assertEqual(request.request.pipeline_id,'ompl')
        self.assertEqual(len(request.request.goal_constraints),1)


if __name__=='__main__':
    unittest.main()
