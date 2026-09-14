import unittest
from unittest.mock import Mock
from types import SimpleNamespace
from tp_gmm_sampling import SamplingClient, MODES


class RuntimeClientTest(unittest.TestCase):
    def test_session_releases_on_exception(self):
        client = SamplingClient.__new__(SamplingClient)
        client.prepare = Mock(return_value=SimpleNamespace(request_id='test-request'))
        client.release = Mock()
        with self.assertRaisesRegex(RuntimeError, 'planning failed'):
            with client.session('model', mode='cartesian_ik') as prepared:
                self.assertEqual(prepared.request_id, 'test-request')
                raise RuntimeError('planning failed')
        client.release.assert_called_once_with('test-request')

    def test_only_runtime_methods_are_advertised(self):
        self.assertEqual(MODES, ('cartesian_ik', 'joint_projected'))


if __name__ == '__main__':
    unittest.main()
