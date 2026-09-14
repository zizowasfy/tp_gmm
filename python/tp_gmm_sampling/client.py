#!/usr/bin/env python3
"""Runtime client for immutable GMM sampling requests and lightweight diagnostics."""
import json
from copy import deepcopy
from contextlib import contextmanager
import rclpy
from tp_gmm.srv import PrepareSampling, SamplingReport

MODES = ('cartesian_ik', 'joint_projected')
DEFAULTS = dict(cutoff=2.0, covariance_floor=1e-8, uniform_fraction=0.1,
                cartesian_fraction=0.1, ik_timeout=0.005, branches=3,
                anchor_attempts=16, nullspace_stddev=0.08, max_joint_delta=0.6,
                linearization_tolerance=0.01)

class SamplingClient:
    def __init__(self, node, namespace=''):
        self.node = node
        self.prepare_client = node.create_client(PrepareSampling, f'{namespace}/gmm_sampling/prepare')
        self.report_client = node.create_client(SamplingReport, f'{namespace}/gmm_sampling/report')

    def call(self, client, request, timeout=20.0):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'Sampling service unavailable: {client.srv_name}; restart MoveIt with the plugin')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            future.cancel()
            raise TimeoutError(f'Service timed out: {client.srv_name}')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(response.message if response else 'Empty service response')
        return response

    def prepare(self, model, group, link, orientation, mode, seed=1, visualize=False, **options):
        if mode not in MODES:
            raise ValueError(f'Unknown sampler {mode}')
        req = PrepareSampling.Request()
        req.model = deepcopy(model)
        req.group_name, req.link_name, req.mode = group, link, mode
        req.orientation = deepcopy(orientation)
        req.seed, req.visualize = int(seed), bool(visualize)
        settings = DEFAULTS | options
        if settings.keys() != DEFAULTS.keys():
            raise ValueError(f'Unknown options: {settings.keys() - DEFAULTS.keys()}')
        for key, value in settings.items():
            setattr(req, key, value)
        return self.call(self.prepare_client, req)

    def report(self, request_id, release=True):
        req = SamplingReport.Request(request_id=request_id, release=release)
        response = self.call(self.report_client, req, timeout=60.0)
        return json.loads(response.json)

    def release(self, request_id):
        """Call after the planning action finishes/cancels; active samplers own their snapshot."""
        return self.report(request_id, release=True)

    @contextmanager
    def session(self, *args, **kwargs):
        """Own one request until the caller finishes or cancels its planning action."""
        prepared = self.prepare(*args, **kwargs)
        try:
            yield prepared
        finally:
            self.release(prepared.request_id)
