#!/usr/bin/env python3
"""Shared request preparation/report protocol for interactive and paired experiments."""
import json
from contextlib import contextmanager
from copy import deepcopy
import rclpy
from tp_gmm.srv import PrepareSampling, SamplingReport

MODES = ('cartesian_ik', 'joint_projected', 'uniform')
PROPOSALS = ('gmm', 'gmr_path', 'hybrid')
DEFAULTS = dict(corridor_mode='covariance', corridor_scale=10.0, cutoff=2.0, covariance_floor=1e-8, uniform_fraction=0.1,
                cartesian_fraction=0.1, ik_timeout=0.005, branches=3,
                anchor_attempts=16, nullspace_stddev=0.08, max_joint_delta=0.6,
                linearization_tolerance=0.01, proposal='gmm', gmr_stddev=0.01, gmr_cutoff=2.0, gmr_fraction=0.8)

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

    def prepare(self, model, group, link, orientation, mode, seed=1, visualize=False, reference_path=None, visualize_paths=None, **options):
        if mode not in MODES:
            raise ValueError(f'Unknown sampler {mode}')
        req = PrepareSampling.Request()
        req.model = deepcopy(model)
        req.group_name, req.link_name, req.mode = group, link, mode
        req.orientation = deepcopy(orientation)
        req.seed, req.visualize = int(seed), bool(visualize)
        req.visualize_path = bool(visualize if visualize_paths is None else visualize_paths)
        if reference_path is not None:
            req.reference_path = deepcopy(reference_path)
        settings = DEFAULTS | options
        if settings.keys() != DEFAULTS.keys():
            raise ValueError(f'Unknown options: {settings.keys() - DEFAULTS.keys()}')
        if settings['proposal'] not in PROPOSALS:
            raise ValueError(f"Unknown proposal {settings['proposal']}")
        if settings['proposal'] != 'gmm' and (mode != 'cartesian_ik' or not req.reference_path.poses):
            raise ValueError('GMR proposals require cartesian_ik and a request-matched reference_path')
        for key, value in settings.items():
            setattr(req, key, value)
        return self.call(self.prepare_client, req)

    def report(self, request_id, result=None, release=True):
        req = SamplingReport.Request(request_id=request_id, release=release)
        if result is not None:
            req.trajectory = result.planned_trajectory
            req.trajectory_start = result.trajectory_start
        response = self.call(self.report_client, req, timeout=60.0)
        return json.loads(response.json)

    def release(self, request_id):
        return self.report(request_id, release=True)

    @contextmanager
    def session(self, *args, **kwargs):
        prepared = self.prepare(*args, **kwargs)
        try:
            yield prepared
        finally:
            self.release(prepared.request_id)
