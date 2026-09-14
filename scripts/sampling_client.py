#!/usr/bin/env python3
"""Compatibility import for existing workspace scripts; new consumers use tp_gmm_sampling."""
from tp_gmm_sampling import SamplingClient, MODES, DEFAULTS

__all__ = ['SamplingClient', 'MODES', 'DEFAULTS']
