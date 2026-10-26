#!/usr/bin/env python3
"""Run a Mission test in its own coordinated ROS domain.

ament_cmake_ros' run_test_isolated.py keeps an inherited ROS_DOMAIN_ID, and
the devcontainer exports ROS_DOMAIN_ID=0, so `colcon test` there ran every
Mission test in the same domain (and in the same domain as Core's). Tests that
serve or call the same production names then answered each other's requests
whenever ctest ran them in parallel.

This runner always picks a coordinated domain (unless DISABLE_ROS_ISOLATION is
set for debugging), skips the inherited domain and the workspace's live stack
domains so a test can never join a running SIM/HIL graph, and limits discovery
to this host.
"""

import contextlib
import os
import sys

import ament_cmake_test
import domain_coordinator

# Defaults of III_HIL_ROS_DOMAIN_ID and III_DATASET_ROS_DOMAIN_ID.
LIVE_STACK_DOMAINS = {42, 74}


def reserved_domains():
    reserved = set(LIVE_STACK_DOMAINS)
    for name in ('ROS_DOMAIN_ID', 'III_HIL_ROS_DOMAIN_ID', 'III_DATASET_ROS_DOMAIN_ID'):
        value = os.environ.get(name, '').strip()
        if value.isdigit():
            reserved.add(int(value))
    return reserved


class FreeDomainSelector:

    def __init__(self, reserved):
        self._candidates = [domain for domain in range(1, 101) if domain not in reserved]
        self._next = 0

    def __call__(self):
        domain = self._candidates[self._next % len(self._candidates)]
        self._next += 1
        return domain


if __name__ == '__main__':
    with contextlib.ExitStack() as stack:
        if 'DISABLE_ROS_ISOLATION' not in os.environ:
            domain_id = stack.enter_context(
                domain_coordinator.domain_id(FreeDomainSelector(reserved_domains()))
            )
            print('Running with ROS_DOMAIN_ID {}'.format(domain_id))
            os.environ['ROS_DOMAIN_ID'] = str(domain_id)
            os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        sys.exit(ament_cmake_test.main())
