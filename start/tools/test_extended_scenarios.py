#!/usr/bin/env python3
"""Exercise scenario checks without a ROS master or a simulated aircraft."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
import extended_flight_probe as probe


class ScenarioChecks(unittest.TestCase):
    def setUp(self):
        clock=patch.object(probe.rospy.Time,'now',return_value=probe.rospy.Time(10))
        clock.start()
        self.addCleanup(clock.stop)

    def client(self):
        p=probe.Probe.__new__(probe.Probe)
        p.data={'phase':SimpleNamespace(data='HOLD'), 'state':SimpleNamespace(mode='OFFBOARD')}
        p.error_sequence=0
        p.position=Mock(return_value=np.array([0.,0.,1.]))
        p.wait=Mock()
        return p

    def test_delayed_navigation_state_is_awaited(self):
        p=self.client();p.goal=Mock(return_value={'accepted':True})
        p.position=Mock(return_value=np.array([.95,0.,1.]))
        p.wait=probe.Probe.wait.__get__(p)
        def advance(_):
            if _==.05:p.data['phase'].data='NAVIGATING'
            elif _==.1:p.data['phase'].data='HOLD'
        with patch.object(probe.time,'sleep',side_effect=advance):
            self.assertTrue(p.navigate(np.array([1.,0.,1.]))['passed'])

    def test_protected_hold_near_goal_is_not_arrival(self):
        p=self.client()
        def accept(_):
            p.receive('error',SimpleNamespace(data='Trajectory segment intersects an inflated obstacle voxel'))
            return {'accepted':True}
        p.goal=Mock(side_effect=accept)
        with patch.object(probe.time,'sleep'):
            with self.assertRaisesRegex(RuntimeError,'protected before arrival'):
                p.navigate(np.array([.1,0.,1.]))

    def test_hold_in_wrong_mode_cannot_certify_arrival(self):
        p=self.client();p.goal=Mock(return_value={'accepted':True});p.data['state'].mode='AUTO.LAND'
        with patch.object(probe.time,'sleep'):
            with self.assertRaisesRegex(RuntimeError,'Unexpected flight state at arrival'):
                p.navigate(np.array([.1,0.,1.]))

    def test_missing_navigation_transition_cannot_pass(self):
        p=self.client();p.goal=Mock(return_value={'accepted':True})
        p.wait=probe.Probe.wait.__get__(p)
        with patch.object(probe.time,'monotonic',side_effect=[0,4]):
            with self.assertRaisesRegex(RuntimeError,'Timed out'):
                p.navigate(np.array([1.,0.,1.]))

    def test_goal_keeps_enu_values(self):
        p=self.client()
        service=Mock(return_value=SimpleNamespace(success=True,message='accepted'))
        with patch.object(probe.rospy,'ServiceProxy',return_value=service), \
             patch.object(probe.rospy.Time,'now',return_value=probe.rospy.Time(10)):
            result=p.goal([1.,-2.,.9])
        goal=service.call_args.args[0]
        self.assertEqual(goal.header.frame_id,'odom')
        self.assertEqual([goal.pose.position.x,goal.pose.position.y,goal.pose.position.z],[1.,-2.,.9])
        self.assertTrue(result['accepted'])

    def test_native_offboard_decode_does_not_require_auto_base_bit(self):
        self.assertEqual(probe.px4_mode(6<<16),'OFFBOARD')
        self.assertEqual(probe.px4_mode((4<<16)|(6<<24)),'LAND')
        self.assertEqual(probe.px4_mode(3<<16),'POSCTL')

    def test_rejected_navigation_cannot_pass(self):
        p=self.client();p.goal=Mock(return_value={'accepted':False})
        with self.assertRaisesRegex(RuntimeError,'Goal rejected'):
            p.navigate(np.array([1.,0.,1.]))

    def test_hold_away_from_goal_fails(self):
        p=self.client();p.goal=Mock(return_value={'accepted':True})
        with self.assertRaisesRegex(RuntimeError,'cancelled away'):
            p.navigate(np.array([1.,0.,1.]))

    def test_arrival_requires_hold_and_point_two_error(self):
        p=self.client();p.goal=Mock(return_value={'accepted':True})
        result=p.navigate(np.array([.1,0.,1.]))
        self.assertTrue(result['passed'])
        self.assertAlmostEqual(result['goal_error_m'],.1)

    def test_landing_during_navigation_fails(self):
        p=self.client();p.goal=Mock(return_value={'accepted':True})
        p.data['phase'].data='LANDING'
        with self.assertRaisesRegex(RuntimeError,'Unexpected flight state'):
            p.navigate(np.array([1.,0.,1.]))

    def test_hover_rejects_offboard_exit(self):
        p=self.client();p.data['state'].mode='AUTO.LAND'
        with self.assertRaisesRegex(RuntimeError,'lost HOLD/OFFBOARD'):
            p.hover(20)

    def test_hover_applies_original_limits(self):
        p=self.client()
        p.position.side_effect=[np.array([0.,0.,1.]),np.array([0.,0.,1.]),
                                np.array([0.,0.,1.]),np.array([0.,0.,1.]),
                                np.array([0.,0.,1.16]),np.array([0.,0.,1.16])]
        with patch.object(probe.time,'monotonic',side_effect=[0,.1,.2,1]), \
             patch.object(probe.time,'sleep'):
            with self.assertRaisesRegex(RuntimeError,'Hover out of bounds'):
                p.hover(.5)


if __name__=='__main__':
    unittest.main(verbosity=2)
