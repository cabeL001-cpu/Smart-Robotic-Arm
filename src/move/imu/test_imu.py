import io
import math
import struct
import unittest

from . import G, Sample, Tracker, attitude
from .sensor import Decoder, HEADER, euler_quaternion, read_csv, record_samples


def packet(kind, payload):
    data = HEADER + bytes([len(payload)+5, kind]) + payload
    return data + bytes([sum(data) & 255])


def sample(t, ax=0, q=(1,0,0,0)):
    # 导航 +Z 重力及沿传感器 +X 的运动加速度。
    grav = attitude(q)[2]
    return Sample(t, (grav[0]+ax/G, grav[1], grav[2]), (0,0,0), q)


class ImuTests(unittest.TestCase):
    def test_vendor_stream_split_noise_bad_checksum_and_two_frame_types(self):
        raw = packet(4, struct.pack("<9h",1024,0,2048,0,0,512,0,0,0))
        quat = packet(0x16, struct.pack("<4f",1,0,0,0))
        euler = packet(0x26, struct.pack("<3f",0,0,math.pi/2))
        decoder = Decoder()
        corrupt = raw[:-1] + bytes([raw[-1]^1])
        self.assertEqual(list(decoder.feed(b"noise"+corrupt+raw[:7], 1.)), [])
        out = list(decoder.feed(raw[7:]+quat+raw+euler, 1.02))
        self.assertEqual(len(out), 2)
        self.assertAlmostEqual(out[0].accel_g[0], 1024*16/32767)
        self.assertAlmostEqual(out[0].gyro_rad_s[2], math.radians(512*2000/32767))
        self.assertAlmostEqual(out[1].quat[-1], math.sqrt(.5), places=6)
        self.assertLess(out[0].t, out[1].t)

    def test_tilt_motion_zero_velocity_and_gap(self):
        q=euler_quaternion(.3,.1,.2)
        tr=Tracker([sample(0,q=q),sample(.1,q=q)])
        for i in range(1,11):
            st=tr.update(sample(.1+i*.02,q=q))
        self.assertAlmostEqual(st.x_m,0,places=6)
        tr=Tracker([sample(0),sample(.1)],yaw_rad=math.pi/2)
        for i in range(1,101):
            st=tr.update(sample(.1+i*.01,1.))
        self.assertAlmostEqual(st.x_m,0,places=6)
        self.assertAlmostEqual(st.y_m,.495,delta=.001)
        self.assertGreater(tr.update(sample(1.11)).vy_mps,.99)
        self.assertEqual(tr.update(sample(1.12),stationary=True).vy_mps,0)
        y=tr.state.y_m
        self.assertEqual(tr.update(sample(2.)).y_m,y)
        self.assertFalse(tr.correct(1.12,9,9,0))

    def test_delayed_fix_preserves_subsequent_motion_and_rejects_stale(self):
        tr=Tracker([sample(0),sample(.1)])
        for i in range(1,21):
            tr.update(sample(.1+i*.02,1.))
        old=tr.state
        anchor=next(s for s in tr.history if abs(s.t-.3)<1e-6)
        self.assertTrue(tr.correct(anchor.t,1,2,math.pi/2))
        self.assertAlmostEqual(tr.state.x_m,1)
        self.assertAlmostEqual(tr.state.y_m,2+old.x_m-anchor.x_m)
        self.assertAlmostEqual(tr.state.vy_mps,old.vx_mps)
        self.assertFalse(tr.correct(anchor.t,9,9,0))

    def test_record_and_replay_raw_samples(self):
        src=[sample(0),sample(.1),sample(.2,.8)]
        out=io.StringIO()
        self.assertEqual(list(record_samples(src,out)),src)
        self.assertEqual(list(read_csv(io.StringIO(out.getvalue()))),src)
        with self.assertRaises(ValueError):
            list(read_csv(io.StringIO("time_s,x_m,y_m\n0,0,0\n")))
