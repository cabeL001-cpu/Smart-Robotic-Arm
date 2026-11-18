import math
import unittest

import cv2
import numpy as np

from .board import project, square
from .camera import Mount
from .field import Field, WALLS, locate_floor
from .robot import calibrate, pose_from_plane


class RobotVisionTests(unittest.TestCase):
    def test_vehicle_moves_and_rotates_under_same_camera(self):
        field, mount = Field(), Mount()
        yaw0 = 27
        def homography(x, y, angle):
            a = math.radians(angle)
            r = np.array([[math.cos(a),math.sin(a)], [math.sin(a),-math.cos(a)]])
            h = np.eye(3)
            h[:2,:2] = r * .7
            h[:2,2] = np.array([400.,300.]) - h[:2,:2] @ np.array([x,y])
            return h
        for i, (x, y, yaw) in enumerate([(1000,1000,yaw0), (750,1200,-64)]):
            h = homography(x, y, yaw)
            tags = {k: project(h, v) for k,v in field.floor_layout().items()}
            plane, source = locate_floor(tags, mount, field)
            if i == 0:
                calibrate(mount, plane, (800,600), x/1000, y/1000, yaw)
            pose = pose_from_plane(plane, source, mount)
            np.testing.assert_allclose([pose.x_m,pose.y_m,pose.yaw_rad],
                                       [x/1000,y/1000,math.radians(yaw)], atol=1e-5)
        self.assertIsNone(locate_floor({}, mount, field)[0])

    def test_upright_wall_with_real_height_and_known_intrinsics(self):
        K = np.array([[500.,0,400],[0,500,300],[0,0,1]])
        mount = Mount(camera_matrix=K.tolist(), height_mm=1500)
        field = Field(wall_heights_mm={"1": 250})
        camera = np.array([1050,1500,1500])
        center = np.array([*WALLS[1][0],250])
        # 相机 +Z 向下并略向北看墙标，+X 向东。
        c,s=math.cos(.20),math.sin(.20)
        R=np.array([[1.,0,0],[0,-c,-s],[0,s,-c]])
        rvec = cv2.Rodrigues(R)[0]
        corners = square(0,0,200)
        world = center + np.column_stack([corners[:,0],np.zeros(4),corners[:,1]])
        pixels = cv2.projectPoints(world.astype(float), rvec, -R@camera, K, None)[0].reshape(-1,2)
        plane, source = locate_floor({("36h11",1): pixels}, mount, field)
        self.assertEqual(source, "wall")
        gp = cv2.projectPoints(np.array([[1050.,1500.,0.],[1150.,1500.,0.]]), rvec, -R@camera, K, None)[0].reshape(-1,2)
        mount.robot_center_px,mount.robot_forward_px = gp.tolist()
        pose = pose_from_plane(plane, source, mount)
        np.testing.assert_allclose([pose.x_m,pose.y_m,pose.yaw_rad],[1.05,1.5,0],atol=1e-5)

    def test_unseen_or_uncalibrated_wall_is_not_a_floor_marker(self):
        mount = Mount()
        self.assertEqual(locate_floor({("36h11",4):square(100,100,40)},mount,Field()),(None,None))
        field=Field(floor_tags=[{"family":"36h11","id":20,"center_mm":[50,50],"side_mm":60}])
        self.assertEqual(locate_floor({("36h11",20):square(200,200,60)},mount,field)[1],"floor")
