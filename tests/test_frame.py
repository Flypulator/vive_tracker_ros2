import numpy as np
from scipy.spatial.transform import Rotation

from src.frame import Frame, WorldFrame


class TestFrame:
    def test_init(self):
        frame_name = "my_frame"
        reference_frame = WorldFrame()
        pos_offset = np.array([1, 2, 3])
        rotation = Rotation.from_euler("X", 90, degrees=True)

        frame = Frame(frame_name, reference_frame, pos_offset, rotation)

        assert frame.frame_name == frame_name
        assert frame.reference_frame == reference_frame
        np.testing.assert_equal(frame.pos_offset, pos_offset)
        assert frame.rotation == rotation
        np.testing.assert_equal(frame.world_pos, pos_offset)
        assert frame.world_ori == rotation

    def test_double_ref(self):
        f1_name = "f1"
        f1_reference_frame = WorldFrame()
        f1_pos_offset = np.array([1, 0, 0])
        f1_rotation = Rotation.from_euler("Z", 90, degrees=True)
        f1 = Frame(f1_name, f1_reference_frame, f1_pos_offset, f1_rotation)

        f2_name = "f2"
        f2_reference_frame = f1
        f2_pos_offset = np.array([1, 0, 0])
        f2_rotation = Rotation.from_euler("Y", 90, degrees=True)
        f2 = Frame(f2_name, f2_reference_frame, f2_pos_offset, f2_rotation)

        exp_f2_world_pos = np.array([1, 1, 0])
        exp_f2_world_ori = Rotation.from_euler("ZY", [90, 90], degrees=True)

        np.testing.assert_allclose(f2.world_pos, exp_f2_world_pos)
        np.testing.assert_allclose(f2.world_ori.as_quat(), exp_f2_world_ori.as_quat())

    def test_pos_in_world_frame(self):
        pos = np.array([1, 2, 3])
        pos_ref_frame = WorldFrame()

        pos_in_world = WorldFrame().pos_in_frame_coordinates(pos, pos_ref_frame)

        np.testing.assert_allclose(pos_in_world, pos)

    def test_world_pos_in_f1(self):
        pos = np.array([1, 2, 3])
        pos_ref_frame = WorldFrame()

        f1_name = "f1"
        f1_reference_frame = WorldFrame()
        f1_pos_offset = np.array([1, 0, 0])
        f1_rotation = Rotation.from_euler("Z", 90, degrees=True)
        f1 = Frame(f1_name, f1_reference_frame, f1_pos_offset, f1_rotation)

        pos_in_f1 = f1.pos_in_frame_coordinates(pos, pos_ref_frame)

        exp_vec_in_f1 = [2, 0, 3]

        np.testing.assert_allclose(pos_in_f1, exp_vec_in_f1, atol=1e-15)

    def test_f1_pos_in_f2(self):
        f1_name = "f1"
        f1_reference_frame = WorldFrame()
        f1_pos_offset = np.array([1, 0, 0])
        f1_rotation = Rotation.from_euler("Z", 90, degrees=True)
        f1 = Frame(f1_name, f1_reference_frame, f1_pos_offset, f1_rotation)

        pos = np.array([1, 2, 3])
        pos_ref_frame = f1

        f2_name = "f2"
        f2_reference_frame = WorldFrame()
        f2_pos_offset = np.array([0, 1, 2])
        f2_rotation = Rotation.from_euler("Y", 90, degrees=True)
        f2 = Frame(f2_name, f2_reference_frame, f2_pos_offset, f2_rotation)

        pos_in_f2 = f2.pos_in_frame_coordinates(pos, pos_ref_frame)

        exp_vec_in_f1 = [-1, 0, -1]

        np.testing.assert_allclose(pos_in_f2, exp_vec_in_f1, atol=1e-15)

    def test_ori_in_world_frame(self):
        ori = Rotation.from_euler("Z", 90, degrees=True)
        ori_ref_frame = WorldFrame()

        ori_in_world = WorldFrame().ori_in_frame_coordinates(ori, ori_ref_frame)

        np.testing.assert_allclose((ori_in_world * ori.inv()).as_euler("XYZ"), np.zeros(3))

    def test_world_ori_in_f1(self):
        ori = Rotation.from_euler("Z", 90, degrees=True)
        ori_ref_frame = WorldFrame()

        f1_name = "f1"
        f1_reference_frame = WorldFrame()
        f1_pos_offset = np.array([0, 0, 0])
        f1_rotation = Rotation.from_euler("X", 90, degrees=True)
        f1 = Frame(f1_name, f1_reference_frame, f1_pos_offset, f1_rotation)

        ori_in_f1 = f1.ori_in_frame_coordinates(ori, ori_ref_frame)

        exp_ori_in_f1 = Rotation.from_euler("XYZ", [-90, 0, 90], degrees=True)

        np.testing.assert_allclose((ori_in_f1 * exp_ori_in_f1.inv()).as_euler("XYZ"), np.zeros(3))

    def test_f1_ori_in_f2(self):
        f1_name = "f1"
        f1_reference_frame = WorldFrame()
        f1_pos_offset = np.array([0, 0, 0])
        f1_rotation = Rotation.from_euler("Z", 90, degrees=True)
        f1 = Frame(f1_name, f1_reference_frame, f1_pos_offset, f1_rotation)

        ori = Rotation.from_euler("X", 90, degrees=True)
        ori_ref_frame = f1

        f2_name = "f2"
        f2_reference_frame = WorldFrame()
        f2_pos_offset = np.array([0, 0, 0])
        f2_rotation = Rotation.from_euler("Y", 90, degrees=True)
        f2 = Frame(f2_name, f2_reference_frame, f2_pos_offset, f2_rotation)

        ori_in_f2 = f2.ori_in_frame_coordinates(ori, ori_ref_frame)

        exp_ori_in_f2 = Rotation.from_euler("XYZ", [0, 0, 90], degrees=True)

        np.testing.assert_allclose((ori_in_f2 * exp_ori_in_f2.inv()).as_euler("XYZ"), np.zeros(3))

    def test_relative_frame(self):
        f1_name = "f1"
        f1_reference_frame = WorldFrame()
        f1_pos_offset = np.array([1, 2, 3])
        f1_rotation = Rotation.from_euler("Z", 90, degrees=True)
        f1 = Frame(f1_name, f1_reference_frame, f1_pos_offset, f1_rotation)

        f2_name = "f2"
        f2_reference_frame = WorldFrame()
        f2_pos_offset = np.array([1, 0, 0])
        f2_rotation = Rotation.from_euler("Y", 90, degrees=True)
        f2 = Frame(f2_name, f2_reference_frame, f2_pos_offset, f2_rotation)

        exp_pos_offset = np.array([-2, 0, -3])
        exp_rotation = Rotation.from_euler("XYZ", [90, 0, -90], degrees=True)

        relative_frame = f1.relative_frame(f2)
        assert relative_frame.frame_name == f2.frame_name
        assert relative_frame.reference_frame == f1
        np.testing.assert_allclose(relative_frame.pos_offset, exp_pos_offset, atol=1e-15)
        np.testing.assert_allclose((relative_frame.rotation * exp_rotation.inv()).as_euler("XYZ"), np.zeros(3))
        np.testing.assert_allclose(relative_frame.world_pos, f2.world_pos)
        np.testing.assert_allclose((relative_frame.world_ori * f2.world_ori.inv()).as_euler("XYZ"), np.zeros(3))


