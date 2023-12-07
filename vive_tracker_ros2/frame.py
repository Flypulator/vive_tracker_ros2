import numpy as np
from scipy.spatial.transform import Rotation


class Frame:
    def __init__(self, frame_name: str, reference_frame, pos_offset: np.ndarray, rotation: Rotation):
        self.frame_name: str = frame_name
        self.reference_frame = reference_frame
        self.pos_offset: np.ndarray = pos_offset
        self.rotation: Rotation = rotation

    @property
    def world_pos(self):
        temp_world_pos = self.pos_offset
        current_ref_frame = self.reference_frame
        while current_ref_frame != WorldFrame():
            temp_world_pos = current_ref_frame.pos_offset + current_ref_frame.rotation.apply(temp_world_pos)
            current_ref_frame = current_ref_frame.reference_frame
        return temp_world_pos

    @property
    def world_ori(self):
        temp_world_ori = self.rotation
        current_ref_frame = self.reference_frame
        while current_ref_frame != WorldFrame():
            temp_world_ori = current_ref_frame.rotation * temp_world_ori
            current_ref_frame = current_ref_frame.reference_frame
        return temp_world_ori

    def pos_in_frame_coordinates(self, pos: np.ndarray, pos_reference_frame) -> np.ndarray:
        pos_in_world = pos_reference_frame.world_pos + pos_reference_frame.world_ori.apply(pos)
        pos_in_frame = self.world_ori.apply(pos_in_world - self.world_pos, inverse=True)
        return pos_in_frame

    def ori_in_frame_coordinates(self, ori: Rotation, ori_reference_frame) -> Rotation:
        ori_in_world = ori_reference_frame.world_ori * ori
        ori_in_frame = self.world_ori.inv() * ori_in_world
        return ori_in_frame

    def relative_frame(self, other_frame):
        relative_frame = Frame(
            frame_name=other_frame.frame_name,
            reference_frame=self,
            pos_offset=self.pos_in_frame_coordinates(other_frame.pos_offset, other_frame.reference_frame),
            rotation=self.ori_in_frame_coordinates(other_frame.rotation, other_frame.reference_frame)
        )
        return relative_frame

    def __eq__(self, other_frame):
        return self.frame_name == other_frame.frame_name


class WorldFrame(Frame):
    def __init__(self):
        super().__init__(frame_name="world", reference_frame=None, pos_offset=np.zeros(3),
                         rotation=Rotation.from_quat([0, 0, 0, 1]))

    @property
    def world_pos(self):
        return np.zeros(3)

    @property
    def world_ori(self):
        return Rotation.from_quat([0, 0, 0, 1])
