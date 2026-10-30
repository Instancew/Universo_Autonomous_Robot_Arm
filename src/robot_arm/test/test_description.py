"""The robot description must load: one tree, the expected frames, and every mesh file present and valid."""
import struct
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

PKG = Path(__file__).resolve().parent.parent
URDF = PKG / 'description' / 'urdf' / 'robot_arm.urdf'
MESHES = PKG / 'description' / 'meshes'
PREFIX = 'package://robot_arm/description/meshes/'
ROOT = ET.parse(URDF).getroot()


def test_single_tree_with_expected_frames_and_joints():
    links = [l.get('name') for l in ROOT.findall('link')]
    children = [j.find('child').get('link') for j in ROOT.findall('joint')]
    assert len(children) == len(set(children)), 'a link has two parents'
    assert set(links) - set(children) == {'base_link'}
    assert {'base_link', 'arm_tip'} <= set(links)
    movable = {j.get('name'): j.get('type') for j in ROOT.findall('joint') if j.get('type') != 'fixed'}
    assert movable == {'base_joint': 'continuous', 'shoulder_joint': 'revolute', 'elbow_joint': 'revolute'}


def test_every_mesh_file_exists_and_is_a_valid_stl():
    names = {m.get('filename') for m in ROOT.iter('mesh')}
    assert names and all(n.startswith(PREFIX) for n in names), 'mesh paths must use ' + PREFIX
    missing = sorted(n[len(PREFIX):] for n in names if not (MESHES / n[len(PREFIX):]).is_file())
    assert not missing, f'{len(missing)} mesh files missing from description/meshes: {missing[:5]}'
    for n in names:
        data = (MESHES / n[len(PREFIX):]).read_bytes()
        if data[:5] == b'solid' and b'facet' in data[:1000]:
            continue                                            # ASCII STL
        count = struct.unpack('<I', data[80:84])[0]
        assert len(data) == 84 + 50 * count, f'{n} is not a valid binary STL'


@pytest.mark.parametrize('stl', sorted(p.name for p in MESHES.glob('*.stl')))
def test_no_unused_mesh_files(stl):
    assert PREFIX + stl in {m.get('filename') for m in ROOT.iter('mesh')}, f'{stl} is not used by robot_arm.urdf'
