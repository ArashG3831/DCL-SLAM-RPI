from pathlib import Path
import hashlib


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'my_epuck_project'
REFERENCE = (Path('/home/robot1/cooperative_migration_source') / 'src' /
             'my_epuck_project' / 'my_epuck_project')


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_estimator_closure_matches_verified_source():
    expected = {
        'unknown_pose_frontend.py':
            'db13ac0b07092682bf5748ec63e47d7edf556ddc5f0395f22363b92f9e03557d',
        'unknown_pose_frontend_core.py':
            '5743950b8c19d47da115fd13d7dd8f556adc0790cea565473655d395c454ffde',
        'robust_relative_pose_selector.py':
            '34fcc9039f7802161a33a236e71f4efc72fd74705dead37adac8ed72e497fae2',
    }
    for name, digest in expected.items():
        destination = PACKAGE / name
        source = REFERENCE / name
        assert destination.is_file()
        assert destination.read_bytes() == source.read_bytes()
        assert _sha256(destination) == digest


def test_frontend_entry_point_and_dependencies_are_declared_once():
    setup = (ROOT / 'setup.py').read_text(encoding='utf-8')
    package = (ROOT / 'package.xml').read_text(encoding='utf-8')
    entry = 'unknown_pose_frontend = '
    assert setup.count(entry) == 1
    assert 'my_epuck_project.unknown_pose_frontend:main' in setup
    assert package.count('<exec_depend>python3-opencv</exec_depend>') == 1
    assert package.count('<exec_depend>python3-scipy</exec_depend>') == 1


def test_frontend_uses_verified_unknown_pose_contract():
    source = (PACKAGE / 'unknown_pose_frontend.py').read_text(encoding='utf-8')
    for topic in (
            '/cslam/relative_pose/descriptors',
            '/cslam/relative_pose/crop_requests',
            '/cslam/relative_pose/crops',
            '/cslam/relative_pose/hypotheses'):
        assert topic in source
    for message in (
            'FullMapSnapshotRequest', 'FullMapSnapshotResponse',
            'LocalMapCrop', 'LocalMapCropRequest', 'LocalMapDescriptor',
            'PeerMap', 'RelativePoseHypothesis'):
        assert message in source
    assert 'StaticTransformBroadcaster' in source
    assert 'self.publish_accepted_tf()' in source
    assert 'message.accepted' in source
    assert "status='ACCEPTED'" in source


def test_pi_planned_path_interface_remains_untouched():
    message = (Path('/home/robot1/webots_ws') / 'src' /
               'my_epuck_interfaces' / 'msg' / 'PhysicalTask.msg')
    text = message.read_text(encoding='utf-8')
    assert 'nav_msgs/Path planned_path' in text
