"""Verify the compiled pattern API using real TinyXML fixtures."""
import pathlib
import sys

sys.path.insert(0, sys.argv[1])
import cadexchange_py as cad


def require(condition, message):
    if not condition:
        raise AssertionError(message)


require(hasattr(cad, 'PatternAffectedBodyMode'), 'ordinary pattern scope enum is exposed')
fixtures = pathlib.Path(sys.argv[2])
model = cad.load_model(str(fixtures / 'bodies-tiny.xml'))
linear = model.get_feature(0).as_linear_pattern()
circular = model.get_feature(1).as_circular_pattern()
require(linear.scope == cad.PatternScope.BODIES, 'linear Body scope')
require(circular.scope == cad.PatternScope.BODIES, 'circular Body scope')
require(linear.dir1_count == 4 and linear.dir1_spacing == 12, 'linear primary parameters')
require(circular.count == 6 and circular.skipped_instance_count == 1, 'circular primary and skips')
require(circular.get_skipped_instance(0) == {'dir1_index': 4, 'dir2_index': 0}, 'canonical skip index')
require(linear.get_seed_object(0).as_body().volume == 6000, 'Body reference retained')
require(linear.affected_body_mode is None and linear.affected_bodies == [], 'inactive Body scope options')
seed_only = cad.load_model(str(fixtures / 'seed-only-tiny.xml')).get_feature(0).as_linear_pattern()
require(seed_only.pattern_seed_only and seed_only.has_dir2, 'seed-only with D2')
require(seed_only.dir2_count == 3 and seed_only.dir2_spacing == 20, 'D2 read-only parameters')
require(seed_only.dir2_direction == [0, 1, 0] and seed_only.dir2_reference.is_valid(), 'D2 direction and reference')
geometry = cad.load_model(str(fixtures / 'selected-tiny.xml')).get_feature(2).as_circular_pattern()
require(geometry.affected_body_mode == cad.PatternAffectedBodyMode.SelectedBodies, 'selected affected mode')
require(len(geometry.affected_bodies) == 1 and geometry.affected_bodies[0].volume == 6000, 'affected Body geometry')
bodies = geometry.affected_bodies
bodies.clear()
require(len(geometry.affected_bodies) == 1, 'container mutation cannot alter shared model')
for target, name, value in [(linear, 'dir1_spacing', 999), (linear, 'affected_body_mode', None),
                             (seed_only, 'dir2_count', 999), (geometry, 'affected_bodies', [])]:
    try:
        setattr(target, name, value)
    except AttributeError:
        pass
    else:
        raise AssertionError(f'{name} must be read-only')
try:
    cad.load_model(str(fixtures / 'bad-missing-PatternSeedOnly.xml'))
except RuntimeError:
    pass
else:
    raise AssertionError('malformed pattern must fail Python load')
print('PASS compiled ordinary pattern read-only Python API')
