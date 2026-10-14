"""Exercise the compiled read-only Python Body/Mirror API against actual XML fixtures."""
import pathlib
import sys
sys.path.insert(0, sys.argv[1])
import cadexchange_py as cad
fixtures = pathlib.Path(sys.argv[2])
model = cad.load_model(str(fixtures / 'body-tiny.xml'))
mirror = model.get_feature(0).as_mirror_pattern()
assert mirror.scope == cad.PatternScope.BODIES
assert mirror.merge_results is False
assert mirror.affected_body_mode is None
body = mirror.get_seed_object(0).as_body()
assert body.is_valid()
assert body.bounds_min == [0, 0, 0]
assert body.bounds_max == [10, 20, 30]
assert body.volume == 6000
assert body.volume_centroid == [5, 10, 15]
assert body.surface_area == 2200
assert len(body.surface_points) == 4
assert body.source_feature_ids == ['diagnostic-source-missing-in-target']
points = body.surface_points
points.clear()
assert len(body.surface_points) == 4
try:
    body.volume = 999
except AttributeError:
    pass
else:
    raise AssertionError('Body API must remain read-only')
geometry_model = cad.load_model(str(fixtures / 'selected-only-tiny.xml'))
geometry = geometry_model.get_feature(2).as_mirror_pattern()
assert geometry.affected_body_mode == cad.MirrorAffectedBodyMode.SelectedBodiesOnly
assert geometry.merge_results is None
assert len(geometry.affected_bodies) == 1
assert geometry.affected_bodies[0].volume == 6000
invalid = mirror.mirror_plane_reference.as_body()
assert not invalid.is_valid()
try:
    invalid.volume
except RuntimeError:
    pass
else:
    raise AssertionError('Invalid Body accessor must reject value access')
try:
    cad.load_model(str(fixtures / 'bad-MergeResults.xml'))
except RuntimeError:
    pass
else:
    raise AssertionError('Malformed active option must fail Python load')
print('PASS compiled Body/Mirror Python read-only API')
