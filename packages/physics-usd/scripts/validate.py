"""Validate the generated fixture with OpenUSD, independently of our JS importer."""
import math
import struct
import sys
import zipfile
from pxr import Gf, Usd, UsdGeom, UsdPhysics

path = sys.argv[1]
scale = float(sys.argv[2]) if len(sys.argv) > 2 else 1
stage = Usd.Stage.Open(path)
assert stage
assert UsdGeom.GetStageMetersPerUnit(stage) == 1
assert UsdPhysics.GetStageKilogramsPerUnit(stage) == 1
prims = list(stage.Traverse())
bodies = [prim for prim in prims if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
colliders = [prim for prim in prims if prim.HasAPI(UsdPhysics.CollisionAPI)]
joints = [UsdPhysics.RevoluteJoint(prim) for prim in prims if prim.IsA(UsdPhysics.RevoluteJoint)]
assert len(bodies) == 1 and len(colliders) == 2 and len(joints) == 1
joint = joints[0]
assert joint.GetUpperLimitAttr().Get() == 90
assert joint.GetAxisAttr().Get() == 'Y'
assert not any(schema.startswith("PhysicsDriveAPI") for schema in joint.GetPrim().GetAppliedSchemas())
anchors = []
for body_rel, pos_attr in [(joint.GetBody0Rel(), joint.GetLocalPos0Attr()), (joint.GetBody1Rel(), joint.GetLocalPos1Attr())]:
    body = stage.GetPrimAtPath(body_rel.GetTargets()[0])
    transform = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    anchors.append(transform.Transform(Gf.Vec3d(pos_attr.Get())))
assert (anchors[0] - anchors[1]).GetLength() < 1e-5
assert (anchors[0] - Gf.Vec3d(2, 3 + scale, 4)).GetLength() < 1e-5
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_], False, True)
for collider in colliders:
    body = collider.GetParent()
    visual = next(child for child in Usd.PrimRange(body) if child.IsA(UsdGeom.Mesh))
    a = cache.ComputeWorldBound(collider).ComputeAlignedRange()
    b = cache.ComputeWorldBound(visual).ComputeAlignedRange()
    assert (a.GetMin() - b.GetMin()).GetLength() < 1e-5
    assert (a.GetMax() - b.GetMax()).GetLength() < 1e-5
assert all(prim.GetPath().HasPrefix(stage.GetDefaultPrim().GetPath()) for prim in [*bodies, *colliders, joint.GetPrim()])
with zipfile.ZipFile(path) as archive, open(path, 'rb') as file:
    for entry in archive.infolist():
        file.seek(entry.header_offset)
        header = file.read(30)
        name_length, extra_length = struct.unpack_from('<HH', header, 26)
        assert (entry.header_offset + 30 + name_length + extra_length) % 64 == 0
        assert entry.compress_type == zipfile.ZIP_STORED
print('OpenUSD schemas, visual/collider bounds, anchors, angular units and USDZ alignment verified')
