import * as THREE from "three";
import { GenericJoint, JointDrive, RigidBody } from "@drawcall/physics";

/** Critically damped 5 Hz spring per kilogram, capped at roughly one arm's pull. */
const spring = {
  model: "acceleration",
  stiffness: 1000,
  damping: 63,
  maxForce: 600,
} as const;
const marker = new THREE.SphereGeometry(0.04);
const markerMaterial = new THREE.MeshStandardMaterial({ color: "#ffffff" });

/**
 * Holds a dynamic body at a world point: a kinematic hand and a free six-axis joint whose
 * translation drives pull the body's grab point to the hand. The body stays dynamic, so
 * contacts still stop it while the hand keeps moving. A controller with orientation would
 * add drives on the rotation axes.
 */
export function grab(
  scene: THREE.Object3D,
  body: RigidBody,
  point: THREE.Vector3,
) {
  const hand = new RigidBody({ type: "kinematic", colliders: false });
  hand.name = "Hand";
  hand.add(new THREE.Mesh(marker, markerMaterial));
  hand.position.copy(point);
  scene.add(hand);
  const joint = new GenericJoint({
    body0: hand,
    body1: body,
    frame0: new THREE.Matrix4(),
    frame1: new THREE.Matrix4().makeTranslation(
      body.worldToLocal(point.clone()),
    ),
    dofs: {
      transX: "free",
      transY: "free",
      transZ: "free",
      rotX: "free",
      rotY: "free",
      rotZ: "free",
    },
  });
  for (const axis of ["transX", "transY", "transZ"] as const)
    joint.setDrive(axis, new JointDrive(spring).setTarget({ position: 0 }));
  return {
    body,
    move(target: THREE.Vector3) {
      hand.setKinematicTarget(new THREE.Matrix4().makeTranslation(target));
    },
    /** Disposing the hand releases its joint and drives. */
    release() {
      hand.dispose();
    },
  };
}
