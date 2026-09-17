import * as THREE from "three";
import { BoxCollider, Trigger, type RigidBody } from "@drawcall/physics";

export function createGoal(car: RigidBody) {
  const trigger = new Trigger();
  trigger.name = "Finish zone";
  trigger.position.set(0, 1, 62);
  // The car accepts category 1 and belongs to category 2, just like on the road.
  trigger.setCollisionGroups({ membership: 1, filter: 2 });
  trigger.add(new BoxCollider({ size: [7.2, 2, 8] }));

  const material = new THREE.MeshBasicMaterial({
    color: "#56b9df",
    transparent: true,
    opacity: 0.12,
    depthWrite: false,
  });
  trigger.add(new THREE.Mesh(new THREE.BoxGeometry(7.2, 2, 8), material));
  const border = new THREE.MeshBasicMaterial({ color: "#56b9df" });
  for (const x of [-3.6, 3.6]) {
    const stripe = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.02, 8), border);
    stripe.position.set(x, -0.98, 0);
    trigger.add(stripe);
  }
  for (const z of [-4, 4]) {
    const stripe = new THREE.Mesh(
      new THREE.BoxGeometry(7.2, 0.02, 0.08),
      border,
    );
    stripe.position.set(0, -0.98, z);
    trigger.add(stripe);
  }
  function color(value: string) {
    material.color.set(value);
    border.color.set(value);
  }
  trigger.addEventListener("enter", ({ body }) => {
    if (body === car) color("#84d78a");
  });
  trigger.addEventListener("exit", ({ body }) => {
    if (body === car) color("#56b9df");
  });
  return {
    trigger,
    reset() {
      // World reset clears occupancy without emitting exit events.
      color("#56b9df");
    },
  };
}
