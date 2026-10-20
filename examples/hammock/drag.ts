import * as THREE from "three";
import type { RigidBody, PhysicsWorld } from "@drawcall/physics";
import type { view } from "../view";

export function dragBars(
  demo: ReturnType<typeof view>,
  world: PhysicsWorld,
  bars: readonly RigidBody[],
) {
  const raycaster = new THREE.Raycaster();
  const plane = new THREE.Plane();
  const pointer = new THREE.Vector2();
  const target = new THREE.Vector3();
  let grab: { body: RigidBody; localPoint: THREE.Vector3 } | undefined;

  function ray(event: PointerEvent) {
    const rect = demo.canvas.getBoundingClientRect();
    pointer.set(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      1 - ((event.clientY - rect.top) / rect.height) * 2,
    );
    raycaster.setFromCamera(pointer, demo.camera);
  }

  demo.canvas.addEventListener("pointerdown", (event) => {
    if (event.button !== 0) return;
    ray(event);
    const hit = raycaster.intersectObjects([...bars], true)[0];
    if (!hit) return;
    const body = bars.find((bar) => bar.getObjectById(hit.object.id));
    if (!body) return;
    target.copy(hit.point);
    plane.setFromNormalAndCoplanarPoint(
      demo.camera.getWorldDirection(new THREE.Vector3()),
      target,
    );
    grab = { body, localPoint: body.worldToLocal(hit.point.clone()) };
    demo.controls.enabled = false;
    demo.canvas.setPointerCapture(event.pointerId);
  });
  demo.canvas.addEventListener("pointermove", (event) => {
    if (!grab) return;
    ray(event);
    raycaster.ray.intersectPlane(plane, target);
  });
  function release() {
    grab = undefined;
    demo.controls.enabled = true;
  }
  demo.canvas.addEventListener("pointerup", release);
  demo.canvas.addEventListener("pointercancel", release);
  window.addEventListener("blur", release);
  world.onBeforeStep(() => {
    if (!grab) return;
    const { body, localPoint } = grab;
    const point = body.localToWorld(localPoint.clone());
    const force = target.clone().sub(point).multiplyScalar(120);
    force.addScaledVector(body.getVelocity().linear, -12).clampLength(0, 80);
    body.applyForce(force, point);
  });
  return release;
}
