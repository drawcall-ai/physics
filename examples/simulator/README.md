# Local USDZ physics simulator

This demo uses our importer and Rapier. For independent third-party verification,
use [the Newton launcher](../../tools/README.md).

From the physics repo:

```sh
pnpm install
pnpm build
pnpm --filter @drawcall/physics-simulator dev
```

Open the Vite URL. The default scene is a falling ragdoll with a floor.

- **Open USDZ** loads a local USDZ/USDA and uses its authored gravity and physics.
- **Pause / Play** and **Reset** control simulation; drag to orbit and scroll to zoom.
- **Record video** resets and records the canvas at 30 fps. **Stop & save video**
  downloads a WebM. Recording runs in real time and contains no audio or UI.
- **Export USDZ** saves the scene in its initial pose.

Use a browser supporting WebM MediaRecorder (Chrome or Firefox). Everything runs
locally. The importer supports the ASCII USDA subset documented in
[physics-usd](../../packages/physics-usd/README.md), including USDA inside USDZ;
binary USDC and unsupported physics features produce an error. A scene needs its
own floor or other static collision bodies if objects should stop falling.

`src/ragdoll.ts` authors the sample; `src/main.ts` is the viewer and recorder.
