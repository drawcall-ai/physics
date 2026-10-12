/** Decomposes meshes off the main thread in browsers. */
import { decompose } from "./decompose.js";

export interface Request {
  positions: Float64Array;
  indices: Int32Array;
}
export type Reply = { parts: number[][] } | { error: string };

/** The worker's global scope, which the DOM library types as a window. */
interface Scope {
  addEventListener(
    type: "message",
    listener: (event: MessageEvent<Request>) => void,
  ): void;
  postMessage(reply: Reply): void;
}
const scope: Scope = globalThis;

scope.addEventListener("message", async ({ data }) => {
  const { positions, indices } = data;
  try {
    scope.postMessage({ parts: await decompose(positions, indices) });
  } catch (error) {
    scope.postMessage({
      error: error instanceof Error ? error.message : String(error),
    });
  }
});
