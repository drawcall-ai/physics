import { prepare } from "./runtime.js";

type Send = (
  message: Uint8Array<ArrayBuffer> | { error: string },
  transfer?: ArrayBuffer[],
) => void;

async function handle(message: unknown, send: Send): Promise<void> {
  try {
    if (typeof message !== "string")
      throw new Error("FEM worker expected a JSON request");
    const bytes = await prepare(message);
    send(bytes, [bytes.buffer]);
  } catch (error) {
    send({ error: error instanceof Error ? error.message : String(error) });
  }
}

async function start() {
  if (typeof process !== "undefined" && process.versions?.node) {
    const { parentPort } = await import("node:worker_threads");
    if (!parentPort) throw new Error("FEM preparation must run in a worker");
    const port = parentPort;
    port.on("message", (request: unknown) => {
      void handle(request, (message, transfer) =>
        port.postMessage(message, transfer),
      );
    });
    return;
  }
  globalThis.onmessage = (event: MessageEvent<unknown>) => {
    void handle(event.data, (message, transfer) =>
      globalThis.postMessage(message, { transfer }),
    );
  };
}

void start();
