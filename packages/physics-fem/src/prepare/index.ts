import type { FEMInput } from "../types.js";

interface Pending {
  resolve: (bytes: Uint8Array) => void;
  reject: (error: Error) => void;
}

interface Client {
  run: (request: string) => Promise<Uint8Array>;
}

let client: Promise<Client> | undefined;
let queue = Promise.resolve();

/** One worker owns the mesher and Python runtime; requests never overlap. */
export function prepare(input: FEMInput): Promise<Uint8Array> {
  const request = JSON.stringify(input);
  const result = queue.then(async () => {
    client ??= connect().catch((error: unknown) => {
      client = undefined;
      throw error;
    });
    return (await client).run(request);
  });
  queue = result.then(
    () => {},
    () => {},
  );
  return result;
}

async function connect(): Promise<Client> {
  let pending: Pending | undefined;
  let failure: Error | undefined;
  let send: (request: string) => void;
  let retain = () => {};
  let release = () => {};
  let terminate: () => void;

  function fail(error: Error): void {
    if (failure) return;
    failure = error;
    client = undefined;
    pending?.reject(error);
    pending = undefined;
    terminate();
  }

  function receive(value: unknown): void {
    if (!pending) {
      fail(new Error("FEM worker returned an unexpected response"));
      return;
    }
    const request = pending;
    pending = undefined;
    release();
    if (value instanceof Uint8Array) {
      request.resolve(value);
      return;
    }
    if (
      typeof value === "object" &&
      value !== null &&
      "error" in value &&
      typeof value.error === "string"
    ) {
      request.reject(new Error(value.error));
      return;
    }
    const error = new Error("FEM worker returned an invalid response");
    request.reject(error);
    fail(error);
  }

  if (typeof process !== "undefined" && process.versions?.node) {
    const { Worker } = await import("node:worker_threads");
    // This compiled module needs none of the caller's entry-point or test-runner flags.
    const worker = new Worker(new URL("./worker.js", import.meta.url), {
      execArgv: [],
    });
    send = (request) => worker.postMessage(request);
    retain = () => {
      worker.ref();
    };
    release = () => {
      worker.unref();
    };
    terminate = () => {
      void worker.terminate();
    };
    worker.on("message", receive);
    worker.on("error", fail);
    worker.on("messageerror", fail);
    worker.on("exit", (code) => {
      if (!failure) fail(new Error(`FEM worker exited unexpectedly (${code})`));
    });
    worker.unref();
  } else {
    const worker = new Worker(new URL("./worker.js", import.meta.url), {
      type: "module",
    });
    send = (request) => worker.postMessage(request);
    terminate = () => worker.terminate();
    worker.onmessage = (event: MessageEvent<unknown>) => receive(event.data);
    worker.onerror = (event) =>
      fail(new Error(event.message || "FEM worker failed"));
    worker.onmessageerror = () =>
      fail(new Error("Cannot decode FEM worker response"));
  }

  return {
    run(request) {
      if (failure) return Promise.reject(failure);
      return new Promise((resolve, reject) => {
        pending = { resolve, reject };
        retain();
        try {
          send(request);
        } catch (error) {
          fail(error instanceof Error ? error : new Error(String(error)));
        }
      });
    },
  };
}
