declare module "three/addons/loaders/usd/USDAParser.js" {
  export class USDAParser {
    parseText(text: string): unknown;
    parseData(text: string): unknown;
  }
}
declare module "three/addons/loaders/usd/USDComposer.js" {
  import type { Group, LoadingManager } from "three";
  export class USDComposer {
    constructor(manager?: LoadingManager);
    texturePromises: Promise<unknown>[];
    compose(
      data: unknown,
      assets: Record<string, unknown>,
      buffers: Record<string, unknown>,
      path: string,
    ): Group;
  }
}
