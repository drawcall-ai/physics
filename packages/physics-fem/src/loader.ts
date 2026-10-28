import { Loader, LoaderUtils, type Object3D } from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { GLTSLoader, getGLTSSource } from "@drawcall/glts";
import { rollback } from "@drawcall/physics/backend";
import { SoftBody } from "@drawcall/physics";
import { parse } from "./model.js";
import { cache } from "./cache.js";
import { input } from "./input.js";
import { sceneOf, source, type Source } from "./source.js";
import type { FEMOptions, FEMRequest } from "./types.js";

/**
 * Prepares static, closed glTF or GLTS solids as reduced SoftBody components with bound visuals.
 * Scale and local mesh transforms are baked. Registered manager handlers override built-in loaders.
 * GLTS source files and observed dependencies participate in cache invalidation.
 */
export class FEMLoader extends Loader<SoftBody, FEMRequest> {
  private gltf: GLTFLoader | undefined;
  private glts: GLTSLoader | undefined;
  private disposed = false;
  private cachePath: string | undefined;

  /** Node disk directory or browser URL prefix; coordinate these locations when deploying prepared cache files. */
  setCachePath(path: string): this {
    this.cachePath = path;
    return this;
  }

  override load(
    request: FEMRequest,
    onLoad: (body: SoftBody) => void,
    _onProgress?: (event: ProgressEvent) => void,
    onError?: (error: unknown) => void,
  ): void {
    this.loadAsync(request).then(
      onLoad,
      onError ?? ((error: unknown) => console.error(error)),
    );
  }

  override async loadAsync(request: FEMRequest): Promise<SoftBody> {
    this.assertActive();
    if (request.object !== undefined) {
      if (request.url !== undefined)
        throw new Error("FEMLoader expects either url or object");
      const dependencies = files(getGLTSSource());
      return this.build(source(request.object), request, dependencies);
    }
    let dispose: (() => void) | undefined;
    const url = LoaderUtils.resolveURL(request.url, this.path);
    this.manager.itemStart(url);
    try {
      const asset = await this.loadSource(url);
      dispose = asset.dispose;
      return await this.build(source(asset.object, true), request, asset.files);
    } catch (error) {
      rollback(
        error,
        [() => this.manager.itemError(url), () => dispose?.()],
        "FEM loading failed",
      );
    } finally {
      this.manager.itemEnd(url);
    }
  }

  /** Releases owned GLTS scripts and their resources. Pending and future preparations reject. Custom handlers remain caller-owned. */
  dispose(): void {
    this.disposed = true;
    this.glts?.dispose();
  }

  private assertActive(): void {
    if (this.disposed) throw new Error("FEMLoader has been disposed");
  }

  private async build(
    snapshot: Source,
    options: FEMOptions,
    files: readonly string[],
  ): Promise<SoftBody> {
    const request = input(snapshot, options);
    const entry = await cache(request, files, this.cachePath);
    let bytes = await entry.load();
    const missing = bytes === undefined;
    if (bytes === undefined) {
      const { prepare } = await import("./prepare/index.js");
      bytes = await prepare(request);
    }
    const data: unknown = JSON.parse(new TextDecoder().decode(bytes));
    const model = parse(data);
    if (
      !equal(model.renderPositions, request.positions) ||
      !equal(model.renderTriangles, request.triangles) ||
      !equal(model.mountRegion.min, request.mount.min) ||
      !equal(model.mountRegion.max, request.mount.max) ||
      !equal(model.contactRegion.min, request.contact.min) ||
      !equal(model.contactRegion.max, request.contact.max)
    )
      throw new Error(
        "FEMLoader reduction does not match the source geometry or physical recipe",
      );
    const body = new SoftBody({ model });
    body.setMaterial(request.material);
    if (missing) await entry.save(bytes);
    this.assertActive();
    body.add(snapshot.visual);
    for (const skin of snapshot.skins)
      body.bindGeometry(skin.mesh, skin.vertices);
    body.position.setFromMatrixPosition(snapshot.pose);
    body.quaternion.setFromRotationMatrix(snapshot.pose);
    body.name = snapshot.visual.name;
    return body;
  }

  private async loadSource(url: string): Promise<{
    object: Object3D;
    files: readonly string[];
    dispose?: () => void;
  }> {
    const path = url.split(/[?#]/)[0] ?? "";
    const extension = /\.(glb|gltf|glts)$/i.exec(path)?.[1]?.toLowerCase();
    if (!extension)
      throw new Error("FEMLoader expects a .glb, .gltf or .glts asset");
    const handler =
      this.manager.getHandler(url) ?? this.manager.getHandler(path);
    if (handler) {
      const object = sceneOf(await handler.loadAsync(url));
      return {
        object,
        files: extension === "glts" ? files(Reflect.get(object, "source")) : [],
      };
    }
    if (extension === "glts") {
      this.glts ??= new GLTSLoader(this.manager);
      this.glts
        .setCrossOrigin(this.crossOrigin)
        .setWithCredentials(this.withCredentials)
        .setRequestHeader(this.requestHeader);
      const object = await this.glts.loadAsync(url);
      return {
        object,
        files: files(object.source),
        dispose: () => object.dispose(),
      };
    }
    this.gltf ??= new GLTFLoader(this.manager);
    this.gltf
      .setCrossOrigin(this.crossOrigin)
      .setWithCredentials(this.withCredentials)
      .setRequestHeader(this.requestHeader);
    return { object: (await this.gltf.loadAsync(url)).scene, files: [] };
  }
}

function equal(left: readonly number[], right: readonly number[]): boolean {
  return (
    left.length === right.length &&
    left.every((value, index) => value === right[index])
  );
}

function files(source: unknown): readonly string[] {
  if (typeof source !== "object" || source === null)
    throw new Error("FEMLoader requires GLTS source metadata");
  const url: unknown = Reflect.get(source, "url");
  const dependencies: unknown = Reflect.get(source, "dependencyUrls");
  if (
    typeof url !== "string" ||
    !Array.isArray(dependencies) ||
    !dependencies.every((url: unknown) => typeof url === "string")
  )
    throw new Error("FEMLoader requires GLTS source metadata");
  return Object.freeze([url, ...dependencies]);
}
