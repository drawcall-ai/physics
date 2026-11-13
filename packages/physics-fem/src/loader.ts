import { Loader, LoaderUtils, type Object3D } from "three";
import { GLTSLoader } from "@drawcall/glts";
import { publicURL } from "vitecache";
import { rollback } from "@drawcall/physics/backend";
import type { SoftBody } from "@drawcall/physics";
import { build, reduce } from "./body.js";
import { cache } from "./cache.js";
import { input } from "./input.js";
import { sceneOf, source } from "./source.js";
import type { FEMRequest } from "./types.js";

/**
 * Prepares static, closed GLTS solids as reduced SoftBodies and caches them. The vitecache plugin warms each literal
 * `new CachedFEMLoader().loadAsync({ ... })` in Node, so browsers read the prepared result.
 * The loaded scene becomes the body's visual and is caller-owned; a GLTS scene keeps running until disposed.
 * Scale and local mesh transforms are baked. A registered manager handler loads its URLs instead; the plugin warms
 * without one, so it prepares only the GLTS file.
 */
export class CachedFEMLoader extends Loader<SoftBody, FEMRequest> {
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
    const url = LoaderUtils.resolveURL(request.url, this.path);
    let dispose: (() => void) | undefined;
    this.manager.itemStart(url);
    try {
      const asset = await this.loadSource(url);
      dispose = asset.dispose;
      const snapshot = source(asset.object, true);
      const recipe = input(snapshot, request);
      const entry = cache(recipe);
      const cached = await entry.load();
      const bytes = cached ?? (await reduce(recipe));
      const body = build(snapshot, recipe, bytes);
      if (cached === undefined) await entry.save(bytes);
      return body;
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

  private async loadSource(
    url: string,
  ): Promise<{ object: Object3D; dispose?: () => void }> {
    const path = url.split(/[?#]/)[0] ?? "";
    const handler =
      this.manager.getHandler(url) ?? this.manager.getHandler(path);
    if (handler) return { object: sceneOf(await handler.loadAsync(url)) };
    if (!/\.glts$/i.test(path))
      throw new Error("CachedFEMLoader expects a .glts asset");
    // In Node, which warms the cache, a root-absolute URL names a file in publicDir.
    const object = await new GLTSLoader(this.manager)
      .setCrossOrigin(this.crossOrigin)
      .setWithCredentials(this.withCredentials)
      .setRequestHeader(this.requestHeader)
      .loadAsync(publicURL(url));
    return { object, dispose: () => object.dispose() };
  }
}
