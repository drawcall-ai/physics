import { writeFile } from "node:fs/promises";
import { join } from "node:path";
import { pathToFileURL } from "node:url";
import { expect, it } from "vitest";
import { BoxGeometry, Loader, LoadingManager, Mesh } from "three";
import { FEMLoader } from "../src/loader.js";
import { fixture, folder, options, reducer, sourceFolder } from "./fixtures.js";

it("rejects objects outside GLTS and GLTS handlers without source metadata", async () => {
  const object = new Mesh(new BoxGeometry());
  await expect(
    new FEMLoader().loadAsync({ object, ...options }),
  ).rejects.toThrow("No GLTS execution is active");
  class MissingSource extends Loader<Mesh> {
    override async loadAsync() {
      return object;
    }
  }
  const manager = new LoadingManager().addHandler(
    /\.glts$/,
    new MissingSource(),
  );
  await expect(
    new FEMLoader(manager).loadAsync({ url: "finger.glts", ...options }),
  ).rejects.toThrow("GLTS source metadata");
  expect(reducer).not.toHaveBeenCalled();
  expect(object.parent).toBeNull();
});

it("invalidates unchanged geometry when a helper changes and rejects changes during preparation", async () => {
  const directory = await sourceFolder();
  const url = pathToFileURL(join(directory, "finger.glts")).href;
  const helper = pathToFileURL(join(directory, "helper.ts"));
  await writeFile(helper, "export const size = 1;");
  await writeFile(
    new URL(url),
    `
    import { Mesh, BoxGeometry } from "three";
    import { scene } from "@drawcall/glts";
    import { size } from "./helper.ts";
    scene.add(new Mesh(new BoxGeometry(size, size, size)));
  `,
  );
  const loader = new FEMLoader().setCachePath(await folder());
  reducer.mockImplementation(async (input) => fixture(input));
  try {
    const first = await loader.loadAsync({ url, ...options });
    await loader.loadAsync({ url, ...options });
    expect(reducer).toHaveBeenCalledTimes(1);
    await writeFile(helper, "export const size = 1; // changed source");
    const second = await loader.loadAsync({ url, ...options });
    expect(second.model?.positions).toEqual(first.model?.positions);
    expect(reducer).toHaveBeenCalledTimes(2);
    reducer.mockImplementationOnce(async (input) => {
      await writeFile(
        helper,
        "export const size = 1; // changed during reduction",
      );
      return fixture(input);
    });
    const request = {
      url,
      ...options,
      material: { ...options.material, density: 1200 },
    };
    await expect(loader.loadAsync(request)).rejects.toThrow(
      "dependencies changed during computation",
    );
    await loader.loadAsync(request);
    expect(reducer).toHaveBeenCalledTimes(4);
  } finally {
    loader.dispose();
  }
});
