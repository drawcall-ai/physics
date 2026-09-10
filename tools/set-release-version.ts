import { readFile, writeFile } from "node:fs/promises";

const version = process.argv[2]?.replace(/^v/, "");
if (!version || !/^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)$/.test(version)) {
  throw new Error("Expected a stable release tag such as v0.1.0");
}

for (const name of ["physics", "physics-rapier", "physics-usd"]) {
  const file = new URL(`../packages/${name}/package.json`, import.meta.url);
  const pkg = JSON.parse(await readFile(file, "utf8"));
  pkg.version = version;
  if (pkg.peerDependencies["@drawcall/physics"]) {
    pkg.peerDependencies["@drawcall/physics"] = `^${version}`;
  }
  await writeFile(file, `${JSON.stringify(pkg, null, 2)}\n`);
}
