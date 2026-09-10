import { copyFile, readFile } from "node:fs/promises";

const root = new URL("../", import.meta.url);
const { name } = JSON.parse(await readFile("package.json", "utf8"));

await copyFile(new URL("LICENSE", root), "LICENSE");
if (name === "@drawcall/physics") {
  await copyFile(new URL("README.md", root), "README.md");
}
