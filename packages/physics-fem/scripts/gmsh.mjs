import { execFileSync, spawnSync } from "node:child_process";
import { createHash } from "node:crypto";
import {
  cpSync,
  existsSync,
  mkdirSync,
  readFileSync,
  writeFileSync,
} from "node:fs";
import { availableParallelism } from "node:os";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const wrapper = "dd00f3309cb4b4223452e6c60d1f1a07b8c2ac6a";
const gmsh = "29726e7237db13ff77ef3f2db2d7fb9499c4e65c";
const emscripten = "5.0.2";
const cache = resolve(dirname(fileURLToPath(import.meta.url)), "../.cache");
const work = process.argv[2]
  ? resolve(process.argv[2])
  : join(cache, "gmsh-build");
const output = process.argv[3] ? resolve(process.argv[3]) : join(cache, "gmsh");
const compiler = spawnSync("emcc", ["--version"], { encoding: "utf8" });
if (compiler.error || compiler.status !== 0)
  throw new Error(
    `Building FEM assets requires Emscripten ${emscripten}; activate its SDK so emcc, em++ and emcmake are on PATH`,
    { cause: compiler.error },
  );
const version = compiler.stdout.split("\n")[0];
if (!version?.includes(` ${emscripten}`))
  throw new Error(`Gmsh assets require Emscripten ${emscripten}`);

function run(command, args) {
  execFileSync(command, args, { cwd: work, stdio: "inherit" });
}

mkdirSync(work, { recursive: true });
if (!existsSync(join(work, ".git"))) {
  run("git", ["init"]);
  run("git", [
    "remote",
    "add",
    "origin",
    "https://github.com/loumalouomega/GMSH-JS.git",
  ]);
  run("git", ["fetch", "--depth", "1", "origin", wrapper]);
  run("git", ["checkout", "--detach", wrapper]);
}
const revision = execFileSync("git", ["rev-parse", "HEAD"], {
  cwd: work,
  encoding: "utf8",
}).trim();
if (revision !== wrapper)
  throw new Error("Gmsh build directory has a different wrapper revision");
run("git", ["submodule", "update", "--init", "--depth", "1"]);
const meshRevision = execFileSync("git", ["rev-parse", "HEAD"], {
  cwd: join(work, "gmsh"),
  encoding: "utf8",
}).trim();
if (meshRevision !== gmsh)
  throw new Error("Gmsh submodule has a different revision");

const options = [
  "CMAKE_BUILD_TYPE=Release",
  "CMAKE_CXX_FLAGS=-fexceptions",
  "ENABLE_BUILD_LIB=ON",
  "ENABLE_BUILD_SHARED=OFF",
  "ENABLE_BUILD_DYNAMIC=OFF",
  "ENABLE_FLTK=OFF",
  "ENABLE_GRAPHICS=OFF",
  "ENABLE_OS_SPECIFIC_INSTALL=OFF",
  "ENABLE_OPENMP=OFF",
  "ENABLE_MPI=OFF",
  "ENABLE_OCC=OFF",
  "ENABLE_EIGEN=ON",
  "ENABLE_BLAS_LAPACK=OFF",
  "ENABLE_PETSC=OFF",
  "ENABLE_SLEPC=OFF",
  "ENABLE_MUMPS=OFF",
  "ENABLE_MED=OFF",
  "ENABLE_CGNS=OFF",
  "ENABLE_MESH=ON",
  "ENABLE_PARSER=ON",
  "ENABLE_POST=ON",
  "ENABLE_HXT=OFF",
  "ENABLE_NETGEN=OFF",
  "ENABLE_TESTS=OFF",
  "GMSH_PACKAGER=drawcall-physics-fem-serial",
  "GMSH_HOST=emscripten",
];
run("emcmake", [
  "cmake",
  "-S",
  "gmsh",
  "-B",
  "build-serial",
  ...options.map((value) => `-D${value}`),
]);
run("cmake", [
  "--build",
  "build-serial",
  "--target",
  "lib",
  "--",
  `-j${Math.min(8, availableParallelism())}`,
]);
run("python3", ["scripts/gen_js.py"]);
mkdirSync(join(work, "dist"), { recursive: true });
run("em++", [
  "build-serial/libgmsh.a",
  "-O3",
  "-fexceptions",
  "-sMODULARIZE=1",
  "-sEXPORT_NAME=initGmsh",
  "-sEXPORT_ES6=1",
  "-sALLOW_MEMORY_GROWTH=1",
  "-sINITIAL_MEMORY=64MB",
  "-sMAXIMUM_MEMORY=4GB",
  "-sMALLOC=emmalloc",
  "-sSTACK_SIZE=4MB",
  "-sALLOW_TABLE_GROWTH=1",
  "-sFORCE_FILESYSTEM=1",
  "-sEXPORTED_RUNTIME_METHODS=FS,ccall,cwrap,getValue,setValue,UTF8ToString,stringToUTF8,lengthBytesUTF8,wasmMemory,addFunction,removeFunction,HEAP8,HEAPU8,HEAP16,HEAPU16,HEAP32,HEAPU32,HEAPF32,HEAPF64",
  "-sEXPORTED_FUNCTIONS=@generated/exported_functions.json",
  "-sENVIRONMENT=node,web,worker",
  "-o",
  "dist/gmsh-core.mjs",
]);
run("node", ["scripts/assemble.mjs"]);

mkdirSync(output, { recursive: true });
const files = [
  "gmsh.mjs",
  "gmsh-core.mjs",
  "gmsh-core.wasm",
  "gmsh-descriptor.mjs",
  "runtime.mjs",
];
for (const file of files) cpSync(join(work, "dist", file), join(output, file));
cpSync(join(work, "LICENSE"), join(output, "GMSH-JS-LICENSE.txt"));
cpSync(join(work, "gmsh/LICENSE.txt"), join(output, "GMSH-LICENSE.txt"));
cpSync(join(work, "gmsh/CREDITS.txt"), join(output, "GMSH-CREDITS.txt"));
const script = fileURLToPath(import.meta.url);
if (script !== join(output, "build.mjs"))
  cpSync(script, join(output, "build.mjs"));
writeFileSync(
  join(output, "provenance.json"),
  JSON.stringify(
    {
      wrapper: {
        repository: "https://github.com/loumalouomega/GMSH-JS",
        revision: wrapper,
      },
      gmsh: {
        repository: "https://gitlab.onelab.info/gmsh/gmsh",
        revision: gmsh,
      },
      emscripten,
      options,
      files: Object.fromEntries(
        [
          ...files,
          "build.mjs",
          "GMSH-LICENSE.txt",
          "GMSH-JS-LICENSE.txt",
          "GMSH-CREDITS.txt",
        ].map((file) => [
          file,
          createHash("sha256")
            .update(readFileSync(join(output, file)))
            .digest("hex"),
        ]),
      ),
    },
    null,
    2,
  ) + "\n",
);
console.log(`Built serial Gmsh assets in ${output}; build directory: ${work}`);
