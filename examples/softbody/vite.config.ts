import { defineConfig } from "vite";
import { vitecache } from "vitecache/vite";

export default defineConfig({
  // Vitest's module runner cannot run vitecache's warming; tests compute their own entries.
  plugins: process.env.VITEST ? [] : [vitecache()],
  resolve: { dedupe: ["three", "@drawcall/physics"] },
  build: { target: "es2022" },
});
