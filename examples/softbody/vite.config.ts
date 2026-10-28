import { defineConfig } from "vite";
import { vitecache } from "vitecache/vite";

export default defineConfig({
  plugins: [vitecache()],
  resolve: { dedupe: ["three", "@drawcall/physics", "@drawcall/glts"] },
  build: { target: "es2022" },
});
