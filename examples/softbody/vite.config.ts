import { defineConfig } from "vite";
import { glts } from "@drawcall/glts/vite";
import { vitecache } from "vitecache/vite";

export default defineConfig({
  plugins: [glts(), vitecache()],
  resolve: { dedupe: ["three", "@drawcall/physics", "@drawcall/glts"] },
  build: { target: "es2022" },
});
