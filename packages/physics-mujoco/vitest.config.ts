import { defineConfig } from "vitest/config";

export default defineConfig({
  test: {
    // Execute generated vendor code natively, as Vitest already does for node_modules.
    server: { deps: { external: [/\/physics-mujoco\/generated\//] } },
  },
});
