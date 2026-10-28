import { gripper } from "./model.ts";

// Build the same cache that browsers read; generated files stay outside Git.
const model = await gripper(new URL("./public/pad.glts", import.meta.url).href);
model.dispose();
