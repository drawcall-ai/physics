/**
 * Decomposes on the calling thread, as Node, Bun and Deno do; browser bundles take
 * `run.browser.ts` instead, through the package's `browser` field.
 */
export async function run(
  positions: Float64Array,
  indices: Int32Array,
): Promise<number[][]> {
  // Imported on demand, so CoACD loads only once a mesh needs it.
  const { decompose } = await import("./decompose.js");
  return decompose(positions, indices);
}
