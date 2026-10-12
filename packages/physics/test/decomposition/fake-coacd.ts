/** Stands in for the CoACD module: every mesh decomposes into `part`. */
import { vi } from "vitest";

export const part = [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1];
export const decompose = vi.fn(() => ({
  hulls: Object.assign(
    [
      {
        vertices: Object.assign([...part], { delete() {} }),
        indices: { delete() {} },
      },
    ],
    { delete() {} },
  ),
}));
/** Loads the module, as the generated factory does. */
const load = vi.fn(async () => ({ decompose }));
export default load;
