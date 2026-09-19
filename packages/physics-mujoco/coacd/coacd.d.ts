export interface CoACD {
  decompose(vertices: Float64Array, indices: Int32Array): number[][];
}
export default function createCoACD(options?: {
  locateFile?: (path: string) => string;
}): Promise<CoACD>;
