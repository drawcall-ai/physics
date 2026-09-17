function sameClass<T extends object>(value: unknown, source: T): value is T {
  return (
    typeof value === "object" &&
    value !== null &&
    Object.getPrototypeOf(value) === Object.getPrototypeOf(source)
  );
}

/** Constructs `source`'s exact class with `args`, so clone methods preserve subclasses. */
export function constructLike<T extends object>(source: T, args: unknown[]): T {
  const target: unknown = Reflect.construct(source.constructor, args);
  if (!sameClass(target, source))
    throw new Error(
      `${source.constructor.name} clone constructor returned an incompatible object`,
    );
  return target;
}
