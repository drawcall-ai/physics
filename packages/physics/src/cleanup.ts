/** Complete every cleanup action, then propagate all failures without hiding the first. */
export function cleanup(
  actions: readonly (() => void)[],
  message: string,
): void {
  const errors: unknown[] = [];
  for (const action of actions) {
    try {
      action();
    } catch (error) {
      errors.push(error);
    }
  }
  if (errors.length === 1) throw errors[0];
  if (errors.length > 1) throw new AggregateError(errors, message);
}
