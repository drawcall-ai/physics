/** Complete every cleanup action, then propagate all failures without hiding the first. */
export function cleanup(
  actions: readonly (() => void)[],
  message: string,
): void {
  const errors = cleanupErrors(actions);
  if (errors.length === 1) throw errors[0];
  if (errors.length > 1) throw new AggregateError(errors, message);
}

/** Release partial work, then propagate the original failure and any cleanup failures. */
export function rollback(
  error: unknown,
  actions: readonly (() => void)[],
  message: string,
): never {
  const errors = cleanupErrors(actions);
  if (errors.length === 0) throw error;
  throw new AggregateError([error, ...errors], message);
}

function cleanupErrors(actions: readonly (() => void)[]): unknown[] {
  const errors: unknown[] = [];
  for (const action of actions) {
    try {
      action();
    } catch (error) {
      errors.push(error);
    }
  }
  return errors;
}
