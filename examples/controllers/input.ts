export function bindInput(
  canvas: HTMLCanvasElement,
  press: (code: string) => void,
  reset: () => void,
) {
  const keys = new Set<string>();
  const keyboard = new Set<string>();
  const pointers = new Map<number, string>();
  const events = new AbortController();
  const options = { signal: events.signal };
  const codes = new Set([
    "KeyW",
    "KeyA",
    "KeyS",
    "KeyD",
    "ArrowUp",
    "ArrowLeft",
    "ArrowDown",
    "ArrowRight",
    "Space",
    "ShiftLeft",
    "ShiftRight",
    "KeyC",
    "ControlLeft",
    "KeyQ",
    "KeyE",
  ]);
  function update() {
    const next = new Set([...keyboard, ...pointers.values()]);
    for (const code of next) if (!keys.has(code)) press(code);
    keys.clear();
    for (const code of next) keys.add(code);
  }
  function clear() {
    keyboard.clear();
    pointers.clear();
    keys.clear();
  }
  window.addEventListener(
    "keydown",
    (event) => {
      if (
        event.target instanceof HTMLSelectElement ||
        event.target instanceof HTMLButtonElement ||
        event.target instanceof HTMLAnchorElement
      )
        return;
      if (event.code === "KeyR") {
        event.preventDefault();
        reset();
        return;
      }
      if (!codes.has(event.code)) return;
      event.preventDefault();
      keyboard.add(event.code);
      update();
    },
    options,
  );
  window.addEventListener(
    "keyup",
    (event) => {
      keyboard.delete(event.code);
      update();
    },
    options,
  );
  window.addEventListener("blur", clear, options);
  canvas.addEventListener("pointerdown", () => canvas.focus(), options);
  for (const button of Array.from(
    document.querySelectorAll<HTMLButtonElement>("[data-key]"),
  )) {
    const code = button.dataset.key;
    if (!code) throw new Error("Missing touch control binding");
    button.addEventListener(
      "pointerdown",
      (event) => {
        event.preventDefault();
        button.setPointerCapture(event.pointerId);
        pointers.set(event.pointerId, code);
        update();
      },
      options,
    );
    const release = (event: PointerEvent) => {
      pointers.delete(event.pointerId);
      update();
    };
    button.addEventListener("pointerup", release, options);
    button.addEventListener("pointercancel", release, options);
    button.addEventListener("lostpointercapture", release, options);
  }
  return {
    keys,
    clear,
    dispose() {
      clear();
      events.abort();
    },
  };
}
