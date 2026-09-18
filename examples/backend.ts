const url = new URL(location.href);
export const backend = url.searchParams.get("backend") ?? "rapier";
if (backend !== "rapier" && backend !== "mujoco")
  throw new Error(`Unknown physics backend: ${backend}`);
const nav = document.querySelector("nav");
if (!nav) throw new Error("Missing example navigation");
const label = document.createElement("label");
label.textContent = "Backend ";
const select = document.createElement("select");
select.setAttribute("aria-label", "Physics backend");
select.title = "Changing the backend restarts the simulation";
select.add(new Option("Rapier", "rapier"));
select.add(new Option("MuJoCo WASM", "mujoco"));
select.value = backend;
label.append(select);
nav.append(label);
select.addEventListener("change", () => {
  url.searchParams.set("backend", select.value);
  // A fresh page also releases the renderer, input handlers, and WASM world.
  location.assign(url);
});
window.addEventListener("unhandledrejection", (event) => {
  const status = document.querySelector("output");
  if (status) status.textContent = String(event.reason);
});
