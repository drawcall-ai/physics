if (new URL(location.href).searchParams.get("demo") === "decomposition") {
  await import("./decomposition-main");
} else {
  await import("./main");
}
export {};
