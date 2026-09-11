type Kind =
  "box" | "sphere" | "capsule" | "cylinder" | "convex hull" | "triangle mesh";
type Placement = "body" | "ancestor" | "collider" | "combined";
export interface Case {
  name: string;
  kind: Kind;
  placement: Placement;
  scale: [number, number, number];
  explicit: boolean;
  compound?: boolean;
  type?: "dynamic" | "static" | "kinematic";
  geometry?: "sphere" | "capsule";
  error?: string;
  shear?: boolean;
  edit?: boolean;
}
const kinds: Kind[] = [
  "box",
  "sphere",
  "capsule",
  "cylinder",
  "convex hull",
  "triangle mesh",
];
const placements: Placement[] = ["body", "ancestor", "collider", "combined"];
export const cases: Case[] = [];
for (const kind of kinds) {
  for (const explicit of [false, true]) {
    for (const placement of placements) {
      for (const uniform of [true, false]) {
        const scale: [number, number, number] = uniform
          ? [1.5, 1.5, 1.5]
          : kind === "cylinder"
            ? [1.5, 2, 1.5]
            : [1.25, 2, 1.75];
        const error =
          !uniform && placement === "ancestor" && kind !== "triangle mesh"
            ? "uniform ancestor"
            : !uniform && (kind === "sphere" || kind === "capsule")
              ? "nonuniform"
              : undefined;
        cases.push({
          name: `${kind === "triangle mesh" && !explicit ? "triangle mesh (3 children)" : kind} · ${explicit ? "explicit" : "auto"} · ${placement} · ${uniform ? "uniform" : "nonuniform"}`,
          kind,
          placement,
          scale,
          explicit,
          ...(error ? { error } : {}),
        });
      }
    }
  }
}
for (const [name, scale, error] of [
  ["Box: zero scale", [0, 1, 1], "positive scale"],
  ["Box: negative scale", [-1, 1, 1], "positive scale"],
  ["Box: two negative axes", [-1, -1, 1], "positive scale"],
  ["Cylinder: unequal radial axes", [2, 1, 1], "nonuniform"],
] satisfies [string, [number, number, number], string][]) {
  cases.push({
    name,
    kind: name.startsWith("Cylinder") ? "cylinder" : "box",
    placement: "body",
    scale,
    explicit: true,
    error,
  });
}
cases.push(
  {
    name: "Box: shear rejected",
    kind: "box",
    placement: "collider",
    scale: [1, 1, 1],
    explicit: true,
    shear: true,
    error: "shear",
  },
  {
    name: "Box: live scale edit rejected",
    kind: "box",
    placement: "body",
    scale: [1, 1, 1],
    explicit: false,
    edit: true,
    error: "scale cannot change",
  },
  {
    name: "Kinematic box: nonuniform body",
    kind: "box",
    placement: "body",
    scale: [2, 1, 1.5],
    explicit: true,
    type: "kinematic",
  },
  {
    name: "Kinematic box: nonuniform ancestor rejected",
    kind: "box",
    placement: "ancestor",
    scale: [2, 1, 1.5],
    explicit: true,
    type: "kinematic",
    error: "uniform ancestor",
  },
);

for (const geometry of ["sphere", "capsule"] as const) {
  cases.push({
    name: `Stretched ${geometry}: convex mesh fallback`,
    kind: "convex hull",
    geometry,
    placement: "body",
    scale: [1.5, 2, 1],
    explicit: true,
  });
}
cases.push({
  name: "Dynamic triangle mesh rejected",
  kind: "triangle mesh",
  type: "dynamic",
  placement: "body",
  scale: [2, 2, 2],
  explicit: true,
  error: "Triangle mesh colliders require static bodies",
});
for (const kind of [
  "box",
  "sphere",
  "capsule",
  "cylinder",
  "convex hull",
] as const) {
  cases.push({
    name: `Static ${kind}: nonuniform ancestor`,
    kind,
    type: "static",
    placement: "ancestor",
    scale: [1.5, 2, 1.5],
    explicit: true,
    ...(kind === "sphere" || kind === "capsule" ? { error: "nonuniform" } : {}),
  });
}

for (const placement of ["body", "combined"] as const) {
  cases.push({
    name: `Falling compound: 3 convex colliders · ${placement} scale`,
    kind: "convex hull",
    placement,
    scale: [1.25, 1.5, 1.25],
    explicit: false,
    compound: true,
  });
}
cases.push({
  name: "Kinematic lift: uniform ancestor scale",
  kind: "box",
  placement: "ancestor",
  scale: [1.5, 1.5, 1.5],
  explicit: true,
  type: "kinematic",
});
