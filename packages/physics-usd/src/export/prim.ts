export class Prim {
  readonly schemas: string[] = [];
  readonly properties: string[] = [];
  readonly children: Prim[] = [];
  displayName?: string;

  constructor(
    readonly name: string,
    public type = "Xform",
  ) {}

  write(depth = 0): string {
    const pad = "  ".repeat(depth);
    const entries = [];
    if (this.schemas.length)
      entries.push(`prepend apiSchemas = ${JSON.stringify(this.schemas)}`);
    if (this.displayName !== undefined)
      entries.push(`displayName = ${JSON.stringify(this.displayName)}`);
    const metadata = entries.length
      ? ` (\n${entries.map((value) => `${pad}  ${value}`).join("\n")}\n${pad})`
      : "";
    const contents = [
      ...this.properties.map((value) => `${pad}  ${value}`),
      ...this.children.map((child) => child.write(depth + 1)),
    ].join("\n");
    return `${pad}def ${this.type ? this.type + " " : ""}${JSON.stringify(this.name)}${metadata}\n${pad}{\n${contents}\n${pad}}`;
  }
}
