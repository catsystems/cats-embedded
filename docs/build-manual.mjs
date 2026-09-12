import { mkdirSync } from "node:fs";
import { spawnSync } from "node:child_process";

const expectedHtmlWarning = [
  "warning: html export is under active development and incomplete",
  " = hint: its behaviour may change at any time",
  " = hint: do not rely on this feature for production use cases",
  " = hint: see https://github.com/typst/typst/issues/5512 for more information",
].join("\n");

function typst(args, expectedWarning = "") {
  const result = spawnSync("typst", args, { encoding: "utf8" });
  if (result.stdout) process.stdout.write(result.stdout);
  if (result.status !== 0) {
    if (result.stderr) process.stderr.write(result.stderr);
    process.exit(result.status ?? 1);
  }

  const warnings = result.stderr.trim().replaceAll("\r\n", "\n");
  if (warnings !== expectedWarning) {
    console.error(warnings || "Unexpectedly missing Typst warning output.");
    process.exit(1);
  }
}

const version = spawnSync("typst", ["--version"], { encoding: "utf8" });
if (version.status !== 0 || !version.stdout.trim().startsWith("typst 0.15.1")) {
  console.error(`CATS manuals require Typst 0.15.1; found: ${version.stdout.trim() || "unavailable"}`);
  process.exit(1);
}

mkdirSync("docs/generated", { recursive: true });

const common = ["--root", "docs", "--font-path", "docs/fonts", "--ignore-system-fonts"];
typst(["compile", "--creation-timestamp", "1786303398", ...common, "docs/Main.typ", "CATS User Manual.pdf"]);
typst(["compile", "--features", "html", ...common, "docs/Web.typ", "docs/generated/manual.html"], expectedHtmlWarning);
