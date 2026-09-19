import { existsSync, readFileSync, readdirSync } from "node:fs";
import { resolve } from "node:path";

const htmlPath = "docs/generated/manual.html";
const html = readFileSync(htmlPath, "utf8");
const webSource = readFileSync("docs/Web.typ", "utf8");
const chapterFiles = readdirSync("docs/Chapters")
  .filter((name) => name.endsWith(".typ"))
  .map((name) => readFileSync(`docs/Chapters/${name}`, "utf8"))
  .join("\n");

function requireCondition(condition, message) {
  if (!condition) throw new Error(message);
}

requireCondition(html.includes('<link rel="canonical" href="https://catsystems.io/manual">'), "Missing manual canonical URL");
requireCondition(html.includes("Last updated: 18 September 2026"), "Missing last-updated date");
requireCondition(!html.includes("Version 2.1.2"), "Obsolete manual version remains");
requireCondition(!html.includes("Revision History"), "Obsolete revision history remains");
requireCondition(html.includes('<article class="manual-article">'), "Missing semantic manual article");
requireCondition(html.includes("<math"), "Equations were not exported as MathML");
requireCondition(!/<script\b/i.test(html), "Scripts are forbidden in the generated manual");
requireCondition(!webSource.includes("html.style"), "Manual styling must come from CATS Flights");
requireCondition(!/<form\b/i.test(html), "Forms are forbidden in the generated manual");
requireCondition(!/\son[a-z]+\s*=/i.test(html), "Event-handler attributes are forbidden in the generated manual");
requireCondition(!/data:image/i.test(html), "Images must not be embedded in the generated manual");
requireCondition(!/(?:src|href)="(?:\.\.?\/|images\/)/i.test(html), "Generated manual contains a local asset path");
requireCondition(!html.includes("CATS Vega flight computer and Ground Station"), "Obsolete manual subtitle remains");
requireCondition(!html.includes('class="site-header"'), "Generated manual contains a duplicate site header");
requireCondition(!html.includes('class="mobile-contents"'), "Generated manual contains duplicate responsive navigation");
requireCondition(!html.includes('class="web-footnote"'), "Generated manual contains redundant source footnotes");
requireCondition(!html.includes(".cfg"), "Obsolete .cfg flight-log format remains");
requireCondition(!/\b(?:Home|Event|Timer) (?:tab|screen)\b/i.test(html), "Obsolete Configurator navigation remains");
requireCondition(!/(?:Configuration|Events|Timers) tab\b/i.test(html), "Obsolete Configurator tab wording remains");
requireCondition(!html.includes("bring a shovel"), "Obsolete testing-mode joke remains");
requireCondition(!html.includes("only enables triggering events and not actions"), "Incorrect testing-mode behavior remains");
requireCondition(html.includes("Events &amp; Timers"), "Current Events & Timers workflow is missing");
requireCondition(html.includes("Preflight"), "Preflight workflow is missing");
requireCondition(html.includes("Profiles"), "Profiles workflow is missing");
requireCondition(html.includes(".cfl"), "Current .cfl flight-log format is missing");
requireCondition(html.includes("1 MB FAT data partition"), "Ground Station data-partition capacity is missing");
requireCondition(html.includes("Every action assigned to that event is executed"), "Testing-mode action warning is missing");
requireCondition(html.includes("Telemetry receiver firmware 1.2.0 is the first version"), "Radio-update compatibility boundary is missing");
requireCondition(html.includes('class="manual-table fsm-transitions"'), "FSM transition table hook is missing");
requireCondition(!html.includes("Overview of error beeping patterns"), "Unimplemented audible-error table remains");
requireCondition(!html.includes("7 to 25 volts"), "Obsolete Vega input-voltage guidance remains");
requireCondition(
  html.includes('<a href="https://github.com/catsystems/cats-configurator/releases/">Configurator releases page</a>'),
  "Configurator releases link is not attached to its main text",
);
requireCondition(
  !html.includes('>https://github.com/catsystems/cats-configurator/releases<'),
  "Configurator releases URL leaked into web body text",
);
requireCondition(html.includes('href="#sec-FirmwareUpdates">6</a>'), "Firmware Updates cross-reference number is stale");
requireCondition(html.includes('href="#sec-Testing">8</a>'), "Testing cross-reference number is stale");
requireCondition(html.includes('href="#sec-AdvancedInfo">9</a>'), "Advanced Information cross-reference number is stale");
for (const heading of ["Hopping Pattern", "Synchronization", "Calibration of Sensors", "Kalman Filter", "Gain Scheduling"]) {
  requireCondition(html.includes(`<h4>${heading}</h4>`), `Missing semantic technical heading: ${heading}`);
}

const ids = new Set([...html.matchAll(/\sid="([^"]+)"/g)].map((match) => match[1]));
const anchors = [...html.matchAll(/href="#([^"]+)"/g)].map((match) => match[1]);
const missingAnchors = [...new Set(anchors.filter((anchor) => !ids.has(anchor)))];
requireCondition(missingAnchors.length === 0, `Broken internal links: ${missingAnchors.join(", ")}`);
const sourceHeadingCount = (chapterFiles.match(/^=+\s+/gm) ?? []).length + (chapterFiles.match(/#heading\(level:/g) ?? []).length;
const articleHtml = html.slice(html.indexOf('<article class="manual-article">'), html.indexOf("</article>"));
requireCondition(!/href="#gls-/i.test(articleHtml), "Glossary terms must render as plain text");
const generatedHeadingCount = (articleHtml.match(/<h[1-6]\b/g) ?? []).length - 1;
requireCondition(generatedHeadingCount === sourceHeadingCount, `Heading mismatch: ${generatedHeadingCount}/${sourceHeadingCount}`);

const sourceLinks = [...chapterFiles.matchAll(/link\("(https?:\/\/[^"\s]+)"/g)].map((match) => match[1]);
for (const link of new Set(sourceLinks)) {
  requireCondition(html.includes(`href="${link.replaceAll("&", "&amp;")}"`), `Missing external link: ${link}`);
}

const imagePrefix = "https://raw.githubusercontent.com/catsystems/cats-embedded/docs/web-manual/docs/images/";
const generatedImages = [...html.matchAll(/<img\s+[^>]*src="([^"]+)"[^>]*>/g)].map((match) => {
  requireCondition(match[0].includes('loading="lazy"'), `Image is not lazy-loaded: ${match[1]}`);
  requireCondition(match[0].includes('decoding="async"'), `Image does not decode asynchronously: ${match[1]}`);
  requireCondition(match[1].startsWith(imagePrefix), `Unexpected image source: ${match[1]}`);
  return decodeURIComponent(match[1].slice(imagePrefix.length));
});
const sourceImages = [...chapterFiles.matchAll(/doc-image\("([^"]+)"/g)].map((match) => match[1]);
requireCondition(new Set(generatedImages).size === new Set(sourceImages).size, "Generated image set does not match the chapter sources");
for (const image of new Set(generatedImages)) {
  requireCondition(existsSync(resolve("docs/images", image)), `Missing source image: ${image}`);
  requireCondition(sourceImages.includes(image), `Unexpected generated image: ${image}`);
}

const figureCount = (html.match(/<figure\b/g) ?? []).length;
const captionCount = (html.match(/<figcaption\b/g) ?? []).length;
requireCondition(figureCount === captionCount, `Figure/caption mismatch: ${figureCount}/${captionCount}`);

console.log(`Manual HTML verified: ${ids.size} anchors, ${generatedImages.length} lazy images, ${figureCount} captioned figures.`);
