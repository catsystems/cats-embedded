import { existsSync, readFileSync, readdirSync } from "node:fs";
import { resolve } from "node:path";

const htmlPath = "docs/generated/manual.html";
const html = readFileSync(htmlPath, "utf8");
const chapterFiles = readdirSync("docs/Chapters")
  .filter((name) => name.endsWith(".typ"))
  .map((name) => readFileSync(`docs/Chapters/${name}`, "utf8"))
  .join("\n");

function requireCondition(condition, message) {
  if (!condition) throw new Error(message);
}

requireCondition(html.includes('<link rel="canonical" href="https://catsystems.io/manual">'), "Missing manual canonical URL");
requireCondition(html.includes("Version 2.1.2"), "Missing manual version");
requireCondition(html.includes('<article class="manual-article">'), "Missing semantic manual article");
requireCondition(html.includes("<math"), "Equations were not exported as MathML");
requireCondition(html.includes("CATS%20User%20Manual.pdf"), "Missing PDF download");
requireCondition(!/<script\b/i.test(html), "Scripts are forbidden in the generated manual");
requireCondition(!/<form\b/i.test(html), "Forms are forbidden in the generated manual");
requireCondition(!/\son[a-z]+\s*=/i.test(html), "Event-handler attributes are forbidden in the generated manual");
requireCondition(!/data:image/i.test(html), "Images must not be embedded in the generated manual");
requireCondition(!/(?:src|href)="(?:\.\.?\/|images\/)/i.test(html), "Generated manual contains a local asset path");

const ids = new Set([...html.matchAll(/\sid="([^"]+)"/g)].map((match) => match[1]));
const anchors = [...html.matchAll(/href="#([^"]+)"/g)].map((match) => match[1]);
const missingAnchors = [...new Set(anchors.filter((anchor) => !ids.has(anchor)))];
requireCondition(missingAnchors.length === 0, `Broken internal links: ${missingAnchors.join(", ")}`);
const sourceHeadingCount = (chapterFiles.match(/^=+\s+/gm) ?? []).length + (chapterFiles.match(/#heading\(level:/g) ?? []).length;
const articleHtml = html.slice(html.indexOf('<article class="manual-article">'), html.indexOf("</article>"));
const generatedHeadingCount = (articleHtml.match(/<h[1-6]\b/g) ?? []).length - 1;
requireCondition(generatedHeadingCount === sourceHeadingCount, `Heading mismatch: ${generatedHeadingCount}/${sourceHeadingCount}`);

const sourceLinks = [...chapterFiles.matchAll(/(?:link|source-note)\("(https?:\/\/[^"\s]+)"/g)].map((match) => match[1]);
for (const link of new Set(sourceLinks)) {
  requireCondition(html.includes(`href="${link.replaceAll("&", "&amp;")}"`), `Missing external link: ${link}`);
}

const imagePrefix = "https://raw.githubusercontent.com/catsystems/cats-embedded/main/docs/images/";
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
