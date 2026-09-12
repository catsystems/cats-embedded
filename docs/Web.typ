#import "styles.typ": *

#let manual-version = "2.1.2"
#let pdf-url = "https://raw.githubusercontent.com/catsystems/cats-embedded/main/CATS%20User%20Manual.pdf"
#let source-url = "https://github.com/catsystems/cats-embedded/tree/main/docs"
#let manual-css = ```css
:root {
  color-scheme: light dark;
  --accent: #f0870f;
  --bg: #ffffff;
  --surface: #f6f7f8;
  --text: #18202a;
  --muted: #5e6975;
  --border: #d9dde2;
  --note-bg: #eef7ff;
  --note-border: #1f4e78;
  --warning-bg: #fff0f0;
  --warning-border: #8a1c1c;
}
* { box-sizing: border-box; }
html { scroll-behavior: smooth; }
body {
  margin: 0;
  background: var(--bg);
  color: var(--text);
  font: 16px/1.65 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
}
a { color: #a95000; text-underline-offset: 0.16em; }
a:hover { color: var(--accent); }
.site-header {
  border-bottom: 3px solid var(--accent);
  background: #111820;
  color: #fff;
}
.header-inner {
  width: min(1180px, calc(100% - 2rem));
  margin: 0 auto;
  padding: 1.1rem 0;
  display: flex;
  gap: 1rem;
  align-items: center;
  justify-content: space-between;
  flex-wrap: wrap;
}
.brand { color: #fff; text-decoration: none; font-weight: 750; letter-spacing: .035em; }
.header-links { display: flex; gap: .9rem; flex-wrap: wrap; }
.header-links a { color: #fff; }
.manual-shell {
  width: min(1180px, calc(100% - 2rem));
  margin: 0 auto;
  display: grid;
  grid-template-columns: minmax(14rem, 18rem) minmax(0, 1fr);
  gap: clamp(2rem, 5vw, 5rem);
  align-items: start;
}
.contents { position: sticky; top: 1rem; max-height: calc(100vh - 2rem); overflow: auto; padding: 2rem 0; }
.contents h2 { margin: 0 0 .75rem; font-size: 1rem; }
.contents ol { margin: 0; padding-left: 1.35rem; }
.contents li { margin: .25rem 0; }
.contents a { color: var(--muted); text-decoration: none; }
.contents a:hover { color: var(--accent); }
.mobile-contents { display: none; margin: 1rem 0 0; border: 1px solid var(--border); border-radius: .5rem; padding: .75rem 1rem; }
.mobile-contents summary { cursor: pointer; font-weight: 700; }
.manual-article { min-width: 0; padding: clamp(2rem, 5vw, 5rem) 0 6rem; }
.manual-title { padding-bottom: 2rem; border-bottom: 1px solid var(--border); margin-bottom: 3rem; }
.manual-title h1 { font-size: clamp(2.2rem, 6vw, 4.2rem); line-height: 1.05; margin: 0 0 .6rem; }
.manual-title p { margin: .25rem 0; color: var(--muted); }
.manual-article h1, .manual-article h2, .manual-article h3 { line-height: 1.2; scroll-margin-top: 1rem; }
.manual-article h1 { margin-top: 4rem; font-size: clamp(1.8rem, 4vw, 2.6rem); }
.manual-article h2 { margin-top: 3rem; font-size: clamp(1.35rem, 3vw, 1.9rem); }
.manual-article h3 { margin-top: 2.2rem; font-size: 1.2rem; }
.manual-article img { display: block; max-width: 100%; height: auto; margin-inline: auto; }
.manual-figure, .manual-table, .manual-subfigure { margin: 2rem 0; max-width: 100%; }
.manual-figure > figcaption, .manual-table > figcaption, .manual-subfigure > figcaption { color: var(--muted); text-align: center; font-size: .92rem; margin-top: .65rem; }
.manual-table { overflow-x: auto; }
.manual-table table { min-width: max-content; width: 100%; border-collapse: collapse; }
table { display: block; max-width: 100%; overflow-x: auto; border-collapse: collapse; }
th, td { border: 1px solid var(--border); padding: .45rem .6rem; vertical-align: top; }
pre { max-width: 100%; overflow-x: auto; padding: 1rem; border-radius: .45rem; background: var(--surface); }
code { overflow-wrap: anywhere; }
math { overflow-x: auto; max-width: 100%; }
.web-footnote { color: var(--muted); font-size: .86em; }
.notice { margin: 1.3rem 0; padding: .9rem 1rem; border: 1px solid; border-left-width: .35rem; border-radius: .4rem; }
.notice.note { background: var(--note-bg); border-color: var(--note-border); }
.notice.warning { background: var(--warning-bg); border-color: var(--warning-border); }
.responsive-split { display: grid; grid-template-columns: minmax(0, 1.9fr) minmax(14rem, 1fr); gap: 2rem; align-items: start; }
.figure-stack { display: grid; gap: 1rem; }
.figure-stack .manual-subfigure { margin: 0; }
.glossary { display: grid; grid-template-columns: minmax(9rem, 14rem) minmax(0, 1fr); gap: .65rem 1.25rem; }
.glossary dt { font-weight: 700; }
.glossary dd { margin: 0; }
@media (prefers-color-scheme: dark) {
  :root {
    --bg: #12171d;
    --surface: #1b222a;
    --text: #e9edf1;
    --muted: #aab4bf;
    --border: #39434d;
    --note-bg: #122738;
    --note-border: #6da8d1;
    --warning-bg: #351a1a;
    --warning-border: #ef8d8d;
  }
  a { color: #ffb35e; }
}
@media (max-width: 800px) {
  .manual-shell { display: block; }
  .contents { display: none; }
  .mobile-contents { display: block; }
  .mobile-contents[open] { max-height: 65vh; overflow: auto; }
  .manual-article { padding-top: 2rem; }
  .responsive-split { grid-template-columns: 1fr; }
  .glossary { grid-template-columns: 1fr; gap: .15rem; }
  .glossary dd { margin-bottom: 1rem; }
}
```.text

#show pagebreak: it => none
#show v: it => none

#html.elem("html", attrs: (lang: "en"))[
  #html.elem("head")[
    #html.elem("meta", attrs: (charset: "utf-8"))
    #html.elem("meta", attrs: (name: "viewport", content: "width=device-width, initial-scale=1"))
    #html.elem("meta", attrs: (name: "description", content: "CATS User Manual for the Vega flight computer and Ground Station."))
    #html.elem("link", attrs: (rel: "canonical", href: "https://catsystems.io/manual"))
    #html.elem("title")[CATS User Manual]
    #html.style(manual-css)
  ]
  #html.elem("body")[
    #html.elem("header", attrs: (class: "site-header"))[
      #html.elem("div", attrs: (class: "header-inner"))[
        #html.elem("a", attrs: (class: "brand", href: "https://catsystems.io"))[CATS]
        #html.elem("nav", attrs: (class: "header-links", aria-label: "Manual resources"))[
          #html.elem("a", attrs: (href: pdf-url))[Download PDF]
          #html.elem("a", attrs: (href: source-url))[Source]
        ]
      ]
    ]
    #html.elem("div", attrs: (class: "manual-shell"))[
      #html.elem("nav", attrs: (class: "contents", aria-label: "Contents"))[
        #html.elem("h2")[Contents]
        #outline(title: none, depth: 3)
      ]
      #html.elem("main")[
        #html.elem("details", attrs: (class: "mobile-contents"))[
          #html.elem("summary")[Contents]
          #outline(title: none, depth: 3)
        ]
        #html.elem("article", attrs: (class: "manual-article"))[
          #html.elem("div", attrs: (class: "manual-title"))[
            #html.elem("h1")[CATS User Manual]
            #html.elem("p")[Version #manual-version]
            #html.elem("p")[CATS Vega flight computer and Ground Station]
          ]
          #include "Chapters/1_RevisionHistory.typ"
          #include "Chapters/Glossary.typ"
          #include "Chapters/1.5_Disclaimer.typ"
          #include "Chapters/1.6_RegulatoryInformation.typ"
          #include "Chapters/2_Introduction.typ"
          #include "Chapters/3_Vega.typ"
          #include "Chapters/4_GroundStation.typ"
          #include "Chapters/5_Example.typ"
          #include "Chapters/6_Testing.typ"
          #include "Chapters/7_AdvancedInformation.typ"
          #include "Chapters/8_FAQ.typ"
        ]
      ]
    ]
  ]
]
