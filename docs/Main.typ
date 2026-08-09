#import "styles.typ": *

// The supplied cover is reused unchanged and rotated onto a portrait A4 page.
#set page(paper: "a4", margin: 0pt, header: none, footer: none)
#align(center + horizon)[
  #rotate(-90deg, reflow: false)[
    #image("images/CATS_Cover_Page.png", width: 297mm)
  ]
]
#pagebreak()

#show: normal-layout

#show outline.entry.where(level: 1): it => block(above: 15pt, below: 2pt)[#strong(it)]
#show outline.entry.where(level: 2): it => block(below: 1.2pt)[#it]
#show outline.entry.where(level: 3): it => block(below: 0.8pt)[#it]
#block[
  #set text(size: 10pt)
  #outline(title: [Contents], depth: 3, indent: auto)
]
#pagebreak()

#include "Chapters/1_RevisionHistory.typ"
#include "Chapters/Glossary.typ"
#pagebreak()

#include "Chapters/1.5_Disclaimer.typ"
#include "Chapters/1.6_RegulatoryInformation.typ"
#include "Chapters/2_Introduction.typ"
#include "Chapters/3_Vega.typ"
#include "Chapters/4_GroundStation.typ"
#include "Chapters/5_Example.typ"
#include "Chapters/6_Testing.typ"
#include "Chapters/7_AdvancedInformation.typ"
#include "Chapters/8_FAQ.typ"
