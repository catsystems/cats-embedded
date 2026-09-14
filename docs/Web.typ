#import "styles.typ": *

#show pagebreak: it => none
#show v: it => none

#html.elem("html", attrs: (lang: "en"))[
  #html.elem("head")[
    #html.elem("meta", attrs: (charset: "utf-8"))
    #html.elem("meta", attrs: (name: "viewport", content: "width=device-width, initial-scale=1"))
    #html.elem("meta", attrs: (name: "description", content: "CATS User Manual for the Vega flight computer and Ground Station."))
    #html.elem("link", attrs: (rel: "canonical", href: "https://catsystems.io/manual"))
    #html.elem("title")[CATS User Manual]
  ]
  #html.elem("body")[
    #html.elem("nav", attrs: (class: "contents", aria-label: "Contents"))[
      #html.elem("h2")[Contents]
      #outline(title: none, depth: 3)
    ]
    #html.elem("main")[
      #html.elem("article", attrs: (class: "manual-article"))[
        #html.elem("div", attrs: (class: "manual-title"))[
          #html.elem("h1")[CATS User Manual]
          #html.elem("p")[Last updated: #manual-last-updated]
        ]
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
