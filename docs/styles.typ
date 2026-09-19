// Shared layout and semantic helpers for the CATS User Manual.
#let manual-last-updated = "18 September 2026"
#let cats-orange = rgb("#f0870f")
#let light-blue = rgb("#ddebf7")
#let dark-blue = rgb("#1f4e78")
#let light-red = rgb("#ffabab")
#let dark-red = rgb("#800000")
#let web-image-root = "https://raw.githubusercontent.com/catsystems/cats-embedded/main/docs/images/"

#let doc-image(path, width: auto, alt: "", outline: false) = context {
  if target() == "html" {
    let rendered = html.img(
      src: web-image-root + path.replace(" ", "%20"),
      alt: alt,
      loading: "lazy",
      decoding: "async",
    )
    if outline {
      html.elem(
        "span",
        attrs: (
          class: "manual-screen-outline",
          style: "display:inline-block;line-height:0;border:1px solid color-mix(in srgb,currentColor 22%,transparent);border-radius:2px;overflow:hidden",
        ),
        rendered,
      )
    } else {
      rendered
    }
  } else {
    if outline {
      block(
        width: width,
        inset: 0pt,
        radius: 1pt,
        stroke: 0.5pt + luma(65%),
        clip: true,
      )[
        #image("images/" + path, width: 100%)
      ]
    } else {
      image("images/" + path, width: width)
    }
  }
}

#let source-note(url, label-name: none) = context {
  if target() == "html" {
    if label-name == none { metadata(none) } else { [#metadata(none) #label(label-name)] }
  } else {
    footnote[
      #if label-name != none { [#metadata(none) #label(label-name)] }
      #link(url)[#url]
    ]
  }
}

#let reference-numbers = (
  "sec:VegaHW": "4.1",
  "sec:FSM": "4.2.1",
  "sec:DescriptionOfConfigurator": "4.3.2",
  "sec:GeneratePlots": "4.3.7",
  "sec:softwareupdates": "4.3.8",
  "sec:BeepingPatterns": "4.4",
  "sec:explanMenus": "5.2.1",
  "sec:telemetrymode": "5.2.2",
  "sec:data_streaming": "5.2.3",
  "sec:gs_updates": "5.2.6",
  "sec:FirmwareUpdates": "6",
  "sec:Examples": "7",
  "sec:Testing": "8",
  "sec:AdvancedInfo": "9",
  "sec:EstAlg": "9.3",
  "sec:GeneratePlotsPython": "9.4",
  "sec:CLI": "9.5",
  "sec:FAQ": "10",
  "tab:Specs": "1",
  "tab:FSMTransitions": "2",
  "tab:ActionTable": "3",
  "tab:ConfiguratorNavigation": "4",
  "tab:HowToAct": "5",
  "tab:BeepingPatternsStates": "6",
  "tab:BeepingPatternsErrors": "7",
  "tab:GSSpecs": "8",
  "tab:GSSettings": "9",
  "tab:ExampleSimple": "10",
  "tab:ExampleAdvanced": "11",
  "tab:CLICommands": "12",
  "tab:CLICommandsSetGet": "13",
  "fig:VegaHWSpecs": "2",
  "fig:FSM": "3",
  "fig:RadioUpdateSequence": "19",
  "fig:HWsimpleExample": "20",
  "fig:HWadvancedExample": "21",
  "fig:SoftwareOverview": "24",
  "fig:fhss": "25",
)

#let glossary-names = (
  "apogee": "Apogee",
  "barometer": "Barometer",
  "Calibrating": "Calibrating",
  "CATS": "CATS",
  "CLI": "CLI",
  "Coasting": "Coasting",
  "crc": "CRC",
  "DFU": "DFU",
  "drogue chute": "Drogue Chute",
  "fhss": "FHSS",
  "FreeRTOS": "FreeRTOS",
  "FSM": "FSM",
  "GNSS": "GNSS",
  "I/O": "I/O",
  "IMU": "IMU",
  "Kalman Filter": "Kalman Filter",
  "liftoff": "Liftoff",
  "main chute": "Main Chute",
  "patch antenna": "Patch Antenna",
  "Power Supply": "Power Supply",
  "PWM": "PWM",
  "pyro": "Pyro",
  "quaternion": "Quaternion",
  "Ready": "Ready",
  "RF": "RF",
  "servo": "Servo",
  "Thrusting": "Thrusting",
  "touchdown": "Touchdown",
  "UART": "UART",
)

#let glossary-labels = (
  "apogee": <gls-apogee>,
  "barometer": <gls-barometer>,
  "Calibrating": <gls-Calibrating>,
  "CATS": <gls-CATS>,
  "CLI": <gls-CLI>,
  "Coasting": <gls-Coasting>,
  "crc": <gls-crc>,
  "DFU": <gls-DFU>,
  "drogue chute": <gls-drogue-chute>,
  "fhss": <gls-fhss>,
  "FreeRTOS": <gls-FreeRTOS>,
  "FSM": <gls-FSM>,
  "GNSS": <gls-GNSS>,
  "I/O": <gls-I-O>,
  "IMU": <gls-IMU>,
  "Kalman Filter": <gls-Kalman-Filter>,
  "liftoff": <gls-liftoff>,
  "main chute": <gls-main-chute>,
  "patch antenna": <gls-patch-antenna>,
  "Power Supply": <gls-Power-Supply>,
  "PWM": <gls-PWM>,
  "pyro": <gls-pyro>,
  "quaternion": <gls-quaternion>,
  "Ready": <gls-Ready>,
  "RF": <gls-RF>,
  "servo": <gls-servo>,
  "Thrusting": <gls-Thrusting>,
  "touchdown": <gls-touchdown>,
  "UART": <gls-UART>,
)

#let xref(key) = link(label(key.replace(":", "-").replace("/", "-")))[
  #reference-numbers.at(key, default: key)
]

#let gls(key, cap: false) = {
  let value = glossary-names.at(key, default: [#key])
  metadata(("glossary-use", key))
  if cap { upper(value.slice(0, 1)) + value.slice(1) } else { value }
}

#let glossary-pages(key) = context {
  if target() == "html" { return none }
  let uses = query(metadata).filter(item => item.value == ("glossary-use", key))
  let pages = uses.map(item => (counter(page).at(item.location()).first(), item.location()))
  let unique = pages.fold((), (acc, pair) => {
    if acc.any(existing => existing.first() == pair.first()) { acc } else { acc + (pair,) }
  })
  if unique.len() > 0 {
    h(0.35em)
    unique.enumerate().map(pair => {
      if pair.first() > 0 { text(", ") }
      str(pair.last().first())
    }).join()
  }
}

#let glossary-table(entries, row-gap: 12pt) = context {
  let visible = entries.filter(entry =>
    query(metadata).any(item => item.value == ("glossary-use", entry.at(0)))
  )
  let cells = visible.map(entry => (
    strong(box(entry.at(1))),
    [#(entry.at(2)).#glossary-pages(entry.at(0))],
  )).flatten()
  if target() == "html" {
    html.elem("dl", attrs: (class: "glossary"))[
      #for entry in visible {
        html.elem("dt", entry.at(1))
        html.elem("dd", entry.at(2))
      }
    ]
  } else {
    table(
      columns: (2.7cm, 1fr),
      column-gutter: 6pt,
      row-gutter: row-gap,
      inset: 0pt,
      stroke: none,
      align: (left + top, left + top),
      ..cells,
    )
  }
}

#let note(body) = context {
  if target() == "html" {
    html.elem("aside", attrs: (class: "notice note"), body)
  } else {
    block(
      width: 100%,
      inset: 10pt,
      radius: 6pt,
      fill: light-blue,
      stroke: 1.5pt + dark-blue,
      above: 8pt,
      below: 8pt,
      body,
    )
  }
}

#let warning(body) = context {
  if target() == "html" {
    html.elem("aside", attrs: (class: "notice warning"), body)
  } else {
    block(
      width: 100%,
      inset: 10pt,
      radius: 6pt,
      fill: light-red,
      stroke: 1.5pt + dark-red,
      above: 8pt,
      below: 8pt,
      body,
    )
  }
}

#let figure-counter = counter("cats-figure")
#let table-counter = counter("cats-table")

#let cats-figure(body, caption: none, continued: false, breakable: false) = {
  if not continued { figure-counter.step() }
  context {
    if target() == "html" {
      html.elem("figure", attrs: (class: "manual-figure"))[
        #body
        #if caption != none {
          html.elem("figcaption")[Figure #figure-counter.display(): #caption]
        }
      ]
    } else {
      block(width: 100%, breakable: breakable, above: 8pt, below: 8pt)[
        #align(center, body)
        #if caption != none {
          v(5pt)
          align(center)[#text(size: 8pt)[Figure #context figure-counter.display(): #caption]]
        }
      ]
    }
  }
}

#let cats-table(body, caption: none, continued: false, breakable: false, class-name: none) = {
  if not continued { table-counter.step() }
  context {
    if target() == "html" {
      let classes = if class-name == none { "manual-table" } else { "manual-table " + class-name }
      html.elem("figure", attrs: (class: classes))[
        #body
        #if caption != none {
          html.elem("figcaption")[Table #table-counter.display(): #caption]
        }
      ]
    } else {
      block(width: 100%, breakable: breakable, above: 7pt, below: 7pt)[
        #set par(justify: false, leading: 0.5em, spacing: 0.8em)
        #set text(hyphenate: false)
        #body
        #if caption != none {
          v(4pt)
          align(center)[#text(size: 8pt)[Table #context table-counter.display(): #caption]]
        }
      ]
    }
  }
}

#let subfigure(body, caption, letter, width: 100%, label-name: none) = context {
  let result = if target() == "html" {
    html.elem("figure", attrs: (class: "manual-subfigure"))[
      #body
      #html.elem("figcaption")[(#letter) #caption]
    ]
  } else {
    align(center)[#block(width: width)[
      #align(center, body)
      #v(3pt)
      #align(center)[#text(size: 8pt)[(#letter) #caption]]
    ]]
  }
  if label-name == none {
    result
  } else {
    [#result #label(label-name)]
  }
}

#let responsive-split(left, right, columns: (63%, 1fr, 33%)) = context {
  if target() == "html" {
    html.elem("div", attrs: (class: "responsive-split"))[
      #html.elem("div")[#left]
      #html.elem("div")[#right]
    ]
  } else {
    grid(columns: columns, left, [], right)
  }
}

#let figure-stack(spacing: 8pt, breakable: false, ..children) = context {
  let items = children.pos()
  if target() == "html" {
    html.elem("div", attrs: (class: "figure-stack"))[
      #for item in items { html.elem("div", item) }
    ]
  } else if breakable {
    block(width: 100%, breakable: true)[
      #for (index, item) in items.enumerate() {
        if index > 0 { v(spacing) }
        block(width: 100%, breakable: false, item)
      }
    ]
  } else {
    stack(dir: ttb, spacing: spacing, ..items)
  }
}

#let normal-header = context {
  set par(spacing: 1.65em)
  grid(
    columns: (1fr, auto),
    align: (left + bottom, right + horizon),
    move(dy: 16pt, text(size: 10pt)[Control And Telemetry Systems]),
    image("images/logo_without_smile.png", width: 1.05cm),
  )
  v(3.8pt)
  move(dy: 1.8pt, line(length: 100%, stroke: 0.45pt + luma(35%)))
}

#let normal-footer = context align(center)[
  #move(dy: -5.2pt, text(size: 10pt)[#counter(page).display()])
]

#let normal-layout(body) = {
  set page(
    paper: "a4",
    margin: (left: 3cm, right: 3cm, top: 3cm, bottom: 3cm),
    header: normal-header,
    footer: normal-footer,
  )
  set text(font: "Roboto", weight: "light", size: 10pt, fill: black)
  set par(justify: true, leading: 0.49em, spacing: 1.65em)
  set heading(numbering: "1.1", outlined: true)
  set list(indent: 1.1em, body-indent: 0.55em, spacing: 0.35em)
  show heading.where(level: 1): it => block(above: 8pt, below: 15pt)[
    #text(size: 14.4pt, weight: "bold")[#it]
  ]
  show heading.where(level: 2): it => block(above: 24pt, below: 11pt)[
    #text(size: 12pt, weight: "bold")[#it]
  ]
  show heading.where(level: 3): it => block(above: 24pt, below: 10pt)[
    #text(size: 10pt, weight: "bold")[#it]
  ]
  show link: set text(fill: black)
  body
}
