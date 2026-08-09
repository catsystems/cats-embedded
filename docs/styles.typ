// Shared layout and semantic helpers for the CATS User Manual.
#let cats-orange = rgb("#f0870f")
#let light-blue = rgb("#ddebf7")
#let dark-blue = rgb("#1f4e78")
#let light-red = rgb("#ffabab")
#let dark-red = rgb("#800000")

#let reference-numbers = (
  "fn:note1": "1",
  "sec:VegaHW": "5.1",
  "sec:FSM": "5.2.1",
  "sec:DescriptionOfConfigurator": "5.3.2",
  "sec:GeneratePlots": "5.3.7",
  "sec:softwareupdates": "5.3.8",
  "sec:BeepingPatterns": "5.4",
  "sec:explanMenus": "6.2.1",
  "sec:telemetrymode": "6.2.2",
  "sec:data_streaming": "6.2.3",
  "sec:gs_updates": "6.2.6",
  "sec:Examples": "7",
  "sec:Testing": "8",
  "sec:AdvancedInfo": "9",
  "sec:EstAlg": "9.3",
  "sec:GeneratePlotsPython": "9.4",
  "sec:CLI": "9.5",
  "sec:FAQ": "10",
  "tab:RevHist": "1",
  "tab:Specs": "2",
  "tab:FSMTransitions": "3",
  "tab:ActionTable": "4",
  "tab:HomeTabOverview": "5",
  "tab:ConfigurationTabOverview": "6",
  "tab:HowToAct": "7",
  "tab:BeepingPatternsStates": "8",
  "tab:BeepingPatternsErrors": "9",
  "tab:GSSpecs": "10",
  "tab:CLICommands": "11",
  "tab:CLICommandsSetGet": "12",
  "fig:VegaHWSpecs": "2",
  "fig:FSM": "3",
  "fig:GUIHome": "4",
  "fig:GUIConfig": "5",
  "fig:GUIEvents": "6",
  "fig:GUIEventSel": "7",
  "fig:GUITimers": "8",
  "fig:GUICLI": "9",
  "fig:SWUpdateInit": "12a",
  "fig:SWUpdateUSB": "12b",
  "fig:SWUpdateConnected": "12c",
  "fig:SWUpdateProgram": "12d",
  "fig:SWUpdateProgramFinished": "12e",
  "fig:SWUpdate": "12",
  "fig:HWsimpleExample": "18",
  "fig:ExampleSimpleConfig": "19a",
  "fig:ExampleSimpleEvents": "19b",
  "fig:ExampleSimpleTimers": "19c",
  "fig:ExampleSimple": "19",
  "fig:HWadvancedExample": "20",
  "fig:ExampleAdvancedConfig": "21a",
  "fig:ExampleAdvancedEvents": "21b",
  "fig:ExampleAdvancedTimers": "21c",
  "fig:ExampleAdvanced": "21",
  "fig:GSTestingMainMenu": "22",
  "fig:GSTestingArming": "23",
  "fig:GSTestingWaitingArming": "24",
  "fig:GSTestingEventMenu": "25",
  "fig:GSTestingEventTriggering": "26",
  "fig:SoftwareOverview": "27",
  "fig:fhss": "28",
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
  link(glossary-labels.at(key))[#if cap { upper(value.slice(0, 1)) + value.slice(1) } else { value }]
}

#let glossary-pages(key) = context {
  let uses = query(metadata).filter(item => item.value == ("glossary-use", key))
  let pages = uses.map(item => (counter(page).at(item.location()).first(), item.location()))
  let unique = pages.fold((), (acc, pair) => {
    if acc.any(existing => existing.first() == pair.first()) { acc } else { acc + (pair,) }
  })
  if unique.len() > 0 {
    h(0.35em)
    unique.enumerate().map(pair => {
      if pair.first() > 0 { text(", ") }
      link(pair.last().last())[#str(pair.last().first())]
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

#let note(body) = block(
  width: 100%,
  inset: 10pt,
  radius: 6pt,
  fill: light-blue,
  stroke: 1.5pt + dark-blue,
  above: 8pt,
  below: 8pt,
  body,
)

#let warning(body) = block(
  width: 100%,
  inset: 10pt,
  radius: 6pt,
  fill: light-red,
  stroke: 1.5pt + dark-red,
  above: 8pt,
  below: 8pt,
  body,
)

#let figure-counter = counter("cats-figure")
#let table-counter = counter("cats-table")

#let cats-figure(body, caption: none, continued: false) = {
  if not continued { figure-counter.step() }
  block(width: 100%, breakable: false, above: 8pt, below: 8pt)[
    #align(center, body)
    #if caption != none {
      v(5pt)
      align(center)[#text(size: 8pt)[Figure #context figure-counter.display(): #caption]]
    }
  ]
}

#let cats-table(body, caption: none, continued: false, breakable: false) = {
  if not continued { table-counter.step() }
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

#let subfigure(body, caption, letter) = block(width: 100%)[
  #align(center, body)
  #v(3pt)
  #align(center)[#text(size: 8pt)[(#letter) #caption]]
]

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
