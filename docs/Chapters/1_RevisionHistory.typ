#import "../styles.typ": *

= Revision History

#cats-table(
  table(
  columns: (0.2fr, 0.1fr, 0.7fr,),
  inset: (x: 4pt, y: 4.5pt),
  align: (x, y) => left + top,
  stroke: 0.35pt + luma(45%),
  fill: (x, y) => if calc.even(y) { luma(90%) } else { white },
  [Date],
  [Version],
  [Comment],
  [15 Dec 2022],
  [1.0.0],
  [Initial release],
  [05 Mar 2023],
  [1.1.0],
  [Addressing first round of feedback; added telemetry explanations],
  [16 Jul 2023],
  [2.0.0],
  [Added explanations for new hardware version; added support for sensor tab on the ground station; added explanations for test button; added bootloader explanations on ground station],
  [21 Jan 2026],
  [2.1.0],
  [Added regulatory info; Added more information on software updates and flight recorder settings; fixed typos],
  [21 Jun 2026],
  [2.1.1],
  [Added section on Ground Station USB Data Streaming],
  [09 Aug 2026],
  [2.1.2],
  [Corrected Ground Station flash memory capacity and CATS Vega dimensions],
  table.hline(y: 1, stroke: 0.5pt + black),
  table.hline(y: 2, stroke: 0.5pt + black),
  table.hline(y: 3, stroke: 0.5pt + black),
  table.hline(y: 4, stroke: 0.5pt + black),
  table.hline(y: 5, stroke: 0.5pt + black),
  table.hline(y: 6, stroke: 0.5pt + black),
  table.hline(y: 7, stroke: 0.5pt + black)
),
  caption: [Revision History],
  continued: false,
  breakable: false,
) <tab-RevHist>

#pagebreak()
