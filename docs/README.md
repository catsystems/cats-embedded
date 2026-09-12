# CATS User Manual

This directory contains the Typst sources and required assets for the CATS User Manual.

## Build

Install Typst 0.15.1, then run the following command from the `cats-embedded` repository root:

```powershell
node docs/build-manual.mjs
```

`Main.typ` is the PDF entry point and `Web.typ` is the single-page HTML entry point. Both import the same chapter sources. Shared, target-aware presentation helpers are in `styles.typ`, and the locally vendored Roboto fonts are in `fonts`. Code snippets use Typst's embedded DejaVu Sans Mono, so the build does not depend on system fonts.

The generated outputs are `CATS User Manual.pdf` and `docs/generated/manual.html`. The HTML document loads its images lazily from the matching paths on `cats-embedded/main`, allowing `https://catsystems.io/manual` to follow newly merged documentation without rebuilding CATS Flights.

Run `node docs/check-manual.mjs` after building to validate internal links, external image paths, semantic figures, MathML, and the no-script policy. CI rebuilds both outputs with Typst 0.15.1 and rejects stale generated files or any warning other than Typst's explicit experimental-HTML notice.
