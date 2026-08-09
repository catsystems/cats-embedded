# CATS User Manual

This directory contains the Typst sources and required assets for the CATS User Manual.

## Build

Install Typst 0.15.1, then run the following command from the `cats-embedded` repository root:

```powershell
typst compile --root docs --font-path docs/fonts --ignore-system-fonts docs/Main.typ "Cats User Manual.pdf"
```

`Main.typ` is the document entry point. Chapter sources are in `Chapters`, shared layout definitions are in `styles.typ`, and the locally vendored Roboto fonts are in `fonts`. Code snippets use Typst's embedded DejaVu Sans Mono, so the build does not depend on system fonts.
