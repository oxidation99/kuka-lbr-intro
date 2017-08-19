[![Build Status](https://travis-ci.com/nnadeau/kuka-lbr-intro.svg?token=FN9pU87Hzpg8Cn89F6sm&branch=master)](https://travis-ci.com/nnadeau/kuka-lbr-intro)
[![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)](https://raw.githubusercontent.com/nnadeau/kuka-lbr-intro/master/LICENSE)
[![GitHub release](https://img.shields.io/github/release/nnadeau/kuka-lbr-intro.svg)](https://github.com/nnadeau/kuka-lbr-intro/releases/latest)
[![Github All Releases](https://img.shields.io/github/downloads/nnadeau/kuka-lbr-intro/total.svg)](https://github.com/nnadeau/kuka-lbr-intro/releases/latest)

# kuka-lbr-intro
An introduction to programming the KUKA LBR iiwa robot 

## Presentation
This repository hosts an introductory presentation on collaborative robot programming with the KUKA LBR iiwa robot. Through continuous integration (CI), the [latest version of the presentation](https://github.com/nnadeau/kuka-lbr-intro/releases/latest) is built and deployed automatically.

## Development
- The presentation written in `TeX` using the `beamer` presentaion package.
- `Travis CI` builds `presentation.pdf` in an `Ubuntu Trusty` container, see [`.travis.yml`](.travis.yml) for details.
- Submodules:
  - `mtheme`: beamer presentation theme
  - `minted` & `fvextra`: code importing and wrapping
    - These packages are submoduled in order for `Travis CI` to use the latest version, as `TeX Live 2013/Debian` is old.
