set shell := ["bash", "-c"]


default: nix

nix:
  nix develop --impure

sync:
  git submodule update --init --recursive
