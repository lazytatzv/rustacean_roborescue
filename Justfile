set shell := ["bash", "-c"]


default: nix

nix:
  nix develop --impure

