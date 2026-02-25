{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  outputs = { self, nixpkgs, rust-overlay }:
    let 
      system = "x86_64-linux";

      pkgs = import nixpkgs {
        inherit system;
        overlays = [ (import rust-overlay) ];
      };

      myRust = pkgs.rust-bin.stable.latest.minimal.override {
        extensions = [
          "rust-src"
          "rust-analyzer"
          "clippy"
          "rustfmt"
        ];
        targets = [ "thumbv7em-none-eabihf" ]; # Cortex-M7 (STM32F767ZI)
      };

    in
    {
      devShells.${system}.default = pkgs.mkShell {
        buildInputs = [
          myRust
          pkgs.probe-rs-tools
          pkgs.flip-link
        ];
      }; 
    };
}
