{
  description = "Ferram Aerospace Research";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };
  outputs = { nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
      in {
        devShell = pkgs.mkShell {
          buildInputs = [
            # May be needed by your editor
            # pkgs.dotnet-sdk
            pkgs.mono6
            pkgs.msbuild
            pkgs.python3
          ];
        };
      }
    );
}