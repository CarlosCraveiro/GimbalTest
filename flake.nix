{
  description = "Dev environment for the Gimbal Project - Bootcamp on Autonomous Flight 2024";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.05";  # Specify the Nixpkgs version
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
    let
      pkgs = import nixpkgs {
        inherit system;
      };
      pythonEnv = pkgs.python310.withPackages (ps: [
        ps.pyserial
        ps.matplotlib
        # Add other Python packages here as needed
      ]);
    in
    {
      devShells.default = pkgs.mkShell {
        buildInputs = [ pythonEnv ];
      };

      apps.default = flake-utils.lib.mkApp {
        drv = pkgs.writeShellScriptBin "control-groundstation" ''
          exec ${pythonEnv.interpreter} ${./control_groundstation.py} --port /dev/ttyACM0 --baud 57600
        '';
      };
    });
}
