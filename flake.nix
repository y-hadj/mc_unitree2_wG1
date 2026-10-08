{
  description = "mc-unitree2 and test controller configuration";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    flake-parts.inputs.nixpkgs.follows = "mc-rtc-nix/nixpkgs";
    systems.follows = "mc-rtc-nix/systems";

    ccache-trigger.url = "github:boolean-option/true";

    # You can override dependencies from a commit/pull request by:
    # Adding it as input
    # your-repository.url = "github:username/repository/pull/ID/head";
    # your-repository.flake = true; # use false if the repository does not have a flake
    g1-description.url = "github:isri-aist/g1_description/pull/2/head";
    g1-description.flake = false;

    mc-g1.url = "github:isri-aist/mc_g1/pull/2/head";
    mc-g1.flake = false;

    unitree-sdk2.url = "github:y-hadj/unitree_sdk2";
    unitree-sdk2.flake = false;

    mc-external-forces-observer.url = "github:y-hadj/mc_external_forces_observer";
    mc-external-forces-observer.flake = false;
  };

  outputs =
    inputs:
    /**
      This will:
      - Build the controller from the local sources in this repository
      - Generate two mc-rtc-superbuild development shells:
        - mc-rtc-superbuild-controller-name : a release shell allowing to run the controller
        - mc-rtc-superbuild-controller-name-devel : a development shell allowing to build the controller and run it
        Instructions are displayed upon entering the shells.

       You can control the behaviour or define your own shells with:
       mc-rtc-nix = {};
       mc-rtc-superbuild = {};

       Note that the mc-rtc-superbuild attribute set will be merged with the default controller configuration
    */
    inputs.mc-rtc-nix.lib.mkMcRtcModule inputs (
      { lib, ... }:
      {
        mc-rtc-nix.overlays.ccache = inputs.ccache-trigger.value;

        mc-rtc-superbuild =
          { pkgs, ... }:
          {
            enable = true;
            project.pname = "";
            configurations = {
              mc-rtc-superbuild-g1 = {
                extends = [ "minimal" ];
                runtime = {
                  apps = [
                    pkgs.mc-unitree2
                    pkgs.mc-rtc-magnum # XXX this should be merged automatically from "minimal"
                  ];
                  robots = [
                    pkgs.mc-g1
                    pkgs.mc-revo2
                  ];
                  observers = [
                    pkgs.mc-external-forces-observer
                  ];
                  extraConfigFiles = [ "${pkgs.mc-unitree2}/etc/mc_unitree/mc_rtc_example.yaml" ];
                };
              };
            };
          };

        flakoboros = {
          packages = {
            mc-unitree2 =
              {
                stdenv,
                lib,
                fetchFromGitHub,
                cmake,
                mc-rtc,
                unitree-sdk2,
                unitree-sdk2-examples,
                cli11,
                libfort,
              }:

              stdenv.mkDerivation {
                pname = "mc-unitree2";
                version = "1.0.0";

                # main
                src = fetchFromGitHub {
                  owner = "isri-aist";
                  repo = "mc_unitree2";
                  rev = "1b22c4df1cd533b895f5751b09d7306537c36e1d";
                  hash = "sha256-5uX6Zjq1/bvZlrQm6OiZFl1PxNq6AT2ZduchRUxmvcU=";
                };

                nativeBuildInputs = [
                  cmake
                  cli11
                  libfort
                  unitree-sdk2-examples
                ];
                propagatedBuildInputs = [
                  mc-rtc
                  unitree-sdk2
                ];

                cmakeFlags = [
                  (lib.cmakeBool "GENERATE_G1_CONTROLLER" true)
                  "-DUNITREE_SDK2_SRC_DIR=${unitree-sdk2-examples}/share/unitree_sdk2"
                ];

                doCheck = true;

                meta = with lib; {
                  mainProgram = "MCControlG1";
                  description = "Interface between Unitree G1/g1 and mc_rtc";
                  homepage = "https://github.com/isri-aist/mc_unitree2";
                  license = licenses.bsd2;
                  platforms = platforms.all;
                };
              };

            unitree-sdk2-examples =
              { unitree-sdk2, stdenv }:

              stdenv.mkDerivation rec {
                pname = "unitree_sdk2-example";
                inherit (unitree-sdk2) src version;

                installPhase = ''
                  mkdir -p $out/share/unitree_sdk2
                  cp -r example $out/share/unitree_sdk2/
                '';

                meta = unitree-sdk2.meta // {
                  description = "Unitree SDK2 example folder only";
                };
              };
            mc-external-forces-observer =
              {
                stdenv,
                lib,
                cmake,
                mc-rtc,
              }:

              stdenv.mkDerivation {
                pname = "mc-external-forces-observer-yhadj";
                version = "0.0.0";

                # main
                src = inputs.mc-external-forces-observer;
                nativeBuildInputs = [
                  cmake
                ];
                propagatedBuildInputs = [
                  mc-rtc
                ];

                cmakeFlags = [ ];
                doCheck = true;

                meta = with lib; {
                  mainProgram = "mc-external-forces-observer";
                  description = "State observer for external forces based on torque measurements";
                  homepage = "https://github.com/isri-aist/mc_external_forces_observer";
                  license = licenses.bsd2;
                  platforms = platforms.all;
                };
              };
            libfort =
              {
                stdenv,
                lib,
                fetchFromGitHub,
                cmake,
              }:

              stdenv.mkDerivation {
                pname = "libfort";
                version = "0.5.1";

                src = fetchFromGitHub {
                  owner = "seleznevae";
                  repo = "libfort";
                  tag = "v0.5.1";
                  hash = "sha256-UHDApOTrPNb3e5qoWqsTcx16/rV0nO/Zn4DNH9bEJY0=";
                };

                nativeBuildInputs = [ cmake ];
                doCheck = false; # tests fail

                patches = [ ./patches/fix-libfort-pc.patch ];
                meta = with lib; {
                  description = "C/C++ library to create formatted ASCII tables for console applications";
                  homepage = "https://github.com/seleznevae/libfort";
                  license = licenses.mit;
                  platforms = platforms.all;
                };
              };
            mc-revo2 =
              {
                stdenv,
                lib,
                fetchFromGitHub,
                cmake,
                mc-rtc,
                revo2-description,
              }:

              let

                revo2-description' = revo2-description.override {
                  with-ros = mc-rtc.with-ros;
                };

              in

              stdenv.mkDerivation {
                pname = "mc-revo2";
                version = "1.0.0";

                src = fetchFromGitHub {
                  owner = "isri-aist";
                  repo = "mc_revo2";
                  rev = "d654763f64f329d42707221f24981111fa2abb01";
                  hash = "sha256-T3ccoyWOhNzEtchV2fLAzNRQMgx/M85laFq5DaOVNfc=";
                };
                nativeBuildInputs = [ cmake ];
                propagatedBuildInputs = [
                  revo2-description'
                  mc-rtc
                ];

                cmakeFlags = [
                  "-DBUILD_TESTING=OFF"
                ];

                passthru = {
                  # TODO
                  # mujocoRobots = [ "revo2-mj-description" ];
                };

                doCheck = false;

                meta = with lib; {
                  description = "revo2 RobotModule for mc-rtc";
                  homepage = "https://github.com/isri-aist/mc_revo2";
                  license = licenses.bsd2;
                  platforms = platforms.all;
                };
              };

            revo2-description =
              {
                stdenv,
                lib,
                fetchFromGitHub,
                cmake,
                with-ros ? false,
                ament-cmake,
                buildRosPackage,
              }:

              (if with-ros then buildRosPackage else stdenv.mkDerivation) {
                pname = "revo2-description";
                version = "1.0.0";
                separateDebugInfo = false;

                src = fetchFromGitHub {
                  owner = "isri-aist";
                  repo = "revo2_description";
                  rev = "7b8d7cea3f886f93ae98344766988ac0720125b9";
                  hash = "sha256-Ui6E6gzYdAutNS6tn+T8UkspkmyTqfFNpzL1s3fVIXA=";
                };

                buildType = "ament_cmake";
                nativeBuildInputs = if with-ros then [ ament-cmake ] else [ cmake ];
                propagatedBuildInputs = [ ];

                preConfigure = ''
                  export ROS_VERSION=2
                '';

                cmakeFlags = lib.optional (!with-ros) "-DDISABLE_ROS=ON" ++ [
                  "-DBUILD_TESTING=OFF"
                ];

                doCheck = false;

                meta = with lib; {
                  description = "revo2 urdf and data";
                  homepage = "https://github.com/isri-aist/revo2_description";
                  license = licenses.bsd2;
                  platforms = platforms.all;
                };
              };
            # };
          };
          overrideAttrs.mc-unitree2 = {
            src = lib.cleanSource ./.;
          };
          overrideAttrs.unitree-sdk2 = {
            src = inputs.unitree-sdk2;
          };
          overrideAttrs.mc-g1 = {
            src = inputs.mc-g1;
          };
          overrideAttrs.g1-description = {
            src = inputs.g1-description;
          };
        };
      }
    );
}
