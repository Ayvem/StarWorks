#!/usr/bin/env python3
# =============================================================================
# Tools/solar_scale/check_scale.py — is the solar system the right size?
#
# The question this answers is the one a player asks by looking: "the Sun from
# Mars should be two thirds the size it is from here — is it?" That is an
# ANGULAR question, and an angular size is two numbers (a radius and a
# distance) either of which can be wrong on its own without anything looking
# obviously broken.
#
# So this reads the numbers OUT OF THE SHIPPED SOURCE — Game/Source/
# GameInternal.hpp for the planets, the MoonDef table in GameScene.cpp for the
# moons — rather than out of a copy of them. A checker that carries its own
# copy of the data checks nothing but the person who typed it twice.
#
# The reference column is NASA's planetary fact sheets (volumetric mean radii,
# semi-major axes) and the IAU 2015 nominal solar radius. Where the game and
# the reference disagree the tolerance is 0.5%, which is inside the spread
# between "equatorial", "polar" and "volumetric mean" for the oblate bodies
# and therefore the point at which the disagreement stops being a mistake and
# starts being a choice of definition.
#
#     python3 Tools/solar_scale/check_scale.py
#
# Exit code 0 = every body is the right size and the right distance away.
# =============================================================================
import math
import os
import re
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

# ---- the reference -----------------------------------------------------------
# radius: volumetric mean radius, metres (NASA planetary fact sheets, 2024).
# sma:    semi-major axis about the primary, metres.
# The Sun's radius is the IAU 2015 Resolution B3 nominal value, 6.957e8.
REFERENCE = {
    "SOL":       (6.957e8,   None),
    "MERCURY":   (2.4397e6,  5.7909e10),
    "VENUS":     (6.0518e6,  1.0821e11),
    "TERRA":     (6.371e6,   1.496e11),
    "LUNA":      (1.7374e6,  3.844e8),
    "MARS":      (3.3895e6,  2.2794e11),
    "PHOBOS":    (1.1267e4,  9.376e6),
    "DEIMOS":    (6.2e3,     2.3463e7),
    "JUPITER":   (6.9911e7,  7.7857e11),
    "IO":        (1.8216e6,  4.217e8),
    "EUROPA":    (1.5608e6,  6.709e8),
    "GANYMEDE":  (2.6341e6,  1.0704e9),
    "CALLISTO":  (2.4103e6,  1.8827e9),
    "SATURN":    (5.8232e7,  1.4335e12),
    "ENCELADUS": (2.521e5,   2.3795e8),
    "RHEA":      (7.638e5,   5.2704e8),
    "TITAN":     (2.5747e6,  1.2219e9),
    "URANUS":    (2.5362e7,  2.8725e12),
    "TITANIA":   (7.884e5,   4.363e8),
    "OBERON":    (7.614e5,   5.835e8),
    "NEPTUNE":   (2.4622e7,  4.4951e12),
    "TRITON":    (1.3534e6,  3.5476e8),
}

# Which planet each moon goes round, for the angular-size report.
PRIMARY = {
    "LUNA": "TERRA", "PHOBOS": "MARS", "DEIMOS": "MARS",
    "IO": "JUPITER", "EUROPA": "JUPITER", "GANYMEDE": "JUPITER",
    "CALLISTO": "JUPITER", "ENCELADUS": "SATURN", "RHEA": "SATURN",
    "TITAN": "SATURN", "TITANIA": "URANUS", "OBERON": "URANUS",
    "TRITON": "NEPTUNE",
}

TOLERANCE = 0.005  # 0.5%


def parseConstants():
    """kSolRadius, kTerraSma, ... out of GameInternal.hpp."""
    path = os.path.join(REPO, "Game", "Source", "GameInternal.hpp")
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    found = {}
    for body, kind, value in re.findall(
            r"inline constexpr sw::f64 k(\w+?)(Radius|Sma)\s*=\s*([0-9.eE+-]+)\s*;",
            text):
        found.setdefault(body.upper(), {})[kind] = float(value)
    return found


def parseMoons():
    """The MoonDef rows in GameScene.cpp: {"NAME", radius, mu, sma, ...}."""
    path = os.path.join(REPO, "Game", "Source", "GameScene.cpp")
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    rows = {}
    for name, radius, _mu, sma in re.findall(
            r'\{"([A-Z]+)",\s*([0-9.eE+-]+),\s*([0-9.eE+-]+),\s*([0-9.eE+-]+),',
            text):
        rows[name] = (float(radius), float(sma))
    return rows


def apparentDiameterDegrees(radius, distance):
    """The angle a sphere of `radius` subtends at `distance` — asin, not atan.

    The silhouette is the TANGENT cone, not the cone through the centre and
    the equator, so the half-angle is asin(R/d). The two agree to a part in a
    million from any orbit and differ by a fifth from a low one, which is
    exactly where somebody would notice."""
    ratio = min(radius / distance, 1.0)
    return 2.0 * math.degrees(math.asin(ratio))


def main():
    constants = parseConstants()
    moons = parseMoons()

    game = {}
    for body, fields in constants.items():
        if "Radius" in fields:
            game[body] = (fields["Radius"], fields.get("Sma"))
    for body, (radius, sma) in moons.items():
        game[body] = (radius, sma)
    # Luna is a constant, not a MoonDef row.
    if "LUNA" in constants:
        game["LUNA"] = (constants["LUNA"]["Radius"], constants["LUNA"].get("Sma"))

    failures = 0
    print(f"{'BODY':<10} {'RADIUS m':>12} {'REF':>12} {'ERR':>8}   "
          f"{'ORBIT m':>12} {'REF':>12} {'ERR':>8}")
    for body, (refRadius, refSma) in REFERENCE.items():
        if body not in game:
            print(f"{body:<10}  MISSING FROM THE SCENE")
            failures += 1
            continue
        radius, sma = game[body]
        radiusError = abs(radius - refRadius) / refRadius
        line = f"{body:<10} {radius:12.4e} {refRadius:12.4e} {radiusError*100:7.3f}%"
        smaError = 0.0
        if refSma is not None:
            if sma is None:
                line += "   ORBIT MISSING"
                failures += 1
            else:
                smaError = abs(sma - refSma) / refSma
                line += f"   {sma:12.4e} {refSma:12.4e} {smaError*100:7.3f}%"
        bad = radiusError > TOLERANCE or smaError > TOLERANCE
        if bad:
            failures += 1
            line += "   <-- OUT OF TOLERANCE"
        print(line)

    # ---- what it all means, in degrees -------------------------------------
    print("\nTHE SUN, seen from each planet's cloud tops (apparent diameter):")
    solRadius = game["SOL"][0]
    for planet in ("MERCURY", "VENUS", "TERRA", "MARS", "JUPITER", "SATURN",
                   "URANUS", "NEPTUNE"):
        radius, sma = game[planet]
        gameAngle = apparentDiameterDegrees(solRadius, sma - radius)
        refAngle = apparentDiameterDegrees(REFERENCE["SOL"][0],
                                           REFERENCE[planet][1] - REFERENCE[planet][0])
        mark = "" if abs(gameAngle - refAngle) / refAngle < TOLERANCE else "  <-- WRONG"
        print(f"  from {planet:<9} {gameAngle:7.4f} deg   (real {refAngle:7.4f}){mark}")

    print("\nEACH MOON, seen from its planet's cloud tops:")
    for moon, planet in PRIMARY.items():
        moonRadius, moonSma = game[moon]
        planetRadius = game[planet][0]
        gameAngle = apparentDiameterDegrees(moonRadius, moonSma - planetRadius)
        refAngle = apparentDiameterDegrees(
            REFERENCE[moon][0], REFERENCE[moon][1] - REFERENCE[planet][0])
        mark = "" if abs(gameAngle - refAngle) / refAngle < TOLERANCE else "  <-- WRONG"
        print(f"  {moon:<10} from {planet:<8} {gameAngle:7.4f} deg"
              f"   (real {refAngle:7.4f}){mark}")

    print(f"\n{failures} discrepanc{'y' if failures == 1 else 'ies'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
