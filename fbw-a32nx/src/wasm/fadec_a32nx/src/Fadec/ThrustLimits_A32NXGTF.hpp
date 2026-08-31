// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0
// PW1127G-JM initial iteration

#ifndef FLYBYWIRE_AIRCRAFT_THRUSTLIMITS_A32NXGTF_HPP
#define FLYBYWIRE_AIRCRAFT_THRUSTLIMITS_A32NXGTF_HPP

#include <algorithm>
#include <cmath>

#include "EngineRatios.hpp"
#include "Fadec.h"

/**
 * @class ThrustLimits_A32NX
 * @brief A static class that provides methods for calculating various engine thrust limits for the A33NX aircraft.
 *
 * TODO: extract the reusable code to a common library
 */
class ThrustLimits_A32NXGTF {
  /**
   * @brief A 2D array representing various engine thrust limits.
   *
   * This 2D array contains 72 rows, each with 6 columns. Each row represents a different altitude
   * level and the corresponding engine thrust
   * limits. The columns in each row represent the following parameters:
   * 1. Altitude (in feet)
   * 2. Corner Point (CP) - the temperature below which the engine can operate at full thrust without any restrictions.
   * 3. Limit Point (LP) - the temperature above which the engine thrust starts to be limited.
   * 4. CN1 Flat - the engine's N1 fan speed limit at the CP temperature.
   * 5. CN1 Last - the engine's N1 fan speed limit at the LP temperature.
   * 6. CN1 Flex - the engine's N1 fan speed limit at a temperature of 100 degrees Celsius.
   *
   * The array is divided into sections for different flight phases: Takeoff (TO), Go-Around (GA),
   * Climb (CLB), and Maximum Continuous Thrust (MCT).
   */
  static constexpr double limits[64][6] = {
  // TO
      {-2000,   51.000,  55.000,  84.3, 84.0, 61.535}, // 0
      {-1000,   49.000,  55.000,  85.3, 84.8, 62.105}, //
      {0,       47.000,  55.000,  85.7, 85.6, 62.655}, //
      {1000,    47.000,  52.000,  86.9, 86.4, 62.655}, //
      {2000,    47.000,  50.000,  88.1, 87.2, 62.655}, //
      {3000,    39.000,  48.000,  88.7, 87.4, 61.960}, //
      {4000,    35.000,  42.000,  89.4, 88.5, 61.206}, //
      {5000,    31.000,  42.000,  90.0, 88.5, 61.206}, //
      {6000,    25.000,  42.000,  90.3, 87.9, 61.206}, //
      {7000,    21.000,  40.000,  90.8, 87.7, 61.258}, //
      {8000,    19.000,  38.000,  91.6, 88.7, 61.777}, //
      {9000,    17.000,  36.000,  91.9, 89.1, 60.968}, //
      {10000,   15.000,  34.000,  92.1, 89.5, 60.935}, //
      {11000,   12.000,  32.000,  92.5, 89.8, 59.955}, //
      {12000,   7.000,  30.000,  92.9, 90.1, 58.677}, //
      {13000,   4.000,  28.000,  93.2, 90.4, 59.323}, //
      {14000,   4.000,  26.000,  93.3, 90.7, 59.965}, //
      {15000,   1.000,   24.000,  93.6, 91.0, 58.723}, //
      {16000,   -1.000,   22.000,  94.0, 92.0, 57.189}, // 18
  // GA
      {-2000,   54,  60,  87.4, 85.7, 63.498}, // 19
      {-1000,   52,  60,  88.3, 85.7, 63.920}, //
      {0,       50,  58,  89.2, 86.6, 64.397}, //
      {1000,    48,  56,  90, 87.2, 64.525}, //
      {2000,    46,  54,  90.7, 88, 64.489}, //
      {3000,    42,  52,  91.1, 88.7, 63.364}, //
      {4000,    38,  50,  91.7, 89.5, 62.875}, //
      {5000,    34,  48,  92.2, 90.4, 62.614}, //
      {6000,    28,  46,  92.7, 90.5, 62.290}, //
      {7000,    24,  44,  92.9, 90.5, 61.952}, //
      {8000,    20,  42,  92.9, 91, 62.714}, //
      {9000,    18,  40,  92.9, 91.4, 61.692}, //
      {10000,   16,  38,  93, 91.9, 60.906}, //
      {11000,   14,  36,  93, 92, 59.770}, //
      {12000,   10,  34,  93, 92, 58.933}, //
      {13000,   8,  32,  93, 92.1, 60.503}, //
      {14000,   5,   30,  92.8, 92.2, 62.072}, //
      {15000,   5,   28,  92.7, 92.1, 59.333}, //
      {16000,   2,   26,  92.6, 92.4, 58.045}, // 37
  // CLB
      {2000,    34,  58,  84.1, 79.8, 0.000 }, //38
      {5000,    26,  54,  85.6, 80.2, 0.000 }, //
      {8000,    22,   50,  86.1, 80.9, 0.000 }, //
      {12000,   22,   50,  87.5, 82.7, 0.000 }, //
      {15000,   18,  46,  88.7, 84, 0.000 }, //
      {17000,   14,  42,  89.2, 84.6, 0.000 }, //
      {20000,   10, 38,  90, 85.5, 0.000 }, //
      {25000,   2, 34,   91.2, 86.4, 0.000 }, //
      {27000,   2, 30,  90.8, 86.5, 0.000 }, //
      {31000,   -6, 26,  90.7, 85.6, 0.000 }, //
      {35000,   -14, 14, 90.1, 85.9, 0.000 }, //
      {39000,   -18, 14, 90, 84.8, 0.000 }, //
      {41000,   -18, 14, 89.7, 84.1, 0.000 }, // 50
  // MCT
      {-1000,   34,  62,  83.6, 78.6, 0.000 }, // 51
      {3000,    30,  54,  85.3, 80.7, 0.000 }, //
      {7000,    22,   50,  87.8, 82.6, 0.000 }, //
      {11000,   14,   42,  90, 85.4, 0.000 }, //
      {15000,   10,  34,  91.9, 88.7, 0.000 }, //
      {17000,   6,  34,  92.6, 88.6, 0.000 }, //
      {20000,   2, 30,  93, 89.3, 0.000 }, //
      {22000,   -2, 30,  93.1, 88.3, 0.000 }, //
      {25000,   -6, 26,   90.9, 86.4, 0.000 }, //
      {27000,   -10, 18,  89.9, 86.4, 0.000 }, //
      {31000,   -14, 14,  89.7, 85.7, 0.000 }, //
      {35000,   -18, 10, 89.4, 85, 0.000 }, //
      {39000,   -18, 10, 89.4, 84.7, 0.000 }  // 63
  };

 public:
  /**
   * @brief Finds the top-row boundary in the limits array.
   *
   * @param altitude The altitude to find the top-row boundary for.
   * @param index The index to start the search from.
   * @return The index of the top-row boundary in the limits array.
   */
  static int finder(double altitude, int index) {
    while (altitude >= limits[index][0]) {
      index++;
    }
    return index;
  }

  /**
   * @brief Structure to store the bleed values for different types of limits.
   *
   * This structure contains three members, each representing a specific bleed value:
   * - `n1Packs`: A double representing the bleed value for the packs.
   * - `n1Nai`: A double representing the bleed value for the nacelle anti-ice.
   * - `n1Wai`: A double representing the bleed value for the wing anti-ice.
   */
  struct BleedValues {
    double n1Packs;
    double n1Nai;
    double n1Wai;
  };

  /**
   * @brief Lookup table for bleed values based on limit type.
   *
   * This map stores the bleed values for different types of limits. The key is an integer
   * representing the type of limit (0-TO, 1-GA, 2-CLB, 3-MCT),
   * and the value is a `BleedValues` structure containing the values for `n1Packs`, `n1Nai`, and `n1Wai`.
   */
  static std::map<int, BleedValues> bleedValuesLookup;  // see below for initialization

  /**
   * @brief Calculates the total bleed for the engine.
   *
   * @param type The type of operation (0-TO, 1-GA, 2-CLB, 3-MCT).
   * @param altitude The current altitude of the aircraft in feet.
   * @param oat The outside air temperature in degrees Celsius.
   * @param cp The corner point - the temperature below which the engine can operate at full thrust without any restrictions (in degrees
   * Celsius).
   * @param lp The limit point - the temperature above which the engine thrust starts to be limited (in degrees Celsius).
   * @param flexTemp The flex temperature in degrees Celsius.
   * @param packs The status of the air conditioning (0 for off, 1 for on).
   * @param nacelle The status of the nacelle anti-ice (0 for off, 1 for on).
   * @param wing The status of the wing anti-ice (0 for off, 1 for on).
   * @return The total bleed for the engine
   */
  static double bleedTotal(int    type,      //
                           double altitude,  //
                           double oat,       //
                           double cp,        //
                           double lp,        //
                           double flexTemp,  //
                           int    packs,     //
                           int    nacelle,   //
                           int    wing       //
  ) {
    if (flexTemp > lp && type <= 1) {
      return packs * -0.6 + nacelle * -0.7 + wing * -0.7;
    }

    // Define a map to store the bleed values for different conditions
    // Keys:
    // int - Represents the type of operation (0-TO, 1-GA, 2-CLB, 3-MCT).
    // bool - Represents whether the altitude is less than 8000.
    // bool - Represents whether the outside air temperature is less than the corner point.
    // Values:
    // double - Represents the bleed value for the packs.
    // double - Represents the bleed value for the nacelle anti-ice.
    // double - Represents the bleed value for the wing anti-ice.
    std::map<std::tuple<int, bool, bool>, std::tuple<double, double, double>> bleedValues = {
        {{0, true, true},   {-0.4, -0.6, -0.7}}, //
        {{0, true, false},  {-0.5, -0.6, -0.7}}, //
        {{0, false, true},  {-0.6, -0.8, -0.8}}, //
        {{0, false, false}, {-0.7, -0.8, -0.8}}, //
        {{1, true, true},   {-0.4, -0.6, -0.6}}, //
        {{1, true, false},  {-0.4, -0.6, -0.6}}, //
        {{1, false, true},  {-0.6, -0.7, -0.8}}, //
        {{1, false, false}, {-0.6, -0.7, -0.8}}, //
        {{2, true, false},  {-0.2, -0.8, -0.4}}, //
        {{2, false, false}, {-0.3, -0.8, -0.4}}, //
        {{3, true, false},  {-0.6, -0.9, -1.2}}, //
        {{3, false, false}, {-0.6, -0.9, -1.2}}  //
    };

    double n1Packs = 0;
    double n1Nai   = 0;
    double n1Wai   = 0;

    // Use the map to get the bleed values
    std::tie(n1Packs, n1Nai, n1Wai) = bleedValues[{type, altitude < 8000, oat < cp}];

    return packs * n1Packs + nacelle * n1Nai + wing * n1Wai;
  }

  /**
   * @brief Calculates the N1 limit for the engine.
   *
   * This function calculates the N1 limit for the engine based on various parameters such as the
   * type of operation, altitude, ambient temperature, ambient pressure, flex temperature, and the
   * status of the air conditioning (AC), nacelle anti-ice (nacelle), and wing anti-ice (wing).
   *
   * @param type The type of operation (0-TO, 1-GA, 2-CLB, 3-MCT).
   * @param altitude The current altitude of the aircraft.
   * @param ambientTemp The ambient temperature.
   * @param ambientPressure The ambient pressure.
   * @param flexTemp The flex temperature.
   * @param packs The status of the air conditioning (0 for off, 1 for on).
   * @param nacelle The status of the nacelle anti-ice (0 for off, 1 for on).
   * @param wing The status of the wing anti-ice (0 for off, 1 for on).
   * @return The N1 limit for the engine.
   */
  static double limitN1(int    type,             //
                        double altitude,         //
                        double ambientTemp,      //
                        double ambientPressure,  //
                        double flexTemp,         //
                        int    packs,            //
                        int    nacelle,          //
                        int    wing              //
  ) {
    int    rowMin   = 0;
    int    rowMax   = 0;
    int    loAltRow = 0;
    int    hiAltRow = 0;
    double mach     = 0;

    // Set main variables per Limit Type
    switch (type) {
      case 0:  // TO
        rowMin = 0;
        rowMax = 18;
        mach   = 0;
        break;
      case 1:  // GA
        rowMin = 19;
        rowMax = 37;
        mach   = 0.225;
        break;
      case 2:  // CLB
        rowMin = 38;
        rowMax = 50;
        if (altitude <= 10000) {
          mach = Fadec::cas2mach(250, ambientPressure);
        } else {
          mach = Fadec::cas2mach(300, ambientPressure);
          if (mach > 0.78)
            mach = 0.78;
        }
        break;
      case 3:  // MCT
        rowMin = 51;
        rowMax = 63;
        mach   = Fadec::cas2mach(230, ambientPressure);
        break;
    }

    // Check for over/under flows. Else, find top row value
    if (altitude <= limits[rowMin][0]) {
      hiAltRow = rowMin;
      loAltRow = rowMin;
    } else if (altitude >= limits[rowMax][0]) {
      hiAltRow = rowMax;
      loAltRow = rowMax;
    } else {
      hiAltRow = finder(altitude, rowMin);
      loAltRow = hiAltRow - 1;
    }

    // Define key table variables and interpolation
    const double cp      = Fadec::interpolate(altitude, limits[loAltRow][0], limits[hiAltRow][0], limits[loAltRow][1], limits[hiAltRow][1]);
    const double lp      = Fadec::interpolate(altitude, limits[loAltRow][0], limits[hiAltRow][0], limits[loAltRow][2], limits[hiAltRow][2]);
    const double cn1Flat = Fadec::interpolate(altitude, limits[loAltRow][0], limits[hiAltRow][0], limits[loAltRow][3], limits[hiAltRow][3]);
    const double cn1Last = Fadec::interpolate(altitude, limits[loAltRow][0], limits[hiAltRow][0], limits[loAltRow][4], limits[hiAltRow][4]);
    const double cn1Flex = Fadec::interpolate(altitude, limits[loAltRow][0], limits[hiAltRow][0], limits[loAltRow][5], limits[hiAltRow][5]);

    double cn1 = 0;
    double m   = 0;
    double b   = 0;
    if (flexTemp > 0 && type <= 1) {  // CN1 for Flex Case
      if (flexTemp <= cp) {
        cn1 = cn1Flat;
      } else if (flexTemp > lp) {
        m   = (cn1Flex - cn1Last) / (100 - lp);
        b   = cn1Flex - m * 100;
        cn1 = (m * flexTemp) + b;
      } else {
        m   = (cn1Last - cn1Flat) / (lp - cp);
        b   = cn1Last - m * lp;
        cn1 = (m * flexTemp) + b;
      }
    } else {  // CN1 for All other cases
      if (ambientTemp <= cp) {
        cn1 = cn1Flat;
      } else {
        m   = (cn1Last - cn1Flat) / (lp - cp);
        b   = cn1Last - m * lp;
        cn1 = (m * ambientTemp) + b;
      }
    }

    // Define bleed rating/ de-rating
    const double bleed = bleedTotal(type, altitude, ambientTemp, cp, lp, flexTemp, packs, nacelle, wing);

    return (cn1 * (std::sqrt)(EngineRatios::theta2(mach, ambientTemp))) + bleed;
  }
};

#endif  // FLYBYWIRE_AIRCRAFT_THRUSTLIMITS_A32NX_HPP
