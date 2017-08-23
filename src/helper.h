
#ifndef CARND_T3_P1_HELPER_H
#define CARND_T3_P1_HELPER_H

#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace t3p1help {
    const double MAX_S = 6914.14925765991;
    // 1 meter per second = 2.23694 miles per hour
    const double MPS_TO_MPH = 2.23694;
    const double MAX_SPEED = 50.0;
    const int LANE_NUM = 3;

    // The state of the ego car
    // CS = Constant Speed
    // CLL = Changing Lane Left
    // CLR = Changing Lane Right
    enum EGO_STATE {CS, CLL, CLR};

    /**
     * Keep the state for the planner.
     */
    struct planner_state {
        EGO_STATE ego_state = CS;
        // current car lane
        int lane_num = 1;
        // current velocity in mph, mile per hour (MPH)
        double ref_velocity = 0.0;
        // velocity limit, MPH
        // double limit_velocity = 49.5;
        // double limit_velocity = 49.72; // over speed when constant changing lanes
        // double limit_velocity = 49.70;
        double limit_velocity = 49.65;
        // horizon way point size;
        double horizon_size = 50;
        // horizon distance in meter
        double horizon_dist = 30;
        // time between waypionts in second
        double point_dt = 0.02;
        // velocity change per point_dt without jerking
        double velocity_step = 0.224;
    } planner_state_t;

    /**
     * Return the lane number
     * @param d The d of Frenet coordinates
     * @return lane number
     */
    int getLaneFromD(double d) {
        int r;
        if (d < 4) { // inner lane
            r = 0;
        } else if ( d < 8) { // middle lane
            r = 1;
        } else { // outer lane
            r = 2;
        }
        return r;
    }

    /**
     * Get the Frenet d from lane number
     * @param lane
     * @return d
     */
    double getDFromLane(int lane) {
        return 2 + 4*lane;
    }

    /**
     * Sort the sensor fusion into lanes
     */
    std::vector<std::vector<std::vector<double>>>
    sortSensor(std::vector<std::vector<double>> sensors) {
        double d;
        std::vector<std::vector<double>> lane0, lane1, lane2;
        for (auto car: sensors) {
            d = car[6];
            switch (getLaneFromD(d)) {
            case 0: // inner lane
                lane0.push_back(car);
                break;
            case 1: // middle lane
                lane1.push_back(car);
                break;
            case 2: // outer lane
                lane2.push_back(car);
                break;
            }
        }
        return {lane0, lane1, lane2};
    }

    /**
     * Print the sensor fusion data
     * @param sensors
     */
    void printSensors(std::vector<std::vector<double>> sensors) {
        for (auto car: sensors) {
            for (double info : car) {
                std::cout << info << ", ";
            }
            std::cout << std::endl;
        }
    }

    /**
     * Get the speed from the sensor car data
     * @param car
     * @return
     */
    double getSensorCarSpeedMPS(std::vector<double> car) {
        return sqrt(car[3]*car[3] + car[4]*car[4]);
    }

    /**
     * Get the Frenet s distance ahead
     * @param time_ahead
     * @param car_s
     * @param car_d
     * @param range
     * @return
     */
    double
    getFrontDistS(double time_ahead, double car_s, double car_d,
                  const std::vector<std::vector<std::vector<double>>> lane_sensors) {
        double min_dist = MAX_S;
        int car_lane = getLaneFromD(car_d);
        double dist;
        double v_x, v_y, sen_speed, sen_s;
        for (auto car: lane_sensors[car_lane]) {
            dist = car[5] - car_s;
            sen_speed = getSensorCarSpeedMPS(car);

            dist += time_ahead * sen_speed;
            if (dist < 0) { // the sensored car is behind
                if (dist < (50 - MAX_S)) {
                    // S looped around
                    dist += MAX_S;
                }
            }
            if (dist >= 0) {
                if (dist < min_dist) {
                    min_dist = dist;
                }
            }
        }
        return min_dist;
    }

    /**
     * Get front car distance in S for all lanes
     * @param time_ahead
     * @param car_s
     * @param car_d
     * @return
     */
    std::vector<double>
    getFrontDistSs(double time_ahead, double car_s, double car_d,
                  const std::vector<std::vector<std::vector<double>>> lane_sensors) {
        std::vector<double> fronts;
        for (int l=0; l < LANE_NUM; l++) {
            fronts.push_back(getFrontDistS(time_ahead, car_s,
                                           getDFromLane(l),
                                           lane_sensors));
        }
        return fronts;
    }

    /**
     * Check if there is enough of space for changing lane
     * @param time_ahead The time in the future to check
     * @param car_s
     * @param car_d
     * @param car_v_mps in MPS
     * @return
     */
    bool hasSafeDistLane(double time_ahead, double car_s, double car_d, double car_v_mps,
                     const std::vector<std::vector<std::vector<double>>> lane_sensors) {
        int check_lane = getLaneFromD(car_d);
        double dist;
        double v_x, v_y, sen_speed, sen_s;
        for (auto car: lane_sensors[check_lane]) {
            dist = car[5] - car_s;
            sen_speed = getSensorCarSpeedMPS(car);
            /// if (car_v_mps + 3 < sen_speed) {
            ///     std::cout << "speed is too slow: car_v_mps " << car_v_mps
            ///               << " vs sen_speed " << sen_speed << std::endl;
            ///     return false;
            /// }
            dist += time_ahead * sen_speed;
            if (dist >= 0 ) { // sensored car infront
                if (dist < 15) {  // 21 is conservative
                    std::cout << "short front dist " << dist << std::endl;
                    return false;
                }
            } else { // sensored car behind
                if (car_v_mps > sen_speed) {
                    if (dist > -8) {
                        std::cout << "short back dist when faster: " << dist << std::endl;
                        return false;
                    }

                } else {
                   if (dist > -17) { // -15 is conservative
                       std::cout << "short back dist when slower: " << dist << std::endl;
                       return false;
                   }
                }

            }
        }
        return true;
    }

    /**
     * Detect if car infront is too close
     * @param time_ahead
     * @param car_s
     * @param cars_d
     * @return
     */
    bool tooClose(double time_ahead, double car_s, double car_d,
                  const std::vector<std::vector<std::vector<double>>> lane_sensors) {
        double front_dist_s = getFrontDistS(time_ahead, car_s, car_d, lane_sensors);
        bool ret = false;
        if (front_dist_s < 25) {
            ret = true;
        }
        return ret;
    }

    /**
     * Check if distant is closing for changing lane
     * @param time_ahead
     * @param car_s
     * @param car_d
     * @return
     */
    bool isCloseToChangeLane(double time_ahead, double car_s, double car_d,
              const std::vector<std::vector<std::vector<double>>> lane_sensors) {
        double front_dist_s = getFrontDistS(time_ahead, car_s, car_d, lane_sensors);
        bool ret = false;
        if (front_dist_s < 50) { // 20 is conservative
            ret = true;
        }
        return ret;
    }

    /**
     * Get the next safe changing lane
     * @param time_ahead
     * @param car_s
     * @param cars_d
     * @param cars_v_mps
     * @return
     */
    int getSafeChangeLane(double time_ahead, double car_s, double car_d, double car_v_mps,
                      const std::vector<std::vector<std::vector<double>>> lane_sensors) {
        int cur_lane = getLaneFromD(car_d);
        int safe_lane = cur_lane;
        if (cur_lane == 0 || cur_lane == 2) {
            // if (!tooClose(car_s+3, getDFromLane(1), lane_sensors)) {
            if (hasSafeDistLane(time_ahead, car_s, getDFromLane(1), car_v_mps, lane_sensors)) {
                safe_lane = 1;
            }
        } else if (cur_lane == 1) {
            std::vector<double> fronts =
                    getFrontDistSs(time_ahead, car_s, car_d,lane_sensors);
            bool safe0 =
                    hasSafeDistLane(time_ahead, car_s,
                                    getDFromLane(0), car_v_mps, lane_sensors);
            bool safe2 =
                    hasSafeDistLane(time_ahead, car_s,
                                    getDFromLane(2), car_v_mps, lane_sensors);
            if (safe0 && safe2) {
                if (fronts[0] > fronts[2]) {
                    safe_lane = 0;
                } else {
                    safe_lane = 2;
                }
            } else if (safe0) {
                    safe_lane = 0;
            } else if (safe2) {
                    safe_lane = 2;
            }
        }

        return safe_lane;
    }

    /**
     * Return the lowest cars speed in each lane
     * @param lane_sensors
     * @return
     */
    std::vector<double>
    getLaneSpeedsMPH(const std::vector<std::vector<std::vector<double>>> lane_sensors) {
        double min_speed;
        double car_speed;
        std::vector<double> lane_speeds;
        for (int l=0; l < LANE_NUM; l++) {
            min_speed = MAX_SPEED;
            for (auto car: lane_sensors[l]) {
                car_speed = getSensorCarSpeedMPS(car);
                if (car_speed < min_speed) {
                    min_speed = car_speed;
                }
            }
            lane_speeds.push_back(min_speed*MPS_TO_MPH);
        }
        return lane_speeds;
    }

}
#endif //CARND_T3_P1_HELPER_H
