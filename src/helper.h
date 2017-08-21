
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

    /**
     * Keep the state for the planner.
     */
    struct planner_state {
        // current car lane
        int lane_num = 1;
        // current velocity in mph, mile per hour (MPH)
        double ref_velocity = 0.0;
        // velocity limit, MPH
        double limit_velocity = 49.5;
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
     * Get the Frenet s distance ahead
     * @param time_ahead
     * @param car_s
     * @param car_d
     * @param range
     * @return
     */
    double getFrontDistS(double time_ahead, double car_s, double car_d,
                     const std::vector<std::vector<std::vector<double>>> lane_sensors) {
        double min_dist = MAX_S;
        int car_lane = getLaneFromD(car_d);
        double dist;
        double v_x, v_y, sen_speed, sen_s;
        for (auto car: lane_sensors[car_lane]) {
            dist = car[5] - car_s;
            v_x = car[3];
            v_y = car[4];
            sen_speed = sqrt(v_x*v_x + v_y*v_y);

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
     * Check if there is enough of space for changing lane
     * @param time_ahead The time in the future to check
     * @param car_s
     * @param car_d
     * @return
     */
    bool hasSafeDistLane(double time_ahead, double car_s, double car_d,
                     const std::vector<std::vector<std::vector<double>>> lane_sensors) {
        int check_lane = getLaneFromD(car_d);
        double dist;
        double v_x, v_y, sen_speed, sen_s;
        for (auto car: lane_sensors[check_lane]) {
            dist = car[5] - car_s;
            v_x = car[3];
            v_y = car[4];
            sen_speed = sqrt(v_x*v_x + v_y*v_y);

            dist += time_ahead * sen_speed;
            if (dist >= 0 ) { // sensored car infront
                if (dist < 21) {
                    std::cout << "short front dist " << dist << std::endl;
                    return false;
                }
            } else { // sensored car behind
                if (dist > -10) {
                    std::cout << "short back dist " << dist << std::endl;
                    return false;
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
        if (front_dist_s < 20) {
            ret = true;
        }
        return ret;
    }

    /**
     * Get the next safe changing lane
     * @param time_ahead
     * @param car_s
     * @param cars_d
     * @return
     */
    int getSafeChangeLane(double time_ahead, double car_s, double car_d,
                      const std::vector<std::vector<std::vector<double>>> lane_sensors) {
        int cur_lane = getLaneFromD(car_d);
        int safe_lane = cur_lane;
        if (cur_lane == 0 || cur_lane == 2) {
            // if (!tooClose(car_s+3, getDFromLane(1), lane_sensors)) {
            if (hasSafeDistLane(time_ahead, car_s, getDFromLane(1), lane_sensors)) {
                safe_lane = 1;
            }
        } else if (cur_lane == 1) {
            if (hasSafeDistLane(time_ahead, car_s, getDFromLane(0), lane_sensors)) {
                safe_lane = 0;
            } else if (hasSafeDistLane(time_ahead, car_s, getDFromLane(2), lane_sensors)) {
                safe_lane = 2;
            }
        }
        return safe_lane;
    }

    /** JMT source code from Udacity: Implement Quintic Polynomial Solver Solution
     * @param start
     * @param end
     * @param T
     * @return
     */
    std::vector<double> JMT(std::vector< double> start, std::vector <double> end, double T) {
        /* Calculate the Jerk Minimizing Trajectory that connects the initial state
         * to the final state in time T.
         * INPUTS
         * start - the vehicles start location given as a length three array
         * corresponding to initial values of [s, s_dot, s_double_dot]
         * end   - the desired end state for vehicle. Like "start" this is a
         * length three array.
         * T     - The duration, in seconds, over which this maneuver should occur.
         * OUTPUT
         * an array of length 6, each value corresponding to a coefficent in the polynomial
         * s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
         *
         * EXAMPLE
         *
         * > JMT( [0, 10, 0], [10, 10, 0], 1)
         * [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
        */

        MatrixXd A = MatrixXd(3, 3);
        A << T*T*T, T*T*T*T, T*T*T*T*T,
                3*T*T, 4*T*T*T,5*T*T*T*T,
                6*T, 12*T*T, 20*T*T*T;

        MatrixXd B = MatrixXd(3,1);
        B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
                end[1]-(start[1]+start[2]*T),
                end[2]-start[2];

        MatrixXd Ai = A.inverse();

        MatrixXd C = Ai*B;

        std::vector <double> result = {start[0], start[1], .5*start[2]};
        for(int i = 0; i < C.size(); i++)
        {
            result.push_back(C.data()[i]);
        }

        return result;

    }

    double eval_poly(std::vector<double> poly, double x) {
        int size = poly.size();
        double ret = poly[0];
        double p = 1;
        for (int i = 1; i < size; i++) {
            p *= x;
            ret += poly[i]*p;
        }
        return ret;
    }

}
#endif //CARND_T3_P1_HELPER_H
