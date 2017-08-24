
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
    const int ROAD_NUM_LANES = 3;

    // The state of the ego car
    // CS = Constant Speed
    // CL = Changing CL
    // CLL = Changing Lane Left
    // CLR = Changing Lane Right
    // PCS = Prepare changing Lane
    enum class EGO_STATE {CS, CL, CLL, CLR, PCL};

    // The event to transition the state
    // TOO_CLOSE = too close to the car in front
    // CLOSE_TO_CHANGE_LANE = time to change lane
    // FREE_AHEAD = no cars in front
    enum class STATE_EVENT_ID {
        TOO_CLOSE,
        CLOSE_TO_CHANGE_LANE,
        CHANGE_LANE,
        FREE_AHEAD};

    typedef struct state_event{
        STATE_EVENT_ID id;
        double time_ahead;
        double car_s;
        double car_d;
        double car_speed; // MPH
        int changing_lane = -1;
        std::vector<std::vector<std::vector<double>>>
                lane_sensors;
    } state_event_t;

    /**
     * Keep the state for the planner.
     */
    typedef struct planner_state {
        EGO_STATE ego_state = EGO_STATE::CS;
        // current car lane
        int lane_num = 1;
        int pre_lane_num = 1;
        // current velocity in mph, mile per hour (MPH)
        double ref_velocity = 0.0;
        // velocity limit, MPH
        // double limit_velocity = 49.5;
        // double limit_velocity = 49.72; // over speed when constant changing lanes
        // double limit_velocity = 49.70;
        double limit_velocity = 49.65;
        // velocity when changing lane to avoid speed violations
        double limit_velocity_CL = 49.45; // 49.45; // 49.50 bit fast
        // velocity when costant speed
        double limit_velocity_CS = 49.5; // 49.727 good without change right way; // 49.725, 49.728 bit too fast;
        // horizon way point size;
        double horizon_size = 50;
        // horizon distance in meter
        double horizon_dist = 30;
        // time between waypionts in second
        double point_dt = 0.02;
        // velocity change per point_dt without jerking
        double velocity_step = 0.224;
        // Counting for staying in current state
        int state_ticks = 0;
    } planner_state_t;

    bool isState(planner_state_t& ps, EGO_STATE state) {
        return ps.ego_state == state;
    }

    state_event_t createStateEvent(STATE_EVENT_ID id,
                                   double time_ahead,
                                   double car_s, double car_d, double car_speed,
                                   std::vector<std::vector<std::vector<double>>> lane_sensors) {
        state_event_t event;
        event.id = id;
        event.time_ahead = time_ahead;
        event.car_s = car_s;
        event.car_d = car_d;
        event.car_speed = car_speed;
        event.lane_sensors = lane_sensors;
        return event;
    }

    /*
     * Entering CL state
     */
    void enterCL(planner_state_t & ps) {
        std::cout << "entering CL:" << std::endl;
        ps.ego_state = EGO_STATE::CL;
        ps.state_ticks = 0;
        ps.limit_velocity = ps.limit_velocity_CL;
    }

    void stayCL(planner_state_t& ps) {
        ps.state_ticks++;
        if (ps.state_ticks % 50 == 0) {
            std::cout << "staying CL: ticks: " << ps.state_ticks << std::endl;
        }
    }

    void enterCS(planner_state_t& ps) {
        std::cout << "entering CS:" << std::endl;
        ps.ego_state = EGO_STATE::CS;
        ps.state_ticks = 0;
        ps.limit_velocity = ps.limit_velocity_CS;
    }

    void stayCS(planner_state_t& ps) {
        ps.state_ticks++ ;
        if (ps.state_ticks % 50 == 0) {
            std::cout << "staying CS: " << ps.state_ticks << std::endl;
        }
    }

    void enterPCL(planner_state_t& ps) {
        std::cout << "entering PCL:" << std::endl;
        ps.ego_state = EGO_STATE::PCL;
        ps.state_ticks = 0;
        ps.limit_velocity = ps.limit_velocity_CS;
    }

    void stayPCL(planner_state_t& ps) {
        if (ps.state_ticks % 50 == 0) {
            std::cout << "staying PCL:" << ps.state_ticks << std::endl;
        }
        ps.state_ticks++;
    }

    void speedUp(planner_state_t& ps) {
        if (ps.ref_velocity < ps.limit_velocity) {
            ps.ref_velocity += ps.velocity_step;
        }
    }

    bool isTransitionCL(planner_state_t& ps, int trans_ticks = 50) {
        return (isState(ps, EGO_STATE::CL) && ps.state_ticks < trans_ticks);
    }

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
        for (int l=0; l < ROAD_NUM_LANES; l++) {
            fronts.push_back(getFrontDistS(time_ahead, car_s,
                                           getDFromLane(l),
                                           lane_sensors));
        }
        /// std::cout << "front space: ";
        /// for (int l=0; l < fronts.size(); l++) {
        ///     std::cout << fronts[l] << " ";
        /// }
        /// std::cout << std::endl;
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
                if (car_v_mps > sen_speed) {
                    if (dist < 22) {  // 15 a bit aggressive, 21 is conservative
                        /// std::cout << "short front dist " << dist << std::endl;
                        return false;
                    }
                } else {
                    if (dist < 17) {
                        /// std::cout << "slower front dist " << dist << std::endl;
                        return false;
                    }
                }
            } else { // sensored car behind
                if (car_v_mps > sen_speed) {
                    if (dist > -8) {
                        /// std::cout << "short back dist when faster: " << dist << std::endl;
                        return false;
                    }

                } else {
                   if (dist > -19) { // -15 is conservative, -17 ok
                       /// std::cout << "short back dist when slower: " << dist << std::endl;
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
        int head_room = getFrontDistS(time_ahead, car_s, car_d, lane_sensors);

        if (head_room < 5) {
            /// std::cout << "not enough of head room to change lane" << std::endl;
            return safe_lane;
        }

        if (car_v_mps * MPS_TO_MPH < 30) {
            /// std::cout << "too slow to chang lane at speed " << car_v_mps*MPS_TO_MPH << std::endl;
            return safe_lane;
        }

        if (cur_lane == 0 || cur_lane == 2) {
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
        for (int l=0; l < ROAD_NUM_LANES; l++) {
            min_speed = MAX_SPEED;
            for (auto car: lane_sensors[l]) {
                car_speed = getSensorCarSpeedMPS(car);
                if (car_speed < min_speed) {
                    min_speed = car_speed;
                }
            }
            lane_speeds.push_back(min_speed*MPS_TO_MPH);
        }
        /// std::cout << "lane speed : ";
        /// for (int l=0; l < lane_speeds.size(); l++) {
        /// 	std::cout << lane_speeds[l] << " ";
        /// }
        /// std::cout << " car_v: " << car_speed << " car_v_mps: "
        /// 		  << car_v_mps << std::endl;
        return lane_speeds;
    }

    void adjustSpeed(planner_state_t& ps,
                     bool tooClose,
                     const std::vector<double>& lane_speeds,
                     int changing_lane) {
        if (tooClose) {
            if (ps.ref_velocity > lane_speeds[changing_lane] - 3) {
                ps.ref_velocity -= ps.velocity_step;
                /// std::cout << "decreasing speed to " << ps.ref_velocity << std::endl;
            } else {
                /// std::cout << "stay lane speed in " << ps.ref_velocity << std::endl;
            }

        } else {
            speedUp(ps);
        }
    }

    void handleEventFreeAhead(planner_state_t& ps,
                              const state_event_t& event) {
        switch (ps.ego_state) {
            case EGO_STATE::CS:
                stayCS(ps);
                break;
            case EGO_STATE::PCL:
                enterCS(ps);
                break;
            case EGO_STATE::CL:
                enterCS(ps);
                break;
            default:
                break;
        }
        bool too_close = tooClose(event.time_ahead, event.car_s,
                                 event.car_d, event.lane_sensors);
        std::vector<double> lane_speeds = getLaneSpeedsMPH(event.lane_sensors);
        adjustSpeed(ps, too_close, lane_speeds, ps.lane_num);
    }

    void handleEventChangeLane(planner_state_t& ps, const state_event_t& event) {
        bool canChangeLane = false;
        switch (ps.ego_state) {
            case EGO_STATE::CS:
                std::cout << "entering CL from CS" << std::endl;
                /// ps.lane_num = event.changing_lane;
                canChangeLane = true;
                break;
            case EGO_STATE::PCL:
                if (ps.state_ticks % 10 == 0) {
                    std::cout << "from PCL: current  lane: " << ps.lane_num << std::endl;
                    std::cout << "previous lane: " << ps.pre_lane_num << std::endl;
                }
                if (ps.lane_num == event.changing_lane) {
                    /// should not happen
                } else {
                    canChangeLane = true;
                }
                break;
            case EGO_STATE::CL:
                if (ps.state_ticks % 10 == 0) {
                    std::cout << "from CL: current  lane: " << ps.lane_num << std::endl;
                    std::cout << "previous lane: " << ps.pre_lane_num << std::endl;
                }
                if (ps.lane_num == event.changing_lane) {
                } else {
                    canChangeLane = true;
                }
                break;
            default:
                break;
        }
        /// if (isTransitionCL(ps) && ps.pre_lane_num == event.changing_lane) {
        if (canChangeLane) {
            if (isTransitionCL(ps)) {
                stayPCL(ps);
                if (ps.state_ticks % 10 == 0) {
                    std::cout << "NOT CHANGING RIGHT AWAY. ticks= " << ps.state_ticks << std::endl;
                }
            } else {
                std::cout << "CHANGING RIGHT AFTER ticks: " << ps.state_ticks << std::endl;
                enterCL(ps);
                ps.pre_lane_num = ps.lane_num;
                ps.lane_num = event.changing_lane;
            }
        } else {
          stayCL(ps);
        }
    }

    void postStateEvent(planner_state_t& ps, const state_event_t& event);

    void handleEventCloseToChangeLane(planner_state_t& ps, const state_event_t& event) {
        /// std::cout << "close to change lane: " << std::endl;
        double car_v_mps = event.car_speed / MPS_TO_MPH;
        bool hasChageLaneEvent = false;
        int changing_lane =
                getSafeChangeLane(event.time_ahead,
                                  event.car_s,
                                  event.car_d,
                                  car_v_mps,
                                  event.lane_sensors);
        switch (ps.ego_state) {
            case EGO_STATE::CS:
                enterPCL(ps);
                hasChageLaneEvent = true;
                break;
            case EGO_STATE::PCL:
                if (ps.lane_num == changing_lane) {
                    stayPCL(ps);
                } else {
                    hasChageLaneEvent = true;
                }
                break;
            case EGO_STATE::CL:
                hasChageLaneEvent = true;
                break;
            default:
                break;
        }
        if (hasChageLaneEvent) {
            if (ps.ego_state == EGO_STATE::PCL) {
                state_event_t new_event = createStateEvent(
                        STATE_EVENT_ID::CHANGE_LANE,
                        event.time_ahead, event.car_s, event.car_d,
                        event.car_speed, event.lane_sensors);
                new_event.changing_lane = changing_lane;
                postStateEvent(ps, new_event);
            }
        } else {
            bool too_close = tooClose(event.time_ahead, event.car_s,
                                      event.car_d, event.lane_sensors);
            std::vector<double> lane_speeds = getLaneSpeedsMPH(event.lane_sensors);
            adjustSpeed(ps, too_close, lane_speeds, changing_lane);
        }
    }

    void postStateEvent(planner_state_t& ps, const state_event_t& event) {
        switch (event.id) {
            case STATE_EVENT_ID::FREE_AHEAD:
                handleEventFreeAhead(ps, event);
                break;
            case STATE_EVENT_ID::CLOSE_TO_CHANGE_LANE:
                handleEventCloseToChangeLane(ps, event);
                break;
            case STATE_EVENT_ID::CHANGE_LANE:
                handleEventChangeLane(ps, event);
                break;
            case STATE_EVENT_ID::TOO_CLOSE:
                break;
            default:
                break;
        }
    }

}
#endif //CARND_T3_P1_HELPER_H
