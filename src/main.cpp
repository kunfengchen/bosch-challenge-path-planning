#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helper.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	//// Bug found by Kostas Oreopoulos from Slack chanel.
	// while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	while((prev_wp < (int)(maps_s.size()-1) && s > maps_s[prev_wp+1] ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // the planner state
  struct t3p1help::planner_state p_state;

  p_state.lane_num = 1;
  p_state.ref_velocity = 0.0;
  p_state.limit_velocity = 49.5;
  p_state.horizon_size = 50;
  p_state.horizon_dist = 30.0;
  p_state.point_dt = 0.02;
  p_state.velocity_step = 0.224;

  ///// h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
  h.onMessage([&p_state,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
					  &map_waypoints_dx,&map_waypoints_dy]
					  (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the dajta JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
			// vector<vector<double>>
          	auto sensor_fusion = j[1]["sensor_fusion"];

			int prev_size = previous_path_x.size();

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


          	// DONE: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            /// Starter code provided from the lecture

			auto lane_sensors = t3p1help::sortSensor(sensor_fusion);
			/// Print the sensor_fusion info
			// for (auto lane : lanes) {
			//     cout << "lane sensors:" << endl;
			//     t3p1help::printSensors(lane);
			// }

          	int pre_size = previous_path_x.size();
			// The right time for generating the additional path;
			double time_ahead = pre_size * p_state.point_dt;

			if (pre_size >0) {
				car_s = end_path_s;
			}

            ///// Checking the sensors and state
			///// Adjust the state accordingly
            // Too close
			if (t3p1help::tooClose(time_ahead, car_s, car_d, lane_sensors)) {
				cout << "too close" << endl;
				int changing_lane = t3p1help::getSafeChangeLane(time_ahead, car_s, car_d, lane_sensors);
				if (p_state.lane_num == changing_lane) {
					cout << "Not safe to change lane. Stay in lane " << p_state.lane_num << endl;
				} else {
					p_state.lane_num = t3p1help::getSafeChangeLane(time_ahead, car_s, car_d, lane_sensors);
					cout << "Changing lane to " << p_state.lane_num << endl;
				}

				p_state.ref_velocity -= p_state.velocity_step;
				cout << "decreasing speed" << endl;
			} else if (p_state.ref_velocity < p_state.limit_velocity) {
				cout << "increasing speed" << endl;
				p_state.ref_velocity += p_state.velocity_step;
			} else {
				cout << "constant speed" << endl;
			}

			// Get the horizon way points in global coordinates
			vector<double> ptsx;
			vector<double> ptsy;

			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);
            double pre_car_x;
			double pre_car_y;
            double ref_x_pre;
			double ref_y_pre;

			if (pre_size < 2) {
				pre_car_x = car_x - cos(car_yaw);
				pre_car_y = car_y - sin(car_yaw);

				/// NOT always monotonic, depends on the starting location!
				ptsx.push_back(pre_car_x);
				ptsy.push_back(pre_car_y);
				ptsx.push_back(car_x);
				ptsy.push_back(car_y);
			} else {
				ref_x = previous_path_x[pre_size - 1];
				ref_y = previous_path_y[pre_size - 1];
                ref_x_pre = previous_path_x[pre_size - 2];
				ref_y_pre = previous_path_y[pre_size - 2];
				ref_yaw = atan2(ref_y - ref_y_pre, ref_x - ref_x_pre);

				ptsx.push_back(ref_x_pre);
				ptsy.push_back(ref_y_pre);
				ptsx.push_back(ref_x);
				ptsy.push_back(ref_y);
			}

			vector<double> next_waypoint0 =
					getXY(car_s+30, t3p1help::getDFromLane(p_state.lane_num),
						  map_waypoints_s,
					      map_waypoints_x,
					      map_waypoints_y);
            vector<double> next_waypoint1 =
					getXY(car_s+60, t3p1help::getDFromLane(p_state.lane_num),
						  map_waypoints_s,
					      map_waypoints_x,
					      map_waypoints_y);
            vector<double> next_waypoint2 =
					getXY(car_s+90, t3p1help::getDFromLane(p_state.lane_num),
						  map_waypoints_s,
					      map_waypoints_x,
					      map_waypoints_y);

			ptsx.push_back(next_waypoint0[0]);
			ptsy.push_back(next_waypoint0[1]);
			ptsx.push_back(next_waypoint1[0]);
			ptsy.push_back(next_waypoint1[1]);
			ptsx.push_back(next_waypoint2[0]);
			ptsy.push_back(next_waypoint2[1]);

			// transform coordinates to car's
            double local_x;
			double local_y;
			// cout << "ref_x: " << ref_x << ", ref_y: " << ref_y << endl;
            for (int i=0; i < ptsx.size(); i++) {
				local_x = ptsx[i] - ref_x;
				local_y = ptsy[i] - ref_y;
				/// cout << "ptsx[" << i << "]=" << ptsx[i]
				///	 << ", ptsy[" << i << "]=" << ptsy[i] << endl;
				/// cout << "local_x: " << local_x << ", local_y: " << local_y << endl;
				ptsx[i] = local_x * cos(0 - ref_yaw) - local_y * sin(0-ref_yaw);
				ptsy[i] = local_x * sin(0 - ref_yaw) + local_y * cos(0-ref_yaw);
				/// cout << "ptsx[" << i << "]=" << ptsx[i]
				/// 	 << ", ptsy[" << i << "]=" << ptsy[i] << endl << endl;
			}

			// push back previous waypoints
			for (int i=0; i < pre_size; i++) {
				// cout << "previous_path_x[" << i << "]=" << previous_path_x[i]
				// 	 << ", previous_path_y[" << i << "]=" << previous_path_y[i] << endl;
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			// Idea from Path Planning Walkthrough Video
			// Generating the path in car coordinates, instead of simulator/global or
			// Frenet coordinates. Frenet doesn't have linear one-to-one translation to the
			// global coordinates. That will cause coarse resolutions that produce
			// jitters and jerks.
			tk::spline spline;
			spline.set_points(ptsx, ptsy);

			double horizon_x = p_state.horizon_dist; // meters
			double horizon_y = spline(horizon_x);
			double horizon_dist = sqrt(horizon_x*horizon_x+horizon_y*horizon_y);

			double steps_x = (horizon_dist/
					(p_state.point_dt*p_state.ref_velocity/t3p1help::MPS_TO_MPH));
            double step_x = horizon_x/steps_x;
			double global_x, global_y;
			local_x = 0;

			for (int i=1; i <= p_state.horizon_size-pre_size; i++) {
				local_x += step_x;
				local_y = spline(local_x);

                // transform back to global coordinates
                global_x = ref_x + (local_x * cos(ref_yaw)) - local_y * sin(ref_yaw);
				global_y = ref_y + (local_x * sin(ref_yaw)) + local_y * cos(ref_yaw);

				next_x_vals.push_back(global_x);
				next_y_vals.push_back(global_y);
			}

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}















































































