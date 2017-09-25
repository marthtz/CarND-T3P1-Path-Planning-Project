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

using namespace std;

// for convenience
using json = nlohmann::json;


//==============================================================================
// Speed limit of the track
constexpr double speedlimit() { return 49.5; }

//==============================================================================
// Conversion factor from kmh to mpg
constexpr double convert_kmh_to_mph() { return 2.237; }

//==============================================================================
// Conversion factor from kmh to mpg
constexpr double time_step() { return 0.02; }

//==============================================================================
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

constexpr int lane_width = 4;

//==============================================================================
// Get the lane from Frenet coordinates
int get_lane_from_frenet(double d)
{
  return int(floor(d / lane_width));
}


//==============================================================================
double deg2rad(double x)
{
  return x * pi() / 180;
}


//==============================================================================
double rad2deg(double x)
{
  return x * 180 / pi();
}



//==============================================================================
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
//==============================================================================
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");

  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }

  return "";
}



//==============================================================================
//==============================================================================
double distance(double x1, double y1, double x2, double y2)
{
  return sqrt( ((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)) );
}



//==============================================================================
//==============================================================================
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);

    if (dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}



//==============================================================================
//==============================================================================
int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y - y), (map_x - x) );

  double angle = abs(theta - heading);

  if (angle > pi()/4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}


//==============================================================================
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
//==============================================================================
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);
  int prev_wp;

  prev_wp = next_wp-1;

  if (next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];

  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = ((x_x * n_x) + (x_y * n_y)) / ((n_x * n_x) + (n_y * n_y));
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};

}


//==============================================================================
// Transform from Frenet s,d coordinates to Cartesian x,y
//==============================================================================
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                         (maps_x[wp2] - maps_x[prev_wp]));

  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + (seg_s * cos(heading));
  double seg_y = maps_y[prev_wp] + (seg_s * sin(heading));

  double perp_heading = heading - (pi() / 2);

  double x = seg_x + (d * cos(perp_heading));
  double y = seg_y + (d * sin(perp_heading));

  return {x, y};

}


//==============================================================================
// Main
//==============================================================================
int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  int ego_lane = 1;
  int lane_change_wp = 0;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&ego_lane,&lane_change_wp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;



    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double ego_car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double ego_car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          //cout << "Car speed 1: " << ego_car_speed << endl;

//==============================================================================
//==============================================================================
//==============================================================================
//==============================================================================
          int prev_size = previous_path_x.size();
          int next_wp;
          bool too_close = false;
          bool change_lane = false;
          bool lcl = false;
          bool lcr = false;

          double distance_closest_check = 100000;
          double closest_speed = 100000;

          // reference x, y, yaw states
          // either we will reference the starting points as where the is or at the previous paths and point
          double ref_x;
          double ref_y;
          double ref_x_prev;
          double ref_y_prev;
          double ref_yaw;

          // Set reference speed to speed limit of track
          double ref_vel = speedlimit();


          if (prev_size > 0)
          {
            ego_car_s = end_path_s;

            if(prev_size > 2)
            {
              ref_x      = previous_path_x[prev_size-1];
              ref_x_prev = previous_path_x[prev_size-2];

              ref_y      = previous_path_y[prev_size-1];
              ref_y_prev = previous_path_y[prev_size-2];

              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              next_wp = NextWaypoint(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);

              // Calculate real speed of the ego car from previous path points
              ego_car_speed = (sqrt((ref_x - ref_x_prev) * (ref_x - ref_x_prev) +
                                    (ref_y - ref_y_prev) * (ref_y - ref_y_prev)) / time_step());
              // Convert speed to mph
              ego_car_speed *= convert_kmh_to_mph();
              //cout << "Car speed 2: " << ego_car_speed << endl;
            }
          }
          else
          {
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);
            next_wp = NextWaypoint(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
          }




          // find ref_v to use
          for (int i=0; i<sensor_fusion.size(); i++)
          {
            // car is in my lane
            float d = sensor_fusion[i][6];

            if (get_lane_from_frenet(d) == ego_lane)
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];

              double check_car_speed = sqrt((vx * vx) + (vy * vy));
              double check_car_s = sensor_fusion[i][5];

              check_car_s += ((double)prev_size * time_step() * check_car_speed);  // if using previous points can project s value out

              double distance_ego_to_check = check_car_s - ego_car_s;

              // check s values greater than ego car and s gap
              if((check_car_s > ego_car_s) &&
                 (distance_ego_to_check < 30) &&
                 (distance_ego_to_check < distance_closest_check ))
              {
                distance_closest_check = distance_ego_to_check;
                ref_vel = check_car_speed * convert_kmh_to_mph();

                change_lane = true;
                cout << "trigger lane change" << endl;

                if (distance_closest_check < 20)
                {
                  ref_vel -= 5;
                  cout << "too close" << endl;
                }
              }
            }
          }



          if (ref_vel > ego_car_speed)
          {
            ego_car_speed += 0.4;
            //cout << "Car speed 3: " << ego_car_speed << endl;
          }
          else if(ref_vel < ego_car_speed)
          {
            ego_car_speed -= 0.4;
            //cout << "Car speed 4: " << ego_car_speed << endl;
          }



          //try to change lanes if too close to car in front
          if (change_lane)
          {
            if((next_wp-lane_change_wp) % map_waypoints_x.size() > 2)
            {
              bool changed_lanes = false;
              //first try to change to left lane
              if(ego_lane != 0 && !changed_lanes)
              {
                bool lane_safe = true;
                for(int i = 0; i < sensor_fusion.size(); i++)
                {
                  //car is in left lane
                  float d = sensor_fusion[i][6];
                  if (get_lane_from_frenet(d) == (ego_lane-1))
                  {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_car_speed = sqrt(vx*vx+vy*vy);

                    double check_car_s = sensor_fusion[i][5];
                    check_car_s += ((double)prev_size * time_step() * check_car_speed);

                    double distance_ego_to_check = check_car_s - ego_car_s;

                    if((distance_ego_to_check < 20) &&
                       (distance_ego_to_check > -10))
                    {
                      lane_safe = false;
                    }
                  }
                }
                if(lane_safe)
                {
                  changed_lanes = true;
                  ego_lane -= 1;
                  lane_change_wp = next_wp;
                }
              }
              //next try to change to right lane
              if(ego_lane != 2 && !changed_lanes)
              {
                bool lane_safe = true;
                for(int i = 0; i < sensor_fusion.size(); i++)
                {
                  //car is in right lane
                  float d = sensor_fusion[i][6];
                  if (get_lane_from_frenet(d) == (ego_lane+1))
                  {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_car_speed = sqrt(vx*vx+vy*vy);

                    double check_car_s = sensor_fusion[i][5];
                    check_car_s += ((double)prev_size * time_step() * check_car_speed);

                    double distance_ego_to_check = check_car_s - ego_car_s;

                    if((distance_ego_to_check < 20) &&
                       (distance_ego_to_check > -10))
                    {
                      lane_safe = false;
                    }
                  }
                }
                if(lane_safe)
                {
                  changed_lanes = true;
                  ego_lane += 1;
                  lane_change_wp = next_wp;
                }
              }
            }
          }


          // if (lcl)
          // {
          //   lane -= 1;
          // }
          // else if (lcr)
          // {
          //   lane += 1;
          // }




          // create list of widely spaces x, y waypoints, evenly spaced at 30m
          // later we will interpolate these waypoints with a spline and fill it in with more points that control speed
          vector<double> ptsx;
          vector<double> ptsy;

          // if previous size is almost empty
          if (prev_size < 2)
          {
            // use 2 points that make the path tanget to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            // use the previous path's end point as starting reference

            // redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];

            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // use two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          //cout << "Lane " << lane << endl;

          // in frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(ego_car_s + 1*30, (2+4*ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(ego_car_s + 2*30, (2+4*ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(ego_car_s + 3*30, (2+4*ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);


          for (int i = 0; i < ptsx.size(); i++)
          {
            // shoft car reference angle to 0 degree
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }


          // create a spline
          tk::spline s;

          // set x, y points to the spline
          s.set_points(ptsx, ptsy);

          // define the actual x, y points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }


          // calculate how to break up spline points so that we travel at our desired velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;

          // fill up the rest of our path planner after filling if with previous points, here we will always ouput 50 points
          //cout << "prev path size: " << previous_path_x.size() << endl;
          for (int i = 1; i <= 50 - previous_path_x.size(); i++)
          {
            double N = (target_dist / (time_step() * ego_car_speed/convert_kmh_to_mph()));
            double x_point = x_add_on + (target_x / N);
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after roatating it earlier
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

//==============================================================================
//==============================================================================
//==============================================================================
//==============================================================================


          json msgJson;
          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
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
