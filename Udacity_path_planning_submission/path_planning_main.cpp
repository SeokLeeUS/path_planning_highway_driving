#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h" // add spline.h ( source: https://kluge.in-chemnitz.de/opensource/spline/spline.h)
using namespace std;
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
  
  // start in lane 1;
  int lane = 1;
  // have a reference velocity to target
  double ref_vel = 0.0; //mph to generate target speed to the simulation car 
  
  double weightfactor0{1.0};
  double weightfactor1{1.0};
  double weightfactor2{1.0};
  //  https://boycoding.tistory.com/181
  //int lane_flag = 99;
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel,&weightfactor0,&weightfactor1,&weightfactor2]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
                
    // start in lane 1;
    //int lane = 1;
    // have a reference velocity to target
    //double ref_vel = 49.5; //mph      
                
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          int prev_size = previous_path_x.size(); // setting previous size


          if (prev_size > 0)
          {
            car_s = end_path_s;
          }
          
          bool too_close = false;
          // weighting factor initialization
          //int weightfactor0 = 1;
          //int weightfactor1 = 1;
          //int weightfactor2 = 1;
          //def_weight();
          //int weightfactor0{};
          //int weightfactor1{};
          //int weightfactor2{};
          //  https://boycoding.tistory.com/181         
          //def_weight();
          int lane_flag = 99;
          double weight_tol = 0.05;
          double target_speed = 49.5 * 0.44704; 
          double dist_cost = 1.0;
          double f_g = 10; //front gap
          double r_g = 20;// rear gap
          //49.5(50-0.5(buffer))mph, 0.44704 (conversion factor, mph->mps)

          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            //car is in my lane
            float d = sensor_fusion[i][6];
            
            
            //bool lane_1_0 = (d < (2 + 4 * (lane - 1) + 2) && d >(2 + 4 * (lane - 1) - 2));
            //bool lane_1_2 = (d < (2 + 4 * (lane + 1) + 2) && d >(2 + 4 * (lane + 1) - 2));
            //bool lane_0_1 = (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2));
            //bool lane_2_1 = (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2));
            
            bool lane_0 = (d < (2 + 4 * (1 - 1) + 2) && d >(2 + 4 * (1 - 1) - 2));
            bool lane_2 = (d < (2 + 4 * (1 + 1) + 2) && d >(2 + 4 * (1 + 1) - 2));
            bool lane_1 = (d < (2 + 4 * 1 + 2) && d >(2 + 4 * 1 - 2));
            //bool lane_2_1 = (d < (2 + 4 * 1 + 2) && d >(2 + 4 * 1 - 2));
            //bool in_my_lane = d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2);
 
            
            
            // define weighting factor per case by projecting s for the vehicle in front of you per lane
            
            if (lane_1) 
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_f = sensor_fusion[i][5];
              double check_car_b = sensor_fusion[i][5];
              check_car_f += ((double)prev_size * .02 * check_speed); // project next s point 
              check_car_b -= ((double)prev_size * .02 * check_speed); //car behind
              // checking gap between front/back cars
              if ((check_car_f>car_s)&&((check_car_f-car_s)>f_g)||((check_car_b<car_s)&&(car_s-check_car_b)>r_g))  
              {
                dist_cost = exp(-abs((check_car_f-car_s)/30));
                weightfactor1  = abs((target_speed-check_speed)/target_speed) + dist_cost; // move to lane 0

              }
              else
              {
                weightfactor1 = 1.0;
              }
            }
            
            else if (lane_1) 
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_f = sensor_fusion[i][5];
              double check_car_b = sensor_fusion[i][5];
              
              check_car_f += ((double)prev_size * .02 * check_speed); // project next s point 
              check_car_b -= ((double)prev_size * .02 * check_speed); //car behind
              
              if ((check_car_f>car_s)&&((check_car_f-car_s)>f_g)||((check_car_b<car_s)&&(car_s-check_car_b)>r_g)) 
              {
                dist_cost = exp(-abs((check_car_f-car_s)/30));
                weightfactor1  = abs((target_speed - check_speed) / target_speed) + dist_cost; 
              }
              else
              {
                 weightfactor1 = 1.0;
                
              }
            }            
            else if (lane_0)  
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_f = sensor_fusion[i][5];
              double check_car_b = sensor_fusion[i][5];
              
              check_car_f += ((double)prev_size * .02 * check_speed); // project next s point 
              check_car_b -= ((double)prev_size * .02 * check_speed); //car behind 
              
              if ((check_car_f>car_s)&&((check_car_f-car_s)>f_g)||((check_car_b<car_s)&&(car_s-check_car_b)>r_g)) 
              {
                dist_cost = exp(-abs((check_car_f-car_s)/30));
                weightfactor0  = abs((target_speed - check_speed) / target_speed) + dist_cost; 
                
              }
              else
              {
                weightfactor0 = 1.0;
               
              }
            }
            else if (lane_0)  
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_f = sensor_fusion[i][5];
              double check_car_b = sensor_fusion[i][5];
              
              check_car_f += ((double)prev_size * .02 * check_speed); // project next s point 
              check_car_b -= ((double)prev_size * .02 * check_speed); //car behind 
              
              if ((check_car_f>car_s)&&((check_car_f-car_s)>f_g)||((check_car_b<car_s)&&(car_s-check_car_b)>r_g)) 
              {
                dist_cost = exp(-abs((check_car_f-car_s)/30));
                weightfactor0  = abs((target_speed - check_speed) / target_speed) + dist_cost; 
                
              }
              else
              {
                weightfactor0 = 1.0;
                
              }
            }
            
            else if (lane_2)
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_f = sensor_fusion[i][5];
              double check_car_b = sensor_fusion[i][5];
              
              check_car_f += ((double)prev_size * .02 * check_speed); // project next s point 
              check_car_b -= ((double)prev_size * .02 * check_speed); //car behind 
              
              if ((check_car_f>car_s)&&((check_car_f-car_s)>f_g)||((check_car_b<car_s)&&(car_s-check_car_b)>r_g))
              {
                dist_cost = exp(-abs((check_car_f-car_s)/30));
                weightfactor2 = abs((target_speed - check_speed) / target_speed) + dist_cost;               
              }
              else
              {
                weightfactor2 = 1.0;              
              }
            }
            
            else if (lane_2)
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_f = sensor_fusion[i][5];
              double check_car_b = sensor_fusion[i][5];
              
              check_car_f += ((double)prev_size * .02 * check_speed); // project next s point 
              check_car_b -= ((double)prev_size * .02 * check_speed); //car behind 
              
              if ((check_car_f>car_s)&&((check_car_f-car_s)>f_g)||((check_car_b<car_s)&&(car_s-check_car_b)>r_g))
              {
                dist_cost = exp(-abs((check_car_f-car_s)/30));
                weightfactor2 = abs((target_speed - check_speed) / target_speed) + dist_cost;               
              }
              else
              {
                weightfactor2 = 1.0; 
              }
            }
            
            
            
            
            
            // finite state machine
            if (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2)) // when a vehicle in front of you
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_s = sensor_fusion[i][5];
              check_car_s += ((double)prev_size * .02 * check_speed); // project next s point
              
              // finite state machine         
              
              double designated_wf = abs((target_speed - check_speed) / target_speed);
              double desig_dist_cost = exp(-abs((check_car_s-car_s)/30));
              
              if (lane == 1) 
                {
                  weightfactor1 = designated_wf + desig_dist_cost;
                }
                else if (lane == 2)           
                {
                  weightfactor2 = designated_wf + desig_dist_cost;
                } 
                else if (lane == 0)
                {
                  weightfactor0 = designated_wf + desig_dist_cost;                 
                }
             
                
              
              if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))
              {
                //ref_vel = 29.5;
                too_close = true;       
                
 
                if ((lane == 1) && (!lane_0) && (!lane_2))
                {
                  //weightfactor1 = designated_wf;
                  if ((weightfactor0 < weightfactor2) && (abs(weightfactor0 - weightfactor2) > weight_tol))
                  {
                    lane = 0;
                    lane_flag = 0;
                  }
                  else if ((weightfactor0 > weightfactor2) && (abs(weightfactor0 - weightfactor2)>weight_tol))
                  {
                    lane = 2;
                    lane_flag = 1;
                  }
                  else if ((weightfactor0 == 1) && (weightfactor2 == 1)) // dummy testing 
                  {
                    lane = 1;
                    lane_flag = 1.1;
                  }
                  else
                  {
                    lane = 1;
                    lane_flag = 99999;
                  }
                }
                else if ((lane == 1) && (!lane_0) && (lane_2))
                {
                  //weightfactor1 = designated_wf;
                  if ((weightfactor0 < weightfactor1) && (abs(weightfactor0 - weightfactor1) > weight_tol))
                  {
                    lane = 0;
                    lane_flag = 3;
                  }
                  else
                  {
                    lane = 1;
                    lane_flag = 4;
                  }
                }
                else if ((lane == 1) && (lane_0) && (!lane_2))
                {
                  //weightfactor1 = designated_wf;
                  if ((weightfactor2 < weightfactor1) && (abs(weightfactor2 - weightfactor1) > weight_tol))
                  {
                    lane = 2;
                    lane_flag = 5;
                  }
                  else
                  {
                    lane = 1;
                    lane_flag = 6;
                  }
                }
                else if ((lane == 0) && (lane_1))
                {
                  //weightfactor0 = designated_wf;
                  lane = 0;
                  lane_flag = 7;
                }
                else if ((lane == 0) && (!lane_1))
                {
                  //weightfactor0 = designated_wf;
                  if ((weightfactor0 < weightfactor1) && (abs(weightfactor0 - weightfactor1) > weight_tol))
                  {
                    lane = 0;
                    lane_flag = 8;
                  }
                  else if ((weightfactor0 == 0) && (weightfactor1 ==0))// dummy
                  {
                    lane = 1;
                    lane_flag = 8.1;
                  }
                  else
                  {
                    lane = 1;
                    lane_flag = 9;
                  }
                }
                else if ((lane == 2) && (lane_1))
                {
                  //weightfactor2 = designated_wf;
                  lane = 2;
                  lane_flag = 10;
                }
                else if ((lane == 2) && (!lane_1))
                {
                  //weightfactor2 = designated_wf;
                  if ((weightfactor1 < weightfactor2)&& (abs(weightfactor1 - weightfactor2) > weight_tol))
                  {
                    lane = 1;
                    lane_flag = 11;
                  }
                  else if ((weightfactor1 == 0) &&(weightfactor2 ==0)) // dummy rare corner case
                  {
                    lane = 1;
                    lane_flag = 11.1;
                  }
                  else
                  {
                    lane = 2;
                    lane_flag = 12;
                  }
                }
              }
            }
            cout << "cl:"<<too_close<<",ln: "<<lane<<",d: "<<d<<",l_f:"<<lane_flag<<",L0: "<<lane_0<<",L2: "<<lane_2<<",L1:"<<lane_1<<",wf: "<<"["<<weightfactor0<<","<<weightfactor1<<","<<weightfactor2<<"]"<<"end_d: "<<end_path_d<<endl;
          }
          
          
          
          if(too_close)
          {
            ref_vel -= .224;
          }
          else if (ref_vel<49.5)
          {
            ref_vel += .224;
          }
          
          

          //vector<double> next_x_vals;
          //vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          
          if (prev_size <2)
          {
            double prev_car_x = car_x- cos(ref_yaw); //change from car_yaw to ref_yaw
            double prev_car_y = car_y- sin(ref_yaw); // change from car_yaw to ref_yaw
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);  
          }
          else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          vector<double> next_Wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_Wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_Wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          
          ptsx.push_back(next_Wp0[0]);
          ptsx.push_back(next_Wp1[0]);
          ptsx.push_back(next_Wp2[0]);
          
          ptsy.push_back(next_Wp0[1]);
          ptsy.push_back(next_Wp1[1]);
          ptsy.push_back(next_Wp2[1]);
          
          
          for (int i = 0;i<ptsx.size();++i)
          {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            
            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));            
          }
          
                       
          // create a spline
          tk::spline s;
                       
          // set (x,y) points to the spline
          s.set_points(ptsx,ptsy);
          
          //define the actual (x,y) points we will use for the planner
                       
          vector<double> next_x_vals;
          vector<double> next_y_vals;
                       
          for(int i=0;i<previous_path_x.size();++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
                       
          // calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x) * (target_x)+(target_y) * (target_y));
          double x_add_on = 0;
          
          for (int i = 1; i <= 50 - previous_path_x.size(); i++)
          {
            double N = (target_dist / (.02 * ref_vel / 2.24));
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);
            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;

          // rotate back to normal after rotating it earlier
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
                  
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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