#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "fusion_ukf.h"
#include <limits>

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // Create an Estimator instance
  FusionEstimator *estimator = new FusionUKF();
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  ofstream myfile;
  myfile.open("ukf_fusion.txt");


  h.onMessage([&estimator,&estimations,&ground_truth,&myfile](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if (s != "") {
      	
        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          string sensor_measurment = j[1]["sensor_measurement"];
          
          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
    	  long long timestamp;

    	  // reads first element from the current line
    	  string sensor_type;
    	  iss >> sensor_type;
        float px;
        float py;
    	  if (sensor_type.compare("L") == 0) {
      	  		meas_package.sensor_type_ = MeasurementPackage::LASER;
          		meas_package.raw_measurements_ = VectorXd(2);
          		iss >> px;
          		iss >> py;
          		meas_package.raw_measurements_ << px, py;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          } else if (sensor_type.compare("R") == 0) {

      	  		meas_package.sensor_type_ = MeasurementPackage::RADAR;
          		meas_package.raw_measurements_ = VectorXd(3);
          		float ro;
      	  		float theta;
      	  		float ro_dot;
          		iss >> ro;
          		iss >> theta;
          		iss >> ro_dot;
              px = ro*cos(theta);
              py = ro*sin(theta);
          		meas_package.raw_measurements_ << ro,theta, ro_dot;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          }
          float x_gt;
    	  float y_gt;
    	  float vx_gt;
    	  float vy_gt;
    	  iss >> x_gt;
    	  iss >> y_gt;
    	  iss >> vx_gt;
    	  iss >> vy_gt;
    	  VectorXd gt_values(4);
    	  gt_values(0) = x_gt;
    	  gt_values(1) = y_gt; 
    	  gt_values(2) = vx_gt;
    	  gt_values(3) = vy_gt;

        double p_x,p_y,v,yaw,v1,v2;
        VectorXd estimate = estimator->GetEstimate(meas_package);
    	  if(estimate(0)==std::numeric_limits<float>::max()){
          //std::cout << "Skipped estimate" << std::endl;
          if (!estimations.empty()){
            estimate = estimations.back();    // Take the last estimate
          }
          else{   
            json msgJson;
            // Return nothing
            msgJson["estimate_x"] = 0;
            msgJson["estimate_y"] = 0;
            msgJson["rmse_x"] =  0;
            msgJson["rmse_y"] =  0;
            msgJson["rmse_vx"] = 0;
            msgJson["rmse_vy"] = 0;
            auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            return;
          }
        } 
        else
        {
          p_x = estimate(0);
          p_y = estimate(1);
          v =   estimate(2);
          yaw = estimate(3);
          estimate = VectorXd(4);
          v1 =  cos(yaw)*v;
          v2 =  sin(yaw)*v;
          estimate(0) = p_x;
          estimate(1) = p_y;
          estimate(2) = v1;
          estimate(3) = v2;
          estimations.push_back(estimate);
          ground_truth.push_back(gt_values);
          // ['px_est','py_est','vx_est','vy_est','px_meas','py_meas','px_gt','py_gt','vx_gt','vy_gt']
          myfile <<  estimate(0) << "\t" << estimate(1) << "\t"; 
          myfile <<  estimate(2) << "\t" << estimate(3) << "\t";
          myfile <<  px << "\t" << py << "\t";
          myfile <<  gt_values(0) << "\t" << gt_values(1) << "\t"; 
          myfile <<  gt_values(2) << "\t" << gt_values(3) << std::endl;
        }
        VectorXd RMSE = estimator->CalculateRMSE(estimations, ground_truth);
          json msgJson;
          msgJson["estimate_x"] = estimate(0);
          msgJson["estimate_y"] = estimate(1);
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  
        }
      } else {
        
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }

  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}























































































