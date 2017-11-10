#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "tools.h"

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

std::string message_hunter(json::basic_json j, UKF &ukf, double &target_x, double &target_y);
std::string message_sensor(json::basic_json j, UKF &ukf, Tools &tools, vector<VectorXd> &estimations, vector<VectorXd> &ground_truth);

int main(int argc, char **argv)
{
	uWS::Hub h;

// Create a Kalman Filter instance
	UKF ukf;
	double target_x = 0.0;
	double target_y = 0.0;

  // Parse params
	bool catch_car = false;
	if (argc > 1) {
		std::string param = argv[1];
		if (param == "catch") {
			catch_car = true;
		}
	}

  // used to compute the RMSE later
	Tools tools;
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	h.onMessage([&ukf,&tools,&estimations,&ground_truth,&target_x,&target_y,&catch_car](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          if (catch_car) {
            // Parse catch car message
            auto msg = message_hunter(j, ukf, target_x, target_y);
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					} else {
            // Parse sensor fusion message
            auto msg = message_sensor(j, ukf, tools, estimations, ground_truth);
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					}

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
		else {
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

std::string message_hunter(json::basic_json j, UKF &ukf, double &target_x, double &target_y)
{
	double hunter_x = std::stod(j[1]["hunter_x"].get<std::string>());
	double hunter_y = std::stod(j[1]["hunter_y"].get<std::string>());
	double hunter_heading = std::stod(j[1]["hunter_heading"].get<std::string>());

	string lidar_measurment = j[1]["lidar_measurement"];

	MeasurementPackage meas_package_L;
	istringstream iss_L(lidar_measurment);
	long long timestamp_L;

  // reads first element from the current line
	string sensor_type_L;
	iss_L >> sensor_type_L;

  // read measurements at this timestamp
	meas_package_L.sensor_type_ = MeasurementPackage::LASER;
	meas_package_L.raw_measurements_ = VectorXd(2);
	float px;
	float py;
	iss_L >> px;
	iss_L >> py;
	meas_package_L.raw_measurements_ << px, py;
	iss_L >> timestamp_L;
	meas_package_L.timestamp_ = timestamp_L;

	ukf.ProcessMeasurement(meas_package_L);

	string radar_measurment = j[1]["radar_measurement"];

	MeasurementPackage meas_package_R;
	istringstream iss_R(radar_measurment);
	long long timestamp_R;

  // reads first element from the current line
	string sensor_type_R;
	iss_R >> sensor_type_R;

  // read measurements at this timestamp
	meas_package_R.sensor_type_ = MeasurementPackage::RADAR;
	meas_package_R.raw_measurements_ = VectorXd(3);
	float ro;
	float theta;
	float ro_dot;
	iss_R >> ro;
	iss_R >> theta;
	iss_R >> ro_dot;
	meas_package_R.raw_measurements_ << ro,theta, ro_dot;
	iss_R >> timestamp_R;
	meas_package_R.timestamp_ = timestamp_R;

	ukf.ProcessMeasurement(meas_package_R);

	target_x = ukf.x_[0];
	target_y = ukf.x_[1];

	double distance_difference = sqrt((target_y - hunter_y)*(target_y - hunter_y) + (target_x - hunter_x)*(target_x - hunter_x));

	// Update targer position to the moment when hunter reaches the car
	target_x += distance_difference * cos(ukf.x_[3]);
	target_y += distance_difference * sin(ukf.x_[3]);

	double heading_to_target = atan2(target_y - hunter_y, target_x - hunter_x);
	while (heading_to_target > M_PI) heading_to_target-=2.*M_PI;
	while (heading_to_target <-M_PI) heading_to_target+=2.*M_PI;
  //turn towards the target
	double heading_difference = heading_to_target - hunter_heading;
	while (heading_difference > M_PI) heading_difference-=2.*M_PI;
	while (heading_difference <-M_PI) heading_difference+=2.*M_PI;

	json msgJson;
	msgJson["turn"] = heading_difference;
	msgJson["dist"] = distance_difference;
	auto msg = "42[\"move_hunter\"," + msgJson.dump() + "]";
	return msg;
  // std::cout << msg << std::endl;
}

std::string message_sensor(json::basic_json j, UKF &ukf, Tools &tools, vector<VectorXd> &estimations, vector<VectorXd> &ground_truth) {

	// j[1] is the data JSON object
	string sensor_measurment = j[1]["sensor_measurement"];

	MeasurementPackage meas_package;
	istringstream iss(sensor_measurment);
	long long timestamp;

	// reads first element from the current line
	string sensor_type;
	iss >> sensor_type;

	if (sensor_type.compare("L") == 0) {
		meas_package.sensor_type_ = MeasurementPackage::LASER;
		meas_package.raw_measurements_ = VectorXd(2);
		float px;
		float py;
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
	ground_truth.push_back(gt_values);

	//Call ProcessMeasurment(meas_package) for Kalman filter
	ukf.ProcessMeasurement(meas_package);

	//Push the current estimated x,y positon from the Kalman filter's state vector

	VectorXd estimate(4);

	double p_x = ukf.x_(0);
	double p_y = ukf.x_(1);
	double v  = ukf.x_(2);
	double yaw = ukf.x_(3);

	double v1 = cos(yaw)*v;
	double v2 = sin(yaw)*v;

	estimate(0) = p_x;
	estimate(1) = p_y;
	estimate(2) = v1;
	estimate(3) = v2;

	estimations.push_back(estimate);

	VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

	json msgJson;
	msgJson["estimate_x"] = p_x;
	msgJson["estimate_y"] = p_y;
	msgJson["rmse_x"] =  RMSE(0);
	msgJson["rmse_y"] =  RMSE(1);
	msgJson["rmse_vx"] = RMSE(2);
	msgJson["rmse_vy"] = RMSE(3);
	auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
	// std::cout << msg << std::endl;

	return msg;
}
