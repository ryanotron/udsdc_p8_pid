#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::cout;
using std::endl;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// csv for (string of) comma separated values, not whole csv file
void parse_csv (string partial, vector<double>& output) {
    string::size_type offset = 0;
    string::size_type sz = 0;

    while (offset < partial.size()) {
        double elem = std::stod(partial.substr(offset), &sz);
        output.push_back(elem);
        offset = offset + sz + 1;
    }
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != string::npos) {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main(int argc, char** argv) {
    string paramfile = "../params.txt";
    if (argc > 1) {
        cout << argv[1] << endl;
        paramfile = argv[1];
    }
    else {
        cout << "using default parameters" << endl;
    }

    uWS::Hub h;

    PID pid;
    PID pid_throttle;

    // setup parameters with default values
    vector<double> steer_gains {0.1, 0.0, 0.0};
    vector<double> throttle_gains {0.1, 0.0, 0.0};
    double vmin = 30.0;
    double vmax = 60.0;
    double vref = vmin;

    // read actual values to be used from parameter file
    string instr;
//    std::ifstream instream ("../params.txt");
    cout << paramfile << endl;
    std::ifstream instream (paramfile);

    if (instream.is_open()) {
        while (std::getline(instream, instr)) {
            string::size_type pos = instr.find(":");
            if (pos == string::npos) {
                continue;
            }

            string partial = instr.substr(pos+1, instr.size()-pos-1);
            string parname = instr.substr(0, pos);

            if (parname == "steer") {
                steer_gains.clear();
                parse_csv(partial, steer_gains);
                cout << parname << ": ";
                for (size_t i = 0; i < steer_gains.size(); i++) {
                    cout << std::fixed << std::setprecision(3) << steer_gains[i] << " ";
                }
                cout << endl;
            }
            else if (parname == "throttle") {
                throttle_gains.clear();
                parse_csv(partial, throttle_gains);
                cout << parname << ": ";
                for (size_t i = 0; i < throttle_gains.size(); i++) {
                    cout << std::fixed << std::setprecision(3) << throttle_gains[i] << " ";
                }
                cout << endl;
            }
            else if (parname == "vmin") {
                vmin = std::stod(partial);
                cout << parname << ": ";
                cout << std::fixed << std::setprecision(2) << vmin << endl;
            }
            else if (parname == "vmax") {
                vmax = std::stod(partial);
                cout << parname << ": ";
                cout << std::fixed << std::setprecision(2) << vmax << endl;
            }
            else {
                cout << "parameter " << parname << " found but not used" << endl;
            }
        }
        cout << "---" << endl;
    }
    else {
        cout << "Failed to open parameter file. Use default parameters" << endl;
    }

    // initialise pid gains with values from parameter file
    pid.Init(steer_gains[0], steer_gains[1], steer_gains[2]);
    pid.set_saturation(1000000.0);
    pid_throttle.Init(throttle_gains[0], throttle_gains[1], throttle_gains[2]);
    pid_throttle.set_saturation(1000000.0);

    h.onMessage([&pid, &pid_throttle, &vref, &vmin, &vmax](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        static double count = 0.0;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(string(data).substr(0, length));

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<string>());
                    double speed = std::stod(j[1]["speed"].get<string>());
                    double angle = std::stod(j[1]["steering_angle"].get<string>());
                    double steer_value;

                    pid.UpdateError(0.0 - cte);
                    pid_throttle.UpdateError(vref - speed);
                    steer_value = pid.TotalError();
                    double u_throttle = pid_throttle.TotalError();

                    // scale speed reference with cross-track error
                    // if error is low, speed up to vmax,
                    // if error is high, slow down to vmin
                    vref = vmax + (vmin - vmax)/(2.0)*fabs(cte);
                    vref = std::min(vmax, vref);
                    vref = std::max(vmin, vref);

                    steer_value = std::min(1.0, steer_value);
                    steer_value = std::max(-1.0, steer_value);

                    // DEBUG, formatted for convenient pasting to a spreadsheet
                    // (or pipe it to a csv)
                    std::cout << std::setw(6) << std::setprecision(0) << count << ", ";
                    std::cout << std::fixed << std::setprecision(3);
                    std::cout << std::setw(8) << 0.0-cte << ", ";
                    std::cout << std::setw(8) << steer_value << ", ";
                    std::cout << std::setw(8) << vref << ", ";
                    std::cout << std::setw(8) << vref-speed << ", ";
                    std::cout << std::setw(8) << u_throttle << endl;
                    count += 1.0;

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = u_throttle;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//                    std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket message if
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
