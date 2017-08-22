//------------------------------------------------------------------------------
// Author: Lukasz Janyst <lukasz@jany.st>
// Date:   06.08.2017
//------------------------------------------------------------------------------

#include "map.hh"
#include "auto_pilot.hh"

#include <stdexcept>
#include <iostream>
#include <uWS/uWS.h>
#include <vector>
#include <string>
#include <json.hpp>

using json = nlohmann::json;

//------------------------------------------------------------------------------
// Constants
//------------------------------------------------------------------------------
const double kSpeedLimit = 22.2; // m/s

//------------------------------------------------------------------------------
// Message processor
//------------------------------------------------------------------------------
class MessageHandler {
  public:
    //--------------------------------------------------------------------------
    //! Constructor
    //--------------------------------------------------------------------------
    MessageHandler(const Map &map): pilot_(map, kSpeedLimit) {}

   //---------------------------------------------------------------------------
   //! Message processor
   //---------------------------------------------------------------------------
   void operator() (uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
                    uWS::OpCode opCode) {

    //--------------------------------------------------------------------------
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message, the 2 signifies a websocket event.
    //--------------------------------------------------------------------------
    if(length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = HasData(std::string(data).substr(0, length));

      //------------------------------------------------------------------------
      // Autonomous driving
      //------------------------------------------------------------------------
      if(s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {

          //--------------------------------------------------------------------
          // Decode the current state of the vehicle
          //--------------------------------------------------------------------
          auto v = Vehicle(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"],
                           Deg2Rad(j[1]["yaw"]), MPH2ms(j[1]["speed"]));

          //--------------------------------------------------------------------
          // Decode theprevious path
          //--------------------------------------------------------------------
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          std::vector<Eigen::VectorXd> previous_path;
          for(int i = 0; i < previous_path_x.size(); ++i) {
            Eigen::VectorXd pt(2);
            pt << previous_path_x[i], previous_path_y[i];
            previous_path.push_back(pt);
          }

          //--------------------------------------------------------------------
          // Decode the sensor fusion data
          //--------------------------------------------------------------------
          auto sensor_fusion_data = j[1]["sensor_fusion"];
          std::vector<SFObject> sensor_fusion;
          for(auto &d: sensor_fusion_data) {
            auto o = SFObject(d[0], d[1], d[2], d[5], d[6], d[3], d[4]);
            sensor_fusion.push_back(o);
          }

          //--------------------------------------------------------------------
          // Plan the new path
          //--------------------------------------------------------------------
          auto new_path = pilot_.ComputePath(v, sensor_fusion, previous_path);

          //--------------------------------------------------------------------
          // Send out the new path
          //--------------------------------------------------------------------
          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

          for(auto &pt: new_path) {
            next_x_vals.push_back(pt[0]);
            next_y_vals.push_back(pt[1]);
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      //------------------------------------------------------------------------
      // Manual driving
      //------------------------------------------------------------------------
      else {
        std::string msg = "42[\"manual\",{}]";
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  }

  private:
    //--------------------------------------------------------------------------
    // Checks if the websock event has JSON data. If there is data the JSON
    // object in string format will be returned, else the empty string "" will
    // be returned.
    //--------------------------------------------------------------------------
    std::string HasData(const std::string &s) {
      auto found_null = s.find("null");
      auto b1         = s.find_first_of("[");
      auto b2         = s.find_last_of("]");
      if (found_null != std::string::npos)
        return "";
      else if (b1 != std::string::npos && b2 != std::string::npos)
        return s.substr(b1, b2 - b1 + 1);
      return "";
    }

    AutoPilot  pilot_;
};

//------------------------------------------------------------------------------
// Start the show
//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  //----------------------------------------------------------------------------
  // Read the map
  //----------------------------------------------------------------------------
  std::string map_filename = "../data/highway_map.csv";
  bool        interpolate  = true;
  if(argc >= 2)
    map_filename = argv[1];
  if(argc >= 3)
    if(std::string("nointerpolate") == argv[2])
      interpolate = false;

  Map map;
  try {
    map.LoadFromFile(map_filename, interpolate);
  }
  catch(std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  uWS::Hub h;
  //----------------------------------------------------------------------------
  // Install callbacks
  //----------------------------------------------------------------------------
  MessageHandler msg_handler(map);
  h.onMessage(
    [&msg_handler]
    (uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
      uWS::OpCode opCode) {
      msg_handler(ws, data, length, opCode);
    });
  h.onConnection(
    [&msg_handler]
    (uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
      std::cout << "Connected" << std::endl;
    });

  h.onDisconnection(
    [&msg_handler]
    (uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) {
      std::cout << "Disconnected" << std::endl;
    });


  //----------------------------------------------------------------------------
  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  //----------------------------------------------------------------------------
  h.onHttpRequest(
    [](uWS::HttpResponse *res, uWS::HttpRequest req, char *, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
      res->end(s.data(), s.length());
    else
      res->end(nullptr, 0);
    });

  //----------------------------------------------------------------------------
  // Run the server
  //----------------------------------------------------------------------------
  int port = 4567;
  if(h.listen(port))
    std::cout << "Listening to port " << port << std::endl;
  else {
    std::cerr << "Failed to listen to port: " << port << std::endl;
    return -1;
  }
  h.run();
}
