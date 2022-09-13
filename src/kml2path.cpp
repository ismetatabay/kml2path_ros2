#include <iostream>
#include <thread>
#include <memory>
#include <vector>
#include <exception>
#include <cmath>
#include "kml/base/file.h"
#include "kml/base/math_util.h"
#include "kml/convenience/convenience.h"
#include "kml/dom.h"
#include "kml/engine.h"
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std;
using namespace GeographicLib;
using namespace std::chrono_literals;


using kmldom::CoordinatesPtr;
using kmldom::KmlFactory;
using kmldom::LineStringPtr;
using kmldom::PlacemarkPtr;
using kmldom::KmlPtr;
using kmlengine::KmlFile;
using kmlengine::KmlFilePtr;
using std::cerr;
using std::cout;
using std::endl;

// This FeatureVisitor collects any LineString coordinates contained in the
// KmlFile and appends them to concatenated_coords.
class LineStringVisitor : public kmlengine::FeatureVisitor 
{
 public:
  LineStringVisitor(const KmlFilePtr& kml_file,
                    CoordinatesPtr concatenated_coordinates, Geocentric earth)
    : kml_file_(kml_file),
      earth_(earth),
      concatenated_coordinates_(concatenated_coordinates),
      altitudemode_set_(false),
      altitudemode_(kmldom::ALTITUDEMODE_CLAMPTOGROUND) {
  }

  int get_altitudemode() 
  {
    return altitudemode_;
  }

  // The callback from VisitFeatureHierarchy. If we have a Placemark with
  // a LineString that has coordinates, append the coordinates to a
  // vector.
  virtual void VisitFeature(const kmldom::FeaturePtr& f) 
  {
    pose_msg.header.frame_id = "map";
    path_msg.header.frame_id = "map";
    if (f->Type() == kmldom::Type_Placemark) {
      PlacemarkPtr p = kmldom::AsPlacemark(f);
      if (p->has_geometry() &&
          p->get_geometry()->Type() == kmldom::Type_LineString) {
        LineStringPtr ls = kmldom::AsLineString(p->get_geometry());
        // If this is the first linestring, set the altitudeMode for the final
        // line from here.
        if (!altitudemode_set_) {
          altitudemode_ = ls->get_altitudemode();
          altitudemode_set_ = true;
        }
        if (ls->has_coordinates()) {
          CoordinatesPtr c = ls->get_coordinates();
          LocalCartesian proj(c->get_coordinates_array_at(0).get_latitude(), c->get_coordinates_array_at(0).get_longitude(),
                               c->get_coordinates_array_at(0).get_latitude(), earth_);
          for (size_t i = 1; i < c->get_coordinates_array_size(); ++i) {
            double x, y, z;
            proj.Forward(c->get_coordinates_array_at(i).get_latitude(), c->get_coordinates_array_at(i).get_longitude(),
                           c->get_coordinates_array_at(i).get_altitude(), x, y, z);
            concatenated_coordinates_->add_vec3(c->get_coordinates_array_at(i));
            pose_msg.pose.position.x = x;
            pose_msg.pose.position.y = y;
            pose_msg.pose.position.z = z;
            path_msg.poses.push_back(pose_msg);
          }
        }
      }
    }
  }
  nav_msgs::msg::Path path_msg;
  geometry_msgs::msg::PoseStamped pose_msg;

 private:
  const KmlFilePtr kml_file_;
  Geocentric earth_;
  CoordinatesPtr concatenated_coordinates_;
  bool altitudemode_set_;
  int altitudemode_;
};


class KML2PATH : public rclcpp::Node
{
public:
  KML2PATH() : Node("kml2path_node") {
  publisher_path = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  KmlFilePtr kml_file = KML2PATH::read_kml_file();
  LineStringVisitor linestring_visitor = KML2PATH::parse_kml(kml_file);
  path_msg = linestring_visitor.path_msg;
  KML2PATH::path_publisher();
  timer_ = this->create_wall_timer(
      500ms, std::bind(&KML2PATH::path_publisher, this));
  }

private:
  KmlFilePtr read_kml_file()
  {
    std::string kml_data;
    if (!kmlbase::File::ReadFileToString("src/kml2path_ros2/data/test.kml", &kml_data)) {
      cerr << "error: read of " << "test.kml" << " failed" << endl;
      return (0);
    }

    std::string errors;
    KmlFilePtr kml_file = KmlFile::CreateFromParse(kml_data, &errors);
    if (!kml_file || !errors.empty()) {
      cerr << "parse failed: " << errors << endl;
      return (0);
    }
    return kml_file;
  }
  LineStringVisitor parse_kml(const KmlFilePtr& kmlfile)
  {
    Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
                                              
    KmlFactory* factory = KmlFactory::GetFactory();
    CoordinatesPtr concatenated_coordinates = factory->CreateCoordinates();

    LineStringVisitor linestring_visitor(kmlfile, concatenated_coordinates, earth);
    kmlengine::VisitFeatureHierarchy(
        kmlengine::GetRootFeature(kmlfile->get_root()), linestring_visitor);
    return linestring_visitor;
  }
  void path_publisher() 
  {

    RCLCPP_INFO(this->get_logger(), "Publishing:");
    publisher_path->publish(path_msg);

  }
  nav_msgs::msg::Path path_msg;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_path;
};

int main(int argc, char** argv) 
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KML2PATH>());
  rclcpp::shutdown();
  return (0);
}

